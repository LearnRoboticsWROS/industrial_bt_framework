// Generic scene manager — robot- and application-agnostic replacement for
// hand-written scene_handling.cpp style nodes. Loads a list of collision
// objects from YAML-style ROS parameters and exposes services to remove and
// teleport them at runtime. Dynamic objects are republished on /planning_scene
// at a configurable rate to defeat any stale scene source.
//
// Services:
//   /scene/remove_object      industrial_bt_framework/srv/RemoveSceneObject
//   /scene/set_object_pose    industrial_bt_framework/srv/SetScenePose

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <boost/variant/get.hpp>

#include "industrial_bt_framework/srv/remove_scene_object.hpp"
#include "industrial_bt_framework/srv/set_scene_pose.hpp"

using ApplySceneClient = rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>;

namespace industrial_bt_framework {

struct SceneObjectSpec
{
  std::string id;
  std::string type;            // "box" | "cylinder" | "sphere" | "mesh"
  std::vector<double> size;    // box
  double radius = 0.0;
  double height = 0.0;
  std::string mesh_path;
  std::vector<double> scale = {1.0, 1.0, 1.0};
  std::vector<double> position    = {0.0, 0.0, 0.0};
  std::vector<double> orientation = {0.0, 0.0, 0.0, 1.0};
  bool dynamic = false;
};

class SceneManagerNode : public rclcpp::Node
{
public:
  SceneManagerNode() : Node("scene_manager_node")
  {
    declareParams();

    client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    apply_client_ = create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene", rmw_qos_profile_services_default, client_cb_group_);
    scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

    remove_srv_ = create_service<industrial_bt_framework::srv::RemoveSceneObject>(
      "/scene/remove_object",
      std::bind(&SceneManagerNode::handleRemove, this,
                std::placeholders::_1, std::placeholders::_2));
    set_pose_srv_ = create_service<industrial_bt_framework::srv::SetScenePose>(
      "/scene/set_object_pose",
      std::bind(&SceneManagerNode::handleSetPose, this,
                std::placeholders::_1, std::placeholders::_2));

    init_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SceneManagerNode::loadInitialScene, this));

    const double hz = get_parameter("force_republish_hz").as_double();
    const int period_ms = (hz > 0.0) ? static_cast<int>(1000.0 / hz) : 1000;
    force_timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&SceneManagerNode::reassertDynamic, this));
  }

private:
  void declareParams()
  {
    declare_parameter<std::string>("frame_id", "base_link");
    declare_parameter<double>("force_republish_hz", 1.0);
    declare_parameter<int>("scene_update_wait_ms", 800);
    declare_parameter<std::vector<std::string>>("attach_acm_links", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("object_ids", std::vector<std::string>{});
  }

  bool loadObjectSpec(const std::string & id, SceneObjectSpec & out)
  {
    out.id = id;
    const std::string prefix = "objects." + id + ".";

    if (!declare_parameter<std::string>(prefix + "type", "").empty()) {}
    out.type = get_parameter(prefix + "type").as_string();

    declare_parameter<std::vector<double>>(prefix + "size", std::vector<double>{});
    out.size = get_parameter(prefix + "size").as_double_array();

    declare_parameter<double>(prefix + "radius", 0.0);
    out.radius = get_parameter(prefix + "radius").as_double();

    declare_parameter<double>(prefix + "height", 0.0);
    out.height = get_parameter(prefix + "height").as_double();

    declare_parameter<std::string>(prefix + "mesh_path", "");
    out.mesh_path = get_parameter(prefix + "mesh_path").as_string();

    declare_parameter<std::vector<double>>(prefix + "scale", std::vector<double>{1.0, 1.0, 1.0});
    out.scale = get_parameter(prefix + "scale").as_double_array();

    declare_parameter<std::vector<double>>(prefix + "position", std::vector<double>{0.0, 0.0, 0.0});
    out.position = get_parameter(prefix + "position").as_double_array();

    declare_parameter<std::vector<double>>(prefix + "orientation", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    out.orientation = get_parameter(prefix + "orientation").as_double_array();

    declare_parameter<bool>(prefix + "dynamic", false);
    out.dynamic = get_parameter(prefix + "dynamic").as_bool();

    return !out.type.empty();
  }

  geometry_msgs::msg::Pose makePose(const std::vector<double> & p, const std::vector<double> & q)
  {
    geometry_msgs::msg::Pose pose;
    if (p.size() == 3) {
      pose.position.x = p[0]; pose.position.y = p[1]; pose.position.z = p[2];
    }
    if (q.size() == 4) {
      pose.orientation.x = q[0]; pose.orientation.y = q[1];
      pose.orientation.z = q[2]; pose.orientation.w = q[3];
    } else {
      pose.orientation.w = 1.0;
    }
    return pose;
  }

  bool buildCollisionObject(const SceneObjectSpec & spec, moveit_msgs::msg::CollisionObject & out)
  {
    out.header.frame_id = frame_id_;
    out.id = spec.id;
    out.operation = moveit_msgs::msg::CollisionObject::ADD;
    const auto pose = makePose(spec.position, spec.orientation);

    if (spec.type == "box") {
      if (spec.size.size() != 3) {
        RCLCPP_ERROR(get_logger(), "object '%s': box.size must have 3 values.", spec.id.c_str());
        return false;
      }
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = shape_msgs::msg::SolidPrimitive::BOX;
      prim.dimensions = {spec.size[0], spec.size[1], spec.size[2]};
      out.primitives.push_back(prim);
      out.primitive_poses.push_back(pose);
      return true;
    }
    if (spec.type == "cylinder") {
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      prim.dimensions = {spec.height, spec.radius};
      out.primitives.push_back(prim);
      out.primitive_poses.push_back(pose);
      return true;
    }
    if (spec.type == "sphere") {
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      prim.dimensions = {spec.radius};
      out.primitives.push_back(prim);
      out.primitive_poses.push_back(pose);
      return true;
    }
    if (spec.type == "mesh") {
      if (spec.mesh_path.empty()) {
        RCLCPP_WARN(get_logger(), "object '%s': mesh_path empty, skipping.", spec.id.c_str());
        return false;
      }
      const Eigen::Vector3d scale(
        spec.scale.size() >= 3 ? spec.scale[0] : 1.0,
        spec.scale.size() >= 3 ? spec.scale[1] : 1.0,
        spec.scale.size() >= 3 ? spec.scale[2] : 1.0);
      shapes::Mesh * mesh = shapes::createMeshFromResource(spec.mesh_path, scale);
      if (!mesh) {
        RCLCPP_ERROR(get_logger(), "object '%s': failed to load mesh '%s'.",
                     spec.id.c_str(), spec.mesh_path.c_str());
        return false;
      }
      shapes::ShapeMsg mesh_msg;
      shapes::constructMsgFromShape(mesh, mesh_msg);
      auto concrete = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
      delete mesh;
      out.meshes.push_back(concrete);
      out.mesh_poses.push_back(pose);
      return true;
    }

    RCLCPP_ERROR(get_logger(), "object '%s': unknown type '%s'.",
                 spec.id.c_str(), spec.type.c_str());
    return false;
  }

  void publishDiffOnTopic(const std::vector<moveit_msgs::msg::CollisionObject> & objs)
  {
    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;
    ps.robot_state.is_diff = true;
    ps.world.collision_objects = objs;
    scene_pub_->publish(ps);
  }

  bool applyDiffMany(const std::vector<moveit_msgs::msg::CollisionObject> & objs,
                     const std::string & label)
  {
    if (!apply_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "[%s] /apply_planning_scene unavailable.", label.c_str());
      return false;
    }
    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene.is_diff = true;
    req->scene.robot_state.is_diff = true;
    req->scene.world.collision_objects = objs;
    auto fut = apply_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[%s] timed out.", label.c_str());
      return false;
    }
    auto res = fut.get();
    if (!res->success) {
      RCLCPP_ERROR(get_logger(), "[%s] returned FAILURE.", label.c_str());
      return false;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(scene_update_wait_ms_));
    RCLCPP_INFO(get_logger(), "[%s] OK (%zu objects).", label.c_str(), objs.size());
    return true;
  }

  bool applyDiff(const moveit_msgs::msg::CollisionObject & obj, const std::string & label)
  {
    std::vector<moveit_msgs::msg::CollisionObject> v{obj};
    const bool ok = applyDiffMany(v, label);
    if (ok) publishDiffOnTopic(v);
    return ok;
  }

  void loadInitialScene()
  {
    if (initial_loaded_) return;
    initial_loaded_ = true;
    init_timer_->cancel();
    try {
      frame_id_ = get_parameter("frame_id").as_string();
      scene_update_wait_ms_ = get_parameter("scene_update_wait_ms").as_int();
      const auto ids = get_parameter("object_ids").as_string_array();
      RCLCPP_INFO(get_logger(),
        "scene_manager: loading %zu object(s) (frame_id=%s).",
        ids.size(), frame_id_.c_str());

      // Wipe existing
      if (!ids.empty()) {
        moveit::planning_interface::PlanningSceneInterface psi;
        psi.removeCollisionObjects(ids);
        rclcpp::sleep_for(std::chrono::milliseconds(scene_update_wait_ms_));
      }

      std::vector<moveit_msgs::msg::CollisionObject> objs;
      for (const auto & id : ids) {
        SceneObjectSpec spec;
        if (!loadObjectSpec(id, spec)) {
          RCLCPP_WARN(get_logger(), "Skipping malformed object '%s'.", id.c_str());
          continue;
        }
        specs_[id] = spec;
        moveit_msgs::msg::CollisionObject obj;
        if (!buildCollisionObject(spec, obj)) continue;
        objs.push_back(obj);
        if (spec.dynamic) {
          forced_[id] = obj;
        }
      }

      if (!objs.empty()) applyDiffMany(objs, "INITIAL scene");

      // ACM tweak
      const auto acm_links = get_parameter("attach_acm_links").as_string_array();
      if (!acm_links.empty()) {
        moveit_msgs::msg::PlanningScene ps;
        ps.is_diff = true;
        ps.allowed_collision_matrix.default_entry_names = acm_links;
        ps.allowed_collision_matrix.default_entry_values.assign(acm_links.size(), true);
        rclcpp::sleep_for(std::chrono::milliseconds(300));
        scene_pub_->publish(ps);
        rclcpp::sleep_for(std::chrono::milliseconds(300));
        RCLCPP_INFO(get_logger(),
          "ACM: %zu link(s) allowed to collide with everything.",
          acm_links.size());
      }

      RCLCPP_INFO(get_logger(),
        "scene_manager: ready. Services: /scene/remove_object, /scene/set_object_pose.");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Initial scene load failed: %s", e.what());
    }
  }

  void handleRemove(
    const std::shared_ptr<industrial_bt_framework::srv::RemoveSceneObject::Request> req,
    std::shared_ptr<industrial_bt_framework::srv::RemoveSceneObject::Response> res)
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    forced_.erase(req->object_id);
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id_;
    obj.id = req->object_id;
    obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    res->success = applyDiff(obj, "REMOVE " + req->object_id);
    res->message = res->success ? "removed" : "remove failed";
  }

  void handleSetPose(
    const std::shared_ptr<industrial_bt_framework::srv::SetScenePose::Request> req,
    std::shared_ptr<industrial_bt_framework::srv::SetScenePose::Response> res)
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    auto it = specs_.find(req->object_id);
    if (it == specs_.end()) {
      res->success = false;
      res->message = "unknown object_id";
      return;
    }
    // Overwrite the stored pose and re-emit an ADD diff with the same geometry.
    SceneObjectSpec spec = it->second;
    spec.position = {req->pose.position.x, req->pose.position.y, req->pose.position.z};
    spec.orientation = {req->pose.orientation.x, req->pose.orientation.y,
                        req->pose.orientation.z, req->pose.orientation.w};
    specs_[req->object_id] = spec;

    moveit_msgs::msg::CollisionObject obj;
    if (!buildCollisionObject(spec, obj)) {
      res->success = false;
      res->message = "failed to rebuild collision object";
      return;
    }
    res->success = applyDiff(obj, "TELEPORT " + req->object_id);
    if (res->success && spec.dynamic) {
      forced_[req->object_id] = obj;
    }
    res->message = res->success ? "teleported" : "set_pose failed";
  }

  void reassertDynamic()
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    if (forced_.empty()) return;
    std::vector<moveit_msgs::msg::CollisionObject> objs;
    objs.reserve(forced_.size());
    for (const auto & kv : forced_) objs.push_back(kv.second);
    publishDiffOnTopic(objs);
  }

  // members
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr force_timer_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  ApplySceneClient::SharedPtr apply_client_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
  rclcpp::Service<industrial_bt_framework::srv::RemoveSceneObject>::SharedPtr remove_srv_;
  rclcpp::Service<industrial_bt_framework::srv::SetScenePose>::SharedPtr set_pose_srv_;

  std::mutex scene_mutex_;
  bool initial_loaded_ = false;
  std::string frame_id_ = "base_link";
  int scene_update_wait_ms_ = 800;
  std::unordered_map<std::string, SceneObjectSpec> specs_;
  std::unordered_map<std::string, moveit_msgs::msg::CollisionObject> forced_;
};

}  // namespace industrial_bt_framework

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<industrial_bt_framework::SceneManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
