#include "industrial_bt_framework/bt_nodes.hpp"
#include "industrial_bt_framework/perception_client.hpp"

#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

namespace industrial_bt_framework {

// ─── Helpers ─────────────────────────────────────────────────────────────────

std::vector<double> degVecToRad(const std::vector<double> & deg)
{
  std::vector<double> rad;
  rad.reserve(deg.size());
  for (double d : deg) rad.push_back(d * M_PI / 180.0);
  return rad;
}

std::array<double, 3> parseXyz3(const std::string & s)
{
  std::vector<double> v = parseVec(s);
  if (v.size() != 3) {
    throw std::runtime_error("Expected 3 values, got " + std::to_string(v.size()));
  }
  return {v[0], v[1], v[2]};
}

std::vector<double> parseVec(const std::string & s)
{
  std::vector<double> out;
  std::istringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ';')) {
    if (!tok.empty()) out.push_back(std::stod(tok));
  }
  return out;
}

static std::shared_ptr<RobotClient> getClient(const BT::NodeConfig & cfg)
{
  return cfg.blackboard->get<std::shared_ptr<RobotClient>>(BB_ROBOT_CLIENT);
}

static MotionProfile getProfile(const BT::NodeConfig & cfg, const std::string & name)
{
  const auto & registry = cfg.blackboard->get<MotionProfileRegistry>(BB_MOTION_PROFILES);
  auto it = registry.find(name);
  if (it != registry.end()) return it->second;
  auto it2 = registry.find("default");
  if (it2 != registry.end()) return it2->second;
  return MotionProfile{};
}

// ═════════════════════════════════════════════════════════════════════════════
// MoveToNamedTarget
// ═════════════════════════════════════════════════════════════════════════════

MoveToNamedTarget::MoveToNamedTarget(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList MoveToNamedTarget::providedPorts()
{
  return {
    BT::InputPort<std::string>("target"),
    BT::InputPort<std::string>("profile", "transit")
  };
}

BT::NodeStatus MoveToNamedTarget::tick()
{
  auto t = getInput<std::string>("target");
  auto p = getInput<std::string>("profile");
  if (!t || !p) return BT::NodeStatus::FAILURE;
  auto client = getClient(config());
  auto profile = getProfile(config(), p.value());
  const bool ok = client->moveToNamedTarget(t.value(), profile,
    "MoveToNamedTarget(" + t.value() + ")");
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═════════════════════════════════════════════════════════════════════════════
// MoveToJointTarget
// ═════════════════════════════════════════════════════════════════════════════

MoveToJointTarget::MoveToJointTarget(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList MoveToJointTarget::providedPorts()
{
  return {
    BT::InputPort<std::string>("joints_deg"),
    BT::InputPort<std::string>("profile", "transit")
  };
}

BT::NodeStatus MoveToJointTarget::tick()
{
  auto j = getInput<std::string>("joints_deg");
  auto p = getInput<std::string>("profile");
  if (!j || !p) return BT::NodeStatus::FAILURE;
  const auto joints_deg = parseVec(j.value());
  if (joints_deg.empty()) return BT::NodeStatus::FAILURE;
  const auto joints_rad = degVecToRad(joints_deg);
  auto client = getClient(config());
  auto profile = getProfile(config(), p.value());
  const bool ok = client->moveToJointTarget(joints_rad, profile, "MoveToJointTarget");
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═════════════════════════════════════════════════════════════════════════════
// ExecuteCartesianSegment
// ═════════════════════════════════════════════════════════════════════════════

ExecuteCartesianSegment::ExecuteCartesianSegment(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList ExecuteCartesianSegment::providedPorts()
{
  return {
    BT::InputPort<std::string>("pose_key"),
    BT::InputPort<std::string>("profile", "transit")
  };
}

BT::NodeStatus ExecuteCartesianSegment::tick()
{
  auto k = getInput<std::string>("pose_key");
  auto p = getInput<std::string>("profile");
  if (!k || !p) return BT::NodeStatus::FAILURE;
  geometry_msgs::msg::Pose pose;
  try {
    pose = config().blackboard->get<geometry_msgs::msg::Pose>(k.value());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("ExecuteCartesianSegment"),
      "Blackboard key '%s' not found: %s", k.value().c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }
  auto client = getClient(config());
  auto profile = getProfile(config(), p.value());
  const bool ok = client->executeLinear(pose, profile, "ExecuteCartesianSegment(" + k.value() + ")");
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═════════════════════════════════════════════════════════════════════════════
// MoveToPose
// ═════════════════════════════════════════════════════════════════════════════

MoveToPose::MoveToPose(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList MoveToPose::providedPorts()
{
  return {
    BT::InputPort<std::string>("pose_key"),
    BT::InputPort<std::string>("profile", "default")
  };
}

BT::NodeStatus MoveToPose::tick()
{
  auto k = getInput<std::string>("pose_key");
  auto p = getInput<std::string>("profile");
  if (!k || !p) return BT::NodeStatus::FAILURE;
  geometry_msgs::msg::Pose pose;
  try {
    pose = config().blackboard->get<geometry_msgs::msg::Pose>(k.value());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveToPose"),
      "Blackboard key '%s' not found: %s", k.value().c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }
  auto client  = getClient(config());
  auto profile = getProfile(config(), p.value());
  const bool ok = client->moveToPose(pose, profile, "MoveToPose(" + k.value() + ")");
  return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═════════════════════════════════════════════════════════════════════════════
// StoreCurrentPose
// ═════════════════════════════════════════════════════════════════════════════

StoreCurrentPose::StoreCurrentPose(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList StoreCurrentPose::providedPorts()
{
  return { BT::InputPort<std::string>("pose_key") };
}

BT::NodeStatus StoreCurrentPose::tick()
{
  auto k = getInput<std::string>("pose_key");
  if (!k) return BT::NodeStatus::FAILURE;
  auto client = getClient(config());
  auto pose = client->getCurrentPose();
  config().blackboard->set<geometry_msgs::msg::Pose>(k.value(), pose);
  return BT::NodeStatus::SUCCESS;
}

// ═════════════════════════════════════════════════════════════════════════════
// ComputeTCPTarget
// ═════════════════════════════════════════════════════════════════════════════

ComputeTCPTarget::ComputeTCPTarget(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList ComputeTCPTarget::providedPorts()
{
  return {
    BT::InputPort<std::string>("target_xyz"),
    BT::InputPort<std::string>("tcp_offset_xyz", "0.0;0.0;0.0"),
    BT::InputPort<std::string>("tcp_offset_rpy_deg", "0.0;0.0;0.0"),
    BT::InputPort<std::string>("use_current_orientation", "true"),
    BT::InputPort<std::string>("fixed_rpy_deg", "0.0;0.0;0.0"),
    BT::InputPort<std::string>("out_key")
  };
}

static Eigen::Quaterniond quatFromRpyDeg(const std::array<double, 3> & rpy_deg)
{
  const double r = rpy_deg[0] * M_PI / 180.0;
  const double p = rpy_deg[1] * M_PI / 180.0;
  const double y = rpy_deg[2] * M_PI / 180.0;
  Eigen::Quaterniond q =
    Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
  q.normalize();
  return q;
}

BT::NodeStatus ComputeTCPTarget::tick()
{
  auto txyz_p = getInput<std::string>("target_xyz");
  auto toff_p = getInput<std::string>("tcp_offset_xyz");
  auto trpy_p = getInput<std::string>("tcp_offset_rpy_deg");
  auto uco_p  = getInput<std::string>("use_current_orientation");
  auto frpy_p = getInput<std::string>("fixed_rpy_deg");
  auto out_p  = getInput<std::string>("out_key");

  auto log = rclcpp::get_logger("ComputeTCPTarget");
  if (!txyz_p) { RCLCPP_ERROR(log, "port 'target_xyz' unresolved: %s", txyz_p.error().c_str()); return BT::NodeStatus::FAILURE; }
  if (!toff_p) { RCLCPP_ERROR(log, "port 'tcp_offset_xyz' unresolved: %s", toff_p.error().c_str()); return BT::NodeStatus::FAILURE; }
  if (!trpy_p) { RCLCPP_ERROR(log, "port 'tcp_offset_rpy_deg' unresolved: %s", trpy_p.error().c_str()); return BT::NodeStatus::FAILURE; }
  if (!out_p)  { RCLCPP_ERROR(log, "port 'out_key' unresolved: %s", out_p.error().c_str()); return BT::NodeStatus::FAILURE; }

  try {
    const auto target_xyz = parseXyz3(txyz_p.value());
    const auto tcp_xyz    = parseXyz3(toff_p.value());
    const auto tcp_rpy    = parseXyz3(trpy_p.value());

    auto client = getClient(config());
    geometry_msgs::msg::Pose base_pose;
    base_pose.position.x = target_xyz[0];
    base_pose.position.y = target_xyz[1];
    base_pose.position.z = target_xyz[2];

    const bool use_current = (uco_p && uco_p.value() == "true");
    if (use_current) {
      base_pose.orientation = client->getCurrentPose().orientation;
    } else {
      const auto fixed = parseXyz3(frpy_p.value());
      auto q = quatFromRpyDeg(fixed);
      base_pose.orientation.x = q.x();
      base_pose.orientation.y = q.y();
      base_pose.orientation.z = q.z();
      base_pose.orientation.w = q.w();
    }

    auto tcp_pose = RobotClient::applyTCPOffset(base_pose, tcp_xyz, tcp_rpy);
    config().blackboard->set<geometry_msgs::msg::Pose>(out_p.value(), tcp_pose);
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeTCPTarget"), "Parse error: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// OffsetPoseInToolFrame
// ═════════════════════════════════════════════════════════════════════════════

OffsetPoseInToolFrame::OffsetPoseInToolFrame(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList OffsetPoseInToolFrame::providedPorts()
{
  return {
    BT::InputPort<std::string>("in_key"),
    BT::InputPort<std::string>("dx", "0.0"),
    BT::InputPort<std::string>("dy", "0.0"),
    BT::InputPort<std::string>("dz", "0.0"),
    BT::InputPort<std::string>("out_key")
  };
}

BT::NodeStatus OffsetPoseInToolFrame::tick()
{
  auto in_p  = getInput<std::string>("in_key");
  auto dx_p  = getInput<std::string>("dx");
  auto dy_p  = getInput<std::string>("dy");
  auto dz_p  = getInput<std::string>("dz");
  auto out_p = getInput<std::string>("out_key");
  if (!in_p || !out_p) return BT::NodeStatus::FAILURE;

  try {
    auto src = config().blackboard->get<geometry_msgs::msg::Pose>(in_p.value());
    const double dx = std::stod(dx_p.value());
    const double dy = std::stod(dy_p.value());
    const double dz = std::stod(dz_p.value());
    auto out = RobotClient::offsetInToolFrame(src, dx, dy, dz);
    config().blackboard->set<geometry_msgs::msg::Pose>(out_p.value(), out);
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("OffsetPoseInToolFrame"), "Error: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// OffsetPoseInBaseFrame
// ═════════════════════════════════════════════════════════════════════════════

OffsetPoseInBaseFrame::OffsetPoseInBaseFrame(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList OffsetPoseInBaseFrame::providedPorts()
{
  return {
    BT::InputPort<std::string>("in_key"),
    BT::InputPort<std::string>("dx", "0.0"),
    BT::InputPort<std::string>("dy", "0.0"),
    BT::InputPort<std::string>("dz", "0.0"),
    BT::InputPort<std::string>("out_key")
  };
}

BT::NodeStatus OffsetPoseInBaseFrame::tick()
{
  auto in_p  = getInput<std::string>("in_key");
  auto dx_p  = getInput<std::string>("dx");
  auto dy_p  = getInput<std::string>("dy");
  auto dz_p  = getInput<std::string>("dz");
  auto out_p = getInput<std::string>("out_key");
  if (!in_p || !out_p) return BT::NodeStatus::FAILURE;

  try {
    auto src = config().blackboard->get<geometry_msgs::msg::Pose>(in_p.value());
    src.position.x += std::stod(dx_p.value());
    src.position.y += std::stod(dy_p.value());
    src.position.z += std::stod(dz_p.value());
    config().blackboard->set<geometry_msgs::msg::Pose>(out_p.value(), src);
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("OffsetPoseInBaseFrame"), "Error: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// ActivateTool / ReleaseTool
// ═════════════════════════════════════════════════════════════════════════════

ActivateTool::ActivateTool(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList ActivateTool::providedPorts()
{
  return { BT::InputPort<std::string>("tool") };
}

BT::NodeStatus ActivateTool::tick()
{
  auto t = getInput<std::string>("tool");
  if (!t) return BT::NodeStatus::FAILURE;
  auto client = getClient(config());
  return client->tools()->activate(t.value())
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

ReleaseTool::ReleaseTool(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList ReleaseTool::providedPorts()
{
  return { BT::InputPort<std::string>("tool") };
}

BT::NodeStatus ReleaseTool::tick()
{
  auto t = getInput<std::string>("tool");
  if (!t) return BT::NodeStatus::FAILURE;
  auto client = getClient(config());
  return client->tools()->release(t.value())
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═════════════════════════════════════════════════════════════════════════════
// RemoveSceneObject / TeleportSceneObject
// ═════════════════════════════════════════════════════════════════════════════

RemoveSceneObject::RemoveSceneObject(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList RemoveSceneObject::providedPorts()
{
  return { BT::InputPort<std::string>("object_id") };
}

BT::NodeStatus RemoveSceneObject::tick()
{
  auto id = getInput<std::string>("object_id");
  if (!id) return BT::NodeStatus::FAILURE;
  auto client = getClient(config());
  return client->scene()->removeObject(id.value())
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

TeleportSceneObject::TeleportSceneObject(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList TeleportSceneObject::providedPorts()
{
  return {
    BT::InputPort<std::string>("object_id"),
    BT::InputPort<std::string>("pose_key"),
    BT::InputPort<std::string>("force_upright", "false"),
    BT::InputPort<std::string>("override_z", "")
  };
}

BT::NodeStatus TeleportSceneObject::tick()
{
  auto id = getInput<std::string>("object_id");
  auto k  = getInput<std::string>("pose_key");
  if (!id || !k) return BT::NodeStatus::FAILURE;
  geometry_msgs::msg::Pose pose;
  try {
    pose = config().blackboard->get<geometry_msgs::msg::Pose>(k.value());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("TeleportSceneObject"),
      "Blackboard key '%s' missing: %s", k.value().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }
  // Optional: force identity orientation (object placed upright)
  auto upright_p = getInput<std::string>("force_upright");
  if (upright_p && upright_p.value() == "true") {
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
  }
  // Optional: override position.z (e.g., place object on table surface)
  auto oz_p = getInput<std::string>("override_z");
  if (oz_p && !oz_p.value().empty()) {
    try { pose.position.z = std::stod(oz_p.value()); }
    catch (...) {}
  }
  auto client = getClient(config());
  return client->scene()->setObjectPose(id.value(), pose)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═════════════════════════════════════════════════════════════════════════════
// WaitMs / LogMessage
// ═════════════════════════════════════════════════════════════════════════════

WaitMs::WaitMs(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList WaitMs::providedPorts()
{
  return { BT::InputPort<int>("duration_ms", 100, "duration in ms") };
}

BT::NodeStatus WaitMs::tick()
{
  int ms = 100;
  auto d = getInput<int>("duration_ms");
  if (d) ms = d.value();
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  return BT::NodeStatus::SUCCESS;
}

LogMessage::LogMessage(const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList LogMessage::providedPorts()
{
  return { BT::InputPort<std::string>("message") };
}

BT::NodeStatus LogMessage::tick()
{
  auto m = getInput<std::string>("message");
  if (m) {
    RCLCPP_INFO(rclcpp::get_logger("BT"), "%s", m.value().c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

// ═════════════════════════════════════════════════════════════════════════════
// GetDetectedObjectPose
// ═════════════════════════════════════════════════════════════════════════════

namespace {

// Cache PerceptionClient instances keyed by topic so multiple ticks share one
// subscription. Keyed by topic string; protected by a mutex for multi-threaded
// executors.
std::shared_ptr<PerceptionClient> perceptionClientFor(
  rclcpp::Node::SharedPtr node, const std::string & topic)
{
  static std::mutex mtx;
  static std::unordered_map<std::string, std::shared_ptr<PerceptionClient>> cache;
  std::lock_guard<std::mutex> lk(mtx);
  auto it = cache.find(topic);
  if (it != cache.end()) return it->second;
  auto client = std::make_shared<PerceptionClient>(node, topic);
  cache.emplace(topic, client);
  return client;
}

}  // namespace

GetDetectedObjectPose::GetDetectedObjectPose(
  const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList GetDetectedObjectPose::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic", "/detected_objects/poses_6d"),
    BT::InputPort<std::string>("object_index", "0"),
    BT::InputPort<std::string>("target_frame", "base_link"),
    BT::InputPort<std::string>("wait_timeout_sec", "5.0"),
    BT::InputPort<std::string>("tf_timeout_sec", "2.0"),
    BT::InputPort<std::string>("out_key")
  };
}

BT::NodeStatus GetDetectedObjectPose::tick()
{
  auto topic_p   = getInput<std::string>("topic");
  auto index_p   = getInput<std::string>("object_index");
  auto frame_p   = getInput<std::string>("target_frame");
  auto wait_p    = getInput<std::string>("wait_timeout_sec");
  auto tf_p      = getInput<std::string>("tf_timeout_sec");
  auto out_p     = getInput<std::string>("out_key");
  if (!topic_p || !frame_p || !out_p) {
    RCLCPP_ERROR(rclcpp::get_logger("GetDetectedObjectPose"),
      "Missing required port (topic, target_frame, out_key).");
    return BT::NodeStatus::FAILURE;
  }

  std::size_t idx = 0;
  double wait_sec = 5.0, tf_sec = 2.0;
  try {
    if (index_p && !index_p.value().empty()) idx = static_cast<std::size_t>(std::stoul(index_p.value()));
    if (wait_p  && !wait_p.value().empty())  wait_sec = std::stod(wait_p.value());
    if (tf_p    && !tf_p.value().empty())    tf_sec   = std::stod(tf_p.value());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("GetDetectedObjectPose"),
      "Invalid numeric port: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  rclcpp::Node::SharedPtr node;
  try {
    node = config().blackboard->get<rclcpp::Node::SharedPtr>(BB_ROS_NODE);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("GetDetectedObjectPose"),
      "Blackboard missing '%s' (ros_node). Runner must populate it.", BB_ROS_NODE);
    return BT::NodeStatus::FAILURE;
  }

  auto client = perceptionClientFor(node, topic_p.value());

  auto stamped = client->waitForPose(idx, std::chrono::duration<double>(wait_sec));
  if (!stamped) return BT::NodeStatus::FAILURE;

  auto out_pose = client->transformToFrame(
    stamped.value(), frame_p.value(), std::chrono::duration<double>(tf_sec));
  if (!out_pose) return BT::NodeStatus::FAILURE;

  RCLCPP_INFO(node->get_logger(),
    "GetDetectedObjectPose[%s idx=%zu] -> frame=%s xyz=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
    topic_p.value().c_str(), idx, frame_p.value().c_str(),
    out_pose->position.x, out_pose->position.y, out_pose->position.z,
    out_pose->orientation.x, out_pose->orientation.y,
    out_pose->orientation.z, out_pose->orientation.w);

  config().blackboard->set<geometry_msgs::msg::Pose>(out_p.value(), out_pose.value());
  return BT::NodeStatus::SUCCESS;
}

// ═════════════════════════════════════════════════════════════════════════════
// ComputeSuctionPickPose
// ═════════════════════════════════════════════════════════════════════════════

ComputeSuctionPickPose::ComputeSuctionPickPose(
  const std::string & n, const BT::NodeConfig & c)
: BT::SyncActionNode(n, c) {}

BT::PortsList ComputeSuctionPickPose::providedPorts()
{
  return {
    BT::InputPort<std::string>("detected_pose_key"),
    BT::InputPort<std::string>("tool_offset_z", "0.190"),
    BT::InputPort<std::string>("flip_z", "true"),
    BT::InputPort<std::string>("reference_pose_key", "approach_pose"),
    BT::InputPort<std::string>("out_key")
  };
}

BT::NodeStatus ComputeSuctionPickPose::tick()
{
  auto det_p  = getInput<std::string>("detected_pose_key");
  auto off_p  = getInput<std::string>("tool_offset_z");
  auto flip_p = getInput<std::string>("flip_z");
  auto ref_p  = getInput<std::string>("reference_pose_key");
  auto out_p  = getInput<std::string>("out_key");
  if (!det_p || !out_p) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeSuctionPickPose"),
      "Missing detected_pose_key or out_key.");
    return BT::NodeStatus::FAILURE;
  }

  double tool_offset_z = 0.190;
  bool flip_z = true;
  try {
    if (off_p && !off_p.value().empty()) tool_offset_z = std::stod(off_p.value());
    if (flip_p && !flip_p.value().empty()) {
      std::string v = flip_p.value();
      flip_z = (v == "true" || v == "1" || v == "True" || v == "TRUE");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeSuctionPickPose"),
      "Invalid numeric port: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::Pose det;
  try {
    det = config().blackboard->get<geometry_msgs::msg::Pose>(det_p.value());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeSuctionPickPose"),
      "Blackboard entry '%s' is not a Pose: %s", det_p.value().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  Eigen::Quaterniond q_det(det.orientation.w, det.orientation.x,
                           det.orientation.y, det.orientation.z);
  if (q_det.norm() < 1e-9) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeSuctionPickPose"),
      "Detected orientation quaternion has zero norm.");
    return BT::NodeStatus::FAILURE;
  }
  q_det.normalize();
  Eigen::Matrix3d R_det = q_det.toRotationMatrix();

  // Tool Z = surface normal pointing INTO the object (flip if the detection
  // normal points toward the camera, which is the default from the 6D pose
  // estimator).
  Eigen::Vector3d tool_z = flip_z ? Eigen::Vector3d(-R_det.col(2))
                                  : Eigen::Vector3d(R_det.col(2));
  if (tool_z.norm() < 1e-9) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeSuctionPickPose"),
      "tool_z has zero norm.");
    return BT::NodeStatus::FAILURE;
  }
  tool_z.normalize();

  // Build tool X by projecting a reference X axis onto the plane perpendicular
  // to tool_z. We try, in order:
  //   1. RobotClient::getCurrentPose() — the live MoveIt end-effector pose
  //      (bullet-proof, no BB indirection).
  //   2. A blackboard entry (default key `approach_pose` — see reference_pose_key).
  //   3. World X as a last resort.
  //
  // This keeps the wrist close to its current configuration and avoids IK
  // branches that violate joint limits (otherwise the detection's X axis —
  // which comes from the camera's arbitrary image-X projection — drives the
  // wrist into a singular/out-of-range configuration).
  Eigen::Vector3d ref_x(1.0, 0.0, 0.0);
  std::string ref_source = "world_x";

  auto use_quat_x = [&](const geometry_msgs::msg::Pose & p,
                        const std::string & tag) -> bool {
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x,
                         p.orientation.y, p.orientation.z);
    if (q.norm() < 1e-9) return false;
    q.normalize();
    ref_x = q.toRotationMatrix().col(0);
    ref_source = tag;
    return true;
  };

  // 1) Live current pose from MoveGroupInterface.
  bool got_ref = false;
  try {
    auto client = getClient(config());
    auto cur = client->getCurrentPose();
    if (use_quat_x(cur, "current_pose")) {
      got_ref = true;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("ComputeSuctionPickPose"),
        "current_pose orientation has zero norm; trying BB fallback.");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(rclcpp::get_logger("ComputeSuctionPickPose"),
      "getCurrentPose() threw: %s; trying BB fallback.", e.what());
  }

  // 2) Blackboard entry (default: approach_pose).
  if (!got_ref && ref_p && !ref_p.value().empty()) {
    const std::string key = ref_p.value();
    try {
      auto p = config().blackboard->get<geometry_msgs::msg::Pose>(key);
      if (use_quat_x(p, "bb:" + key)) {
        got_ref = true;
      } else {
        RCLCPP_WARN(rclcpp::get_logger("ComputeSuctionPickPose"),
          "BB['%s'] orientation has zero norm; falling back to world X.", key.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("ComputeSuctionPickPose"),
        "BB['%s'] lookup failed: %s; falling back to world X.", key.c_str(), e.what());
    }
  }

  // 3) World X is already the initial ref_x / ref_source.

  Eigen::Vector3d tool_x = ref_x - ref_x.dot(tool_z) * tool_z;
  if (tool_x.norm() < 1e-3) {
    // Degenerate: ref_x is nearly parallel to tool_z. Try world Y.
    Eigen::Vector3d alt(0.0, 1.0, 0.0);
    tool_x = alt - alt.dot(tool_z) * tool_z;
    ref_source += "+worldY";
  }
  if (tool_x.norm() < 1e-6) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeSuctionPickPose"),
      "Cannot build orthogonal tool_x (tool_z parallel to both world X and Y).");
    return BT::NodeStatus::FAILURE;
  }
  tool_x.normalize();

  Eigen::Vector3d tool_y = tool_z.cross(tool_x);
  tool_y.normalize();

  Eigen::Matrix3d R_tool;
  R_tool.col(0) = tool_x;
  R_tool.col(1) = tool_y;
  R_tool.col(2) = tool_z;

  Eigen::Vector3d wrist3(
    det.position.x - tool_offset_z * tool_z.x(),
    det.position.y - tool_offset_z * tool_z.y(),
    det.position.z - tool_offset_z * tool_z.z());

  Eigen::Quaterniond q_out(R_tool);
  q_out.normalize();

  geometry_msgs::msg::Pose out;
  out.position.x = wrist3.x();
  out.position.y = wrist3.y();
  out.position.z = wrist3.z();
  out.orientation.x = q_out.x();
  out.orientation.y = q_out.y();
  out.orientation.z = q_out.z();
  out.orientation.w = q_out.w();

  RCLCPP_INFO(rclcpp::get_logger("ComputeSuctionPickPose"),
    "wrist3 target xyz=(%.3f, %.3f, %.3f) tool_z=(%.3f, %.3f, %.3f) tool_x=(%.3f, %.3f, %.3f) ref=%s flip_z=%d offset=%.3f",
    out.position.x, out.position.y, out.position.z,
    tool_z.x(), tool_z.y(), tool_z.z(),
    tool_x.x(), tool_x.y(), tool_x.z(),
    ref_source.c_str(), (int)flip_z, tool_offset_z);

  config().blackboard->set<geometry_msgs::msg::Pose>(out_p.value(), out);
  return BT::NodeStatus::SUCCESS;
}

// ═════════════════════════════════════════════════════════════════════════════
// Registration
// ═════════════════════════════════════════════════════════════════════════════

void registerAllNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<MoveToNamedTarget>("MoveToNamedTarget");
  factory.registerNodeType<MoveToJointTarget>("MoveToJointTarget");
  factory.registerNodeType<ExecuteCartesianSegment>("ExecuteCartesianSegment");
  factory.registerNodeType<MoveToPose>("MoveToPose");
  factory.registerNodeType<StoreCurrentPose>("StoreCurrentPose");
  factory.registerNodeType<ComputeTCPTarget>("ComputeTCPTarget");
  factory.registerNodeType<OffsetPoseInToolFrame>("OffsetPoseInToolFrame");
  factory.registerNodeType<OffsetPoseInBaseFrame>("OffsetPoseInBaseFrame");
  factory.registerNodeType<ActivateTool>("ActivateTool");
  factory.registerNodeType<ReleaseTool>("ReleaseTool");
  factory.registerNodeType<RemoveSceneObject>("RemoveSceneObject");
  factory.registerNodeType<TeleportSceneObject>("TeleportSceneObject");
  factory.registerNodeType<WaitMs>("WaitMs");
  factory.registerNodeType<LogMessage>("LogMessage");
  factory.registerNodeType<GetDetectedObjectPose>("GetDetectedObjectPose");
  factory.registerNodeType<ComputeSuctionPickPose>("ComputeSuctionPickPose");
}

}  // namespace industrial_bt_framework
