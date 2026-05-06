// Generic bt_runner_node for the industrial_bt_framework.
//
// Responsibilities:
//   1. Declare & load runner parameters (planning_group, backend plugin name,
//      tools, task_parameters...).
//   2. Load motion profiles from motion_profiles.yaml (yaml-cpp).
//   3. Instantiate the configured CartesianBackend plugin via pluginlib.
//   4. Build RobotClient, ToolRegistry, SceneClient.
//   5. Populate BT blackboard with robot_client, motion profiles, and every
//      scalar/vector key under `task_parameters` (so trees can reference
//      application data via {key}).
//   6. Register framework BT nodes, load the XML tree, tick to SUCCESS/FAILURE.

#include "industrial_bt_framework/bt_nodes.hpp"
#include "industrial_bt_framework/cartesian_backend.hpp"
#include "industrial_bt_framework/motion_profile.hpp"
#include "industrial_bt_framework/robot_client.hpp"
#include "industrial_bt_framework/scene_client.hpp"
#include "industrial_bt_framework/tool_registry.hpp"

#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace industrial_bt_framework {

// ─── Motion profiles loader ──────────────────────────────────────────────────

static MotionProfile profileFromNode(const YAML::Node & node, const MotionProfile & base)
{
  MotionProfile p = base;
#define LD(field) if (node[#field]) p.field = node[#field].as<double>();
#define LI(field) if (node[#field]) p.field = node[#field].as<int>();
  LD(planning_time_sec)
  LI(num_planning_attempts)
  LI(max_free_space_retries)
  LD(velocity_scaling)
  LD(acceleration_scaling)
  LD(goal_position_tolerance)
  LD(goal_orientation_tolerance)
  LD(joint_target_tolerance)
  LD(cartesian_primary_step_m)
  LD(cartesian_fallback_step_m)
  LD(cartesian_eef_step)
  LD(cartesian_jump_threshold)
  LD(cartesian_min_fraction)
  LD(cartesian_final_position_tol)
  LD(movel_speed_percent)
  LI(scene_update_wait_ms)
  LI(retry_sleep_ms)
  LD(world_spawn_safety_z_m)
#undef LD
#undef LI
  return p;
}

static MotionProfileRegistry loadMotionProfiles(const std::string & yaml_path)
{
  MotionProfileRegistry reg;
  const MotionProfile defaults{};
  if (yaml_path.empty()) {
    reg["default"] = defaults;
    return reg;
  }
  YAML::Node root = YAML::LoadFile(yaml_path);
  if (!root["motion_profiles"]) {
    reg["default"] = defaults;
    return reg;
  }
  MotionProfile base = defaults;
  if (root["motion_profiles"]["default"]) {
    base = profileFromNode(root["motion_profiles"]["default"], defaults);
  }
  reg["default"] = base;
  for (const auto & kv : root["motion_profiles"]) {
    const std::string name = kv.first.as<std::string>();
    if (name == "default") continue;
    reg[name] = profileFromNode(kv.second, base);
  }
  return reg;
}

// ─── Blackboard population from task_parameters.* parameters ─────────────────

static void putBlackboardFromParam(
  BT::Blackboard::Ptr bb,
  const std::string & key,
  const rclcpp::Parameter & param)
{
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      bb->set<std::string>(key, param.as_bool() ? "true" : "false");
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      bb->set<std::string>(key, std::to_string(param.as_int()));
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      bb->set<std::string>(key, std::to_string(param.as_double()));
      break;
    case rclcpp::ParameterType::PARAMETER_STRING:
      bb->set<std::string>(key, param.as_string());
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      std::ostringstream os;
      const auto v = param.as_double_array();
      for (size_t i = 0; i < v.size(); ++i) {
        if (i) os << ';';
        os << v[i];
      }
      bb->set<std::string>(key, os.str());
      break;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      std::ostringstream os;
      const auto v = param.as_integer_array();
      for (size_t i = 0; i < v.size(); ++i) {
        if (i) os << ';';
        os << v[i];
      }
      bb->set<std::string>(key, os.str());
      break;
    }
    default:
      break;
  }
}

// Scan declared parameters under `task_parameters.*` and push each into the
// blackboard under the short key (e.g. task_parameters.bottle_pick_xyz → bottle_pick_xyz).
static void loadTaskParametersToBlackboard(
  const rclcpp::Node::SharedPtr & node,
  BT::Blackboard::Ptr bb)
{
  const std::string prefix = "task_parameters";
  auto names = node->list_parameters({prefix}, 10).names;
  for (const auto & full_name : names) {
    const std::string short_key = full_name.substr(prefix.size() + 1);
    rclcpp::Parameter p;
    if (node->get_parameter(full_name, p)) {
      putBlackboardFromParam(bb, short_key, p);
      RCLCPP_INFO(node->get_logger(),
        "BB[%s] = %s", short_key.c_str(), bb->get<std::string>(short_key).c_str());
    }
  }
}

// Scan `tools.<name>.{activate_service,release_service}` parameters.
static void loadToolsIntoRegistry(
  const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<ToolRegistry> reg)
{
  const std::string prefix = "tools";
  auto names = node->list_parameters({prefix}, 10).names;
  // Group by tool name.
  std::unordered_map<std::string, std::pair<std::string, std::string>> tools;
  for (const auto & full_name : names) {
    const auto rest = full_name.substr(prefix.size() + 1);
    const auto dot = rest.find('.');
    if (dot == std::string::npos) continue;
    const std::string tool_name = rest.substr(0, dot);
    const std::string field = rest.substr(dot + 1);
    rclcpp::Parameter p;
    if (!node->get_parameter(full_name, p)) continue;
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) continue;
    if (field == "activate_service") tools[tool_name].first = p.as_string();
    else if (field == "release_service") tools[tool_name].second = p.as_string();
  }
  for (const auto & kv : tools) {
    if (kv.second.first.empty() || kv.second.second.empty()) {
      RCLCPP_WARN(node->get_logger(),
        "Tool '%s' missing activate_service or release_service; skipping.",
        kv.first.c_str());
      continue;
    }
    reg->registerTool(kv.first, kv.second.first, kv.second.second);
  }
}

// ─── Parse cartesian_backend_params from raw YAML file (simplest path) ───────
//
// The parameter system doesn't round-trip nested YAML well, so we allow an
// optional raw YAML file (`cartesian_backend_params_file`) whose root is the
// params dict. Falls back to an empty node if unset.
static YAML::Node loadBackendParamsFromFile(const std::string & path)
{
  if (path.empty()) return YAML::Node();
  try {
    return YAML::LoadFile(path);
  } catch (const std::exception & e) {
    return YAML::Node();
  }
}

}  // namespace industrial_bt_framework

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "bt_runner_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  auto declareIfMissing = [&](const std::string & name, auto def) {
    if (!node->has_parameter(name)) {
      node->declare_parameter(name, def);
    }
  };
  declareIfMissing("planning_group", std::string("manipulator"));
  declareIfMissing("home_named_target", std::string("home"));
  declareIfMissing("cartesian_backend",
    std::string("industrial_bt_framework/MoveItCartesianBackend"));
  declareIfMissing("cartesian_backend_params_file", std::string(""));
  declareIfMissing("motion_profiles_file", std::string(""));
  declareIfMissing("bt_tree_file", std::string(""));
  declareIfMissing("bt_tree_id", std::string("MainTree"));
  declareIfMissing("scene_remove_service", std::string("/scene/remove_object"));
  declareIfMissing("scene_set_pose_service", std::string("/scene/set_object_pose"));

  const std::string planning_group  = node->get_parameter("planning_group").as_string();
  const std::string backend_plugin  = node->get_parameter("cartesian_backend").as_string();
  const std::string backend_params  = node->get_parameter("cartesian_backend_params_file").as_string();
  const std::string profiles_file   = node->get_parameter("motion_profiles_file").as_string();
  const std::string tree_file       = node->get_parameter("bt_tree_file").as_string();
  const std::string tree_id         = node->get_parameter("bt_tree_id").as_string();
  const std::string scene_rm_srv    = node->get_parameter("scene_remove_service").as_string();
  const std::string scene_sp_srv    = node->get_parameter("scene_set_pose_service").as_string();

  if (tree_file.empty()) {
    RCLCPP_FATAL(logger, "Parameter 'bt_tree_file' must be set.");
    return 1;
  }

  // Motion profiles
  industrial_bt_framework::MotionProfileRegistry profiles;
  try {
    profiles = industrial_bt_framework::loadMotionProfiles(profiles_file);
    RCLCPP_INFO(logger, "Loaded %zu motion profile(s).", profiles.size());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger, "Failed to load motion profiles: %s", e.what());
    return 1;
  }

  // Cartesian backend plugin
  std::shared_ptr<industrial_bt_framework::CartesianBackend> backend;
  try {
    static pluginlib::ClassLoader<industrial_bt_framework::CartesianBackend> loader(
      "industrial_bt_framework", "industrial_bt_framework::CartesianBackend");
    backend = loader.createSharedInstance(backend_plugin);
    YAML::Node params = industrial_bt_framework::loadBackendParamsFromFile(backend_params);
    backend->initialize(node, nullptr, planning_group, params);
    RCLCPP_INFO(logger, "Cartesian backend '%s' loaded.", backend_plugin.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger, "Cartesian backend load failed: %s", e.what());
    return 1;
  }

  // Reentrant callback group for service clients (tools, scene)
  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto tools = std::make_shared<industrial_bt_framework::ToolRegistry>(node, cb_group);
  industrial_bt_framework::loadToolsIntoRegistry(node, tools);
  auto scene = std::make_shared<industrial_bt_framework::SceneClient>(
    node, cb_group, scene_rm_srv, scene_sp_srv);

  auto robot_client = std::make_shared<industrial_bt_framework::RobotClient>(
    node, planning_group, backend, tools, scene);
  RCLCPP_INFO(logger, "RobotClient ready (planning_group='%s').", planning_group.c_str());

  // BT
  BT::BehaviorTreeFactory factory;
  industrial_bt_framework::registerAllNodes(factory);
  auto bb = BT::Blackboard::create();
  bb->set<std::shared_ptr<industrial_bt_framework::RobotClient>>(
    industrial_bt_framework::BB_ROBOT_CLIENT, robot_client);
  bb->set<industrial_bt_framework::MotionProfileRegistry>(
    industrial_bt_framework::BB_MOTION_PROFILES, profiles);
  bb->set<rclcpp::Node::SharedPtr>(industrial_bt_framework::BB_ROS_NODE, node);
  industrial_bt_framework::loadTaskParametersToBlackboard(node, bb);

  BT::Tree tree;
  try {
    factory.registerBehaviorTreeFromFile(tree_file);
    tree = factory.createTree(tree_id, bb);
    RCLCPP_INFO(logger, "BT tree '%s' loaded from '%s'.", tree_id.c_str(), tree_file.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger, "Failed to build BT tree: %s", e.what());
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  // Allow DDS action channels to fully establish before the first MoveIt
  // request.  Without this, the MoveGroup action server may fail to deliver
  // goal responses ("Failed to send goal response (timeout)"), causing the
  // client to hang indefinitely.
  RCLCPP_INFO(logger, "Waiting 3 s for DDS action channels to settle...");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  RCLCPP_INFO(logger, "══ Starting BT: '%s' ══", tree_id.c_str());
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(logger, "══ BT finished: SUCCESS ══");
  } else {
    RCLCPP_ERROR(logger, "══ BT finished: FAILURE ══");
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
