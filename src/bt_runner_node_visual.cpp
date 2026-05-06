// bt_runner_node_visual — same as bt_runner_node plus Groot2Publisher
// (when BT.CPP is built with ZMQ support) and a FileLogger2 trace file.

#include "industrial_bt_framework/bt_nodes.hpp"
#include "industrial_bt_framework/cartesian_backend.hpp"
#include "industrial_bt_framework/motion_profile.hpp"
#include "industrial_bt_framework/robot_client.hpp"
#include "industrial_bt_framework/scene_client.hpp"
#include "industrial_bt_framework/tool_registry.hpp"

#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#ifdef BTCPP_GROOT2_SUPPORT
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#endif

namespace industrial_bt_framework {

static MotionProfile profileFromNode(const YAML::Node & node, const MotionProfile & base)
{
  MotionProfile p = base;
#define LD(field) if (node[#field]) p.field = node[#field].as<double>();
#define LI(field) if (node[#field]) p.field = node[#field].as<int>();
  LD(planning_time_sec) LI(num_planning_attempts) LI(max_free_space_retries)
  LD(velocity_scaling) LD(acceleration_scaling)
  LD(goal_position_tolerance) LD(goal_orientation_tolerance) LD(joint_target_tolerance)
  LD(cartesian_primary_step_m) LD(cartesian_fallback_step_m)
  LD(cartesian_eef_step) LD(cartesian_jump_threshold)
  LD(cartesian_min_fraction) LD(cartesian_final_position_tol)
  LD(movel_speed_percent)
  LI(scene_update_wait_ms) LI(retry_sleep_ms) LD(world_spawn_safety_z_m)
#undef LD
#undef LI
  return p;
}

static MotionProfileRegistry loadMotionProfiles(const std::string & path)
{
  MotionProfileRegistry reg;
  const MotionProfile defaults{};
  if (path.empty()) { reg["default"] = defaults; return reg; }
  YAML::Node root = YAML::LoadFile(path);
  if (!root["motion_profiles"]) { reg["default"] = defaults; return reg; }
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

static void putBlackboardFromParam(
  BT::Blackboard::Ptr bb,
  const std::string & key,
  const rclcpp::Parameter & param)
{
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      bb->set<std::string>(key, param.as_bool() ? "true" : "false"); break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      bb->set<std::string>(key, std::to_string(param.as_int())); break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      bb->set<std::string>(key, std::to_string(param.as_double())); break;
    case rclcpp::ParameterType::PARAMETER_STRING:
      bb->set<std::string>(key, param.as_string()); break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      std::ostringstream os;
      const auto v = param.as_double_array();
      for (size_t i = 0; i < v.size(); ++i) { if (i) os << ';'; os << v[i]; }
      bb->set<std::string>(key, os.str());
      break;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      std::ostringstream os;
      const auto v = param.as_integer_array();
      for (size_t i = 0; i < v.size(); ++i) { if (i) os << ';'; os << v[i]; }
      bb->set<std::string>(key, os.str());
      break;
    }
    default: break;
  }
}

static void loadTaskParametersToBlackboard(
  const rclcpp::Node::SharedPtr & node, BT::Blackboard::Ptr bb)
{
  const std::string prefix = "task_parameters";
  auto names = node->list_parameters({prefix}, 10).names;
  RCLCPP_INFO(node->get_logger(), "loadTaskParametersToBlackboard: found %zu params", names.size());
  for (const auto & full_name : names) {
    rclcpp::Parameter p;
    if (node->get_parameter(full_name, p)) {
      const std::string short_key = full_name.substr(prefix.size() + 1);
      putBlackboardFromParam(bb, short_key, p);
      RCLCPP_INFO(node->get_logger(),
        "BB[%s] = %s", short_key.c_str(), bb->get<std::string>(short_key).c_str());
    }
  }
}

static void loadToolsIntoRegistry(
  const rclcpp::Node::SharedPtr & node, std::shared_ptr<ToolRegistry> reg)
{
  const std::string prefix = "tools";
  auto names = node->list_parameters({prefix}, 10).names;
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
    if (kv.second.first.empty() || kv.second.second.empty()) continue;
    reg->registerTool(kv.first, kv.second.first, kv.second.second);
  }
}

static YAML::Node loadBackendParamsFromFile(const std::string & path)
{
  if (path.empty()) return YAML::Node();
  try { return YAML::LoadFile(path); } catch (...) { return YAML::Node(); }
}

}  // namespace industrial_bt_framework

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "bt_runner_node_visual",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  auto declareIfMissing = [&](const std::string & name, auto def) {
    if (!node->has_parameter(name)) node->declare_parameter(name, def);
  };
  declareIfMissing("planning_group", std::string("manipulator"));
  declareIfMissing("cartesian_backend",
    std::string("industrial_bt_framework/MoveItCartesianBackend"));
  declareIfMissing("cartesian_backend_params_file", std::string(""));
  declareIfMissing("motion_profiles_file", std::string(""));
  declareIfMissing("bt_tree_file", std::string(""));
  declareIfMissing("bt_tree_id", std::string("MainTree"));
  declareIfMissing("scene_remove_service", std::string("/scene/remove_object"));
  declareIfMissing("scene_set_pose_service", std::string("/scene/set_object_pose"));
  declareIfMissing("zmq_server_port", 1666);
  declareIfMissing("zmq_publisher_port", 1667);
  declareIfMissing("enable_groot2_monitor", true);
  declareIfMissing("enable_file_logger", true);
  declareIfMissing("bt_log_file", std::string("/tmp/bt_trace.btlog"));
  declareIfMissing("export_node_models", true);
  declareIfMissing("node_models_export_file", std::string("/tmp/bt_node_models_export.xml"));

  const std::string planning_group  = node->get_parameter("planning_group").as_string();
  const std::string backend_plugin  = node->get_parameter("cartesian_backend").as_string();
  const std::string backend_params  = node->get_parameter("cartesian_backend_params_file").as_string();
  const std::string profiles_file   = node->get_parameter("motion_profiles_file").as_string();
  const std::string tree_file       = node->get_parameter("bt_tree_file").as_string();
  const std::string tree_id         = node->get_parameter("bt_tree_id").as_string();
  const std::string scene_rm_srv    = node->get_parameter("scene_remove_service").as_string();
  const std::string scene_sp_srv    = node->get_parameter("scene_set_pose_service").as_string();
  const int zmq_server_port   = node->get_parameter("zmq_server_port").as_int();
  const int zmq_pub_port      = node->get_parameter("zmq_publisher_port").as_int();
  const bool enable_monitor   = node->get_parameter("enable_groot2_monitor").as_bool();
  const bool enable_logger    = node->get_parameter("enable_file_logger").as_bool();
  const std::string log_file  = node->get_parameter("bt_log_file").as_string();
  const bool export_models    = node->get_parameter("export_node_models").as_bool();
  const std::string models_out = node->get_parameter("node_models_export_file").as_string();

  if (tree_file.empty()) {
    RCLCPP_FATAL(logger, "Parameter 'bt_tree_file' must be set.");
    return 1;
  }

  industrial_bt_framework::MotionProfileRegistry profiles;
  try { profiles = industrial_bt_framework::loadMotionProfiles(profiles_file); }
  catch (const std::exception & e) { RCLCPP_FATAL(logger, "profiles: %s", e.what()); return 1; }

  std::shared_ptr<industrial_bt_framework::CartesianBackend> backend;
  try {
    static pluginlib::ClassLoader<industrial_bt_framework::CartesianBackend> loader(
      "industrial_bt_framework", "industrial_bt_framework::CartesianBackend");
    backend = loader.createSharedInstance(backend_plugin);
    YAML::Node params = industrial_bt_framework::loadBackendParamsFromFile(backend_params);
    backend->initialize(node, nullptr, planning_group, params);
  } catch (const std::exception & e) { RCLCPP_FATAL(logger, "backend: %s", e.what()); return 1; }

  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto tools = std::make_shared<industrial_bt_framework::ToolRegistry>(node, cb_group);
  industrial_bt_framework::loadToolsIntoRegistry(node, tools);
  auto scene = std::make_shared<industrial_bt_framework::SceneClient>(
    node, cb_group, scene_rm_srv, scene_sp_srv);

  auto robot_client = std::make_shared<industrial_bt_framework::RobotClient>(
    node, planning_group, backend, tools, scene);

  BT::BehaviorTreeFactory factory;
  industrial_bt_framework::registerAllNodes(factory);

  if (export_models) {
    try {
      const std::string xml = BT::writeTreeNodesModelXML(factory);
      std::ofstream(models_out) << xml;
      RCLCPP_INFO(logger, "Node models exported to '%s'.", models_out.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger, "Failed to export node models: %s", e.what());
    }
  }

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
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger, "tree: %s", e.what()); return 1;
  }

#ifdef BTCPP_GROOT2_SUPPORT
  std::unique_ptr<BT::Groot2Publisher> groot2_pub;
  if (enable_monitor) {
    try {
      groot2_pub = std::make_unique<BT::Groot2Publisher>(tree, zmq_server_port);
      RCLCPP_INFO(logger, "Groot2Publisher ENABLED on tcp://localhost:%d", zmq_server_port);
      (void)zmq_pub_port;
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger, "Groot2Publisher failed: %s", e.what());
    }
  }
#else
  (void)enable_monitor; (void)zmq_server_port; (void)zmq_pub_port;
  RCLCPP_WARN(logger, "BT.CPP built without ZMQ — Groot2 monitor disabled.");
#endif

  std::unique_ptr<BT::FileLogger2> file_logger;
  if (enable_logger) {
    try {
      file_logger = std::make_unique<BT::FileLogger2>(tree, log_file);
      RCLCPP_INFO(logger, "FileLogger2 → '%s'", log_file.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger, "FileLogger2 failed: %s", e.what());
    }
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
  if (status == BT::NodeStatus::SUCCESS) RCLCPP_INFO(logger, "══ BT finished: SUCCESS ══");
  else RCLCPP_ERROR(logger, "══ BT finished: FAILURE ══");

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
