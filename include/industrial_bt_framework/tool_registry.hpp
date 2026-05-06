#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace industrial_bt_framework {

// Maps a logical tool name (e.g. "gripper_primary") to a pair of
// std_srvs/Trigger service endpoints that activate / release the tool.
// Populated from the `tools` section of bt_runner.yaml.
class ToolRegistry
{
public:
  struct Entry
  {
    std::string activate_service;
    std::string release_service;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr activate_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr release_client;
  };

  ToolRegistry(
    rclcpp::Node::SharedPtr node,
    rclcpp::CallbackGroup::SharedPtr callback_group);

  // Register a tool from its YAML-loaded pair of service names.
  void registerTool(
    const std::string & name,
    const std::string & activate_service,
    const std::string & release_service);

  // Calls the activate (or release) Trigger service for a named tool.
  // Returns false if the tool is unknown or the call fails/times out.
  bool activate(const std::string & tool_name, int timeout_ms = 5000);
  bool release (const std::string & tool_name, int timeout_ms = 5000);

  bool has(const std::string & tool_name) const;

private:
  bool callTrigger(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    const std::string & label,
    int timeout_ms);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::unordered_map<std::string, Entry> tools_;
};

}  // namespace industrial_bt_framework
