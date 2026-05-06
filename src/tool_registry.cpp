#include "industrial_bt_framework/tool_registry.hpp"

#include <chrono>

namespace industrial_bt_framework {

ToolRegistry::ToolRegistry(
  rclcpp::Node::SharedPtr node,
  rclcpp::CallbackGroup::SharedPtr callback_group)
: node_(node), cb_group_(callback_group) {}

void ToolRegistry::registerTool(
  const std::string & name,
  const std::string & activate_service,
  const std::string & release_service)
{
  Entry e;
  e.activate_service = activate_service;
  e.release_service  = release_service;
  rclcpp::SubscriptionOptions opts;
  e.activate_client = node_->create_client<std_srvs::srv::Trigger>(
    activate_service, rmw_qos_profile_services_default, cb_group_);
  e.release_client = node_->create_client<std_srvs::srv::Trigger>(
    release_service, rmw_qos_profile_services_default, cb_group_);
  tools_[name] = e;
  RCLCPP_INFO(node_->get_logger(),
    "ToolRegistry: registered '%s' → activate='%s' release='%s'",
    name.c_str(), activate_service.c_str(), release_service.c_str());
}

bool ToolRegistry::has(const std::string & name) const
{
  return tools_.find(name) != tools_.end();
}

bool ToolRegistry::callTrigger(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
  const std::string & label,
  int timeout_ms)
{
  auto logger = node_->get_logger();
  if (!client->wait_for_service(std::chrono::milliseconds(timeout_ms))) {
    RCLCPP_ERROR(logger, "[%s] service unavailable.", label.c_str());
    return false;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(req);
  if (future.wait_for(std::chrono::milliseconds(timeout_ms)) != std::future_status::ready) {
    RCLCPP_ERROR(logger, "[%s] call timed out after %d ms.", label.c_str(), timeout_ms);
    return false;
  }
  auto resp = future.get();
  if (!resp->success) {
    RCLCPP_ERROR(logger, "[%s] service reported failure: %s",
                 label.c_str(), resp->message.c_str());
    return false;
  }
  return true;
}

bool ToolRegistry::activate(const std::string & name, int timeout_ms)
{
  auto it = tools_.find(name);
  if (it == tools_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "ToolRegistry: unknown tool '%s'.", name.c_str());
    return false;
  }
  return callTrigger(it->second.activate_client,
                     "activate:" + name + " (" + it->second.activate_service + ")",
                     timeout_ms);
}

bool ToolRegistry::release(const std::string & name, int timeout_ms)
{
  auto it = tools_.find(name);
  if (it == tools_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "ToolRegistry: unknown tool '%s'.", name.c_str());
    return false;
  }
  return callTrigger(it->second.release_client,
                     "release:" + name + " (" + it->second.release_service + ")",
                     timeout_ms);
}

}  // namespace industrial_bt_framework
