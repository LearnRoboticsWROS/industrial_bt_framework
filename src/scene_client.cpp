#include "industrial_bt_framework/scene_client.hpp"

#include <chrono>

namespace industrial_bt_framework {

SceneClient::SceneClient(
  rclcpp::Node::SharedPtr node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  const std::string & remove_service,
  const std::string & set_pose_service)
: node_(node), cb_group_(callback_group)
{
  remove_client_ = node_->create_client<industrial_bt_framework::srv::RemoveSceneObject>(
    remove_service, rmw_qos_profile_services_default, cb_group_);
  set_pose_client_ = node_->create_client<industrial_bt_framework::srv::SetScenePose>(
    set_pose_service, rmw_qos_profile_services_default, cb_group_);
}

bool SceneClient::removeObject(const std::string & object_id, int timeout_ms)
{
  auto logger = node_->get_logger();
  if (!remove_client_->wait_for_service(std::chrono::milliseconds(timeout_ms))) {
    RCLCPP_ERROR(logger, "SceneClient.removeObject: service unavailable.");
    return false;
  }
  auto req = std::make_shared<industrial_bt_framework::srv::RemoveSceneObject::Request>();
  req->object_id = object_id;
  auto future = remove_client_->async_send_request(req);
  if (future.wait_for(std::chrono::milliseconds(timeout_ms)) != std::future_status::ready) {
    RCLCPP_ERROR(logger, "SceneClient.removeObject('%s'): timed out.", object_id.c_str());
    return false;
  }
  auto resp = future.get();
  if (!resp->success) {
    RCLCPP_ERROR(logger, "SceneClient.removeObject('%s') failed: %s",
                 object_id.c_str(), resp->message.c_str());
    return false;
  }
  return true;
}

bool SceneClient::setObjectPose(
  const std::string & object_id,
  const geometry_msgs::msg::Pose & pose,
  int timeout_ms)
{
  auto logger = node_->get_logger();
  if (!set_pose_client_->wait_for_service(std::chrono::milliseconds(timeout_ms))) {
    RCLCPP_ERROR(logger, "SceneClient.setObjectPose: service unavailable.");
    return false;
  }
  auto req = std::make_shared<industrial_bt_framework::srv::SetScenePose::Request>();
  req->object_id = object_id;
  req->pose = pose;
  auto future = set_pose_client_->async_send_request(req);
  if (future.wait_for(std::chrono::milliseconds(timeout_ms)) != std::future_status::ready) {
    RCLCPP_ERROR(logger, "SceneClient.setObjectPose('%s'): timed out.", object_id.c_str());
    return false;
  }
  auto resp = future.get();
  if (!resp->success) {
    RCLCPP_ERROR(logger, "SceneClient.setObjectPose('%s') failed: %s",
                 object_id.c_str(), resp->message.c_str());
    return false;
  }
  return true;
}

}  // namespace industrial_bt_framework
