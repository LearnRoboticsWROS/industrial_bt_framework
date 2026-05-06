#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "industrial_bt_framework/srv/remove_scene_object.hpp"
#include "industrial_bt_framework/srv/set_scene_pose.hpp"

namespace industrial_bt_framework {

// Thin client that BT primitive nodes use to manipulate the collision scene
// via the scene_manager_node. Calls are synchronous (waited on) with a
// configurable timeout.
class SceneClient
{
public:
  SceneClient(
    rclcpp::Node::SharedPtr node,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    const std::string & remove_service = "/scene/remove_object",
    const std::string & set_pose_service = "/scene/set_object_pose");

  bool removeObject(const std::string & object_id, int timeout_ms = 5000);
  bool setObjectPose(const std::string & object_id,
                     const geometry_msgs::msg::Pose & pose,
                     int timeout_ms = 5000);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Client<industrial_bt_framework::srv::RemoveSceneObject>::SharedPtr remove_client_;
  rclcpp::Client<industrial_bt_framework::srv::SetScenePose>::SharedPtr set_pose_client_;
};

}  // namespace industrial_bt_framework
