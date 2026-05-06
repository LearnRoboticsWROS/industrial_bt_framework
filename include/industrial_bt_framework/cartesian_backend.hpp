#pragma once

#include "motion_profile.hpp"

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <yaml-cpp/yaml.h>

namespace moveit { namespace planning_interface { class MoveGroupInterface; } }

namespace industrial_bt_framework {

// Abstract base class for Cartesian (linear TCP) motion backends.
// Implementations are pluginlib-loaded at runtime.
class CartesianBackend
{
public:
  virtual ~CartesianBackend() = default;

  // Called once at startup. `params` is the `cartesian_backend_params` YAML
  // subtree from bt_runner.yaml (may be null).
  virtual void initialize(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    const std::string & planning_group,
    const YAML::Node & params) = 0;

  // Move the robot TCP linearly from its current pose to `tcp_target_pose`
  // (expressed in the robot's planning frame / base_link).
  // Implementations MAY apply an internal flange<->TCP offset.
  virtual bool executeLinear(
    const geometry_msgs::msg::Pose & tcp_target_pose,
    const MotionProfile & profile,
    const std::string & description) = 0;
};

}  // namespace industrial_bt_framework
