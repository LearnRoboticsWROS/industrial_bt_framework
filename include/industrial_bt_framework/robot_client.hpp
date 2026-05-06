#pragma once

#include "motion_profile.hpp"
#include "cartesian_backend.hpp"
#include "tool_registry.hpp"
#include "scene_client.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

namespace industrial_bt_framework {

// Robot-agnostic facade holding:
//   - MoveGroupInterface for joint / named-target motion
//   - a pluginlib-loaded CartesianBackend for linear TCP motion
//   - a ToolRegistry for activating/releasing end-effectors
//   - a SceneClient for scene ops delegated to scene_manager_node
class RobotClient
{
public:
  RobotClient(
    rclcpp::Node::SharedPtr node,
    const std::string & planning_group,
    std::shared_ptr<CartesianBackend> cartesian_backend,
    std::shared_ptr<ToolRegistry> tools,
    std::shared_ptr<SceneClient> scene);

  // ── Free-space (OMPL) motion ─────────────────────────────────────────────
  bool moveToNamedTarget(
    const std::string & target,
    const MotionProfile & profile,
    const std::string & desc = "move_to_named");

  bool moveToJointTarget(
    const std::vector<double> & joints_rad,
    const MotionProfile & profile,
    const std::string & desc = "move_to_joint");

  // Plan to a Cartesian pose as a MoveJ (joint-space) motion. IK is solved
  // seeded with the robot's current joint state so the nearest branch is
  // chosen — avoids large wrist flips and OMPL detours. Falls back to
  // MoveIt's setPoseTarget + OMPL plan if IK fails.
  bool moveToPose(
    const geometry_msgs::msg::Pose & pose,
    const MotionProfile & profile,
    const std::string & desc = "move_to_pose");

  // ── Cartesian motion (delegates to backend) ──────────────────────────────
  bool executeLinear(
    const geometry_msgs::msg::Pose & tcp_target_pose,
    const MotionProfile & profile,
    const std::string & desc = "cartesian");

  // ── Queries / helpers ────────────────────────────────────────────────────
  geometry_msgs::msg::Pose getCurrentPose();

  // Offset a pose along its own tool frame (+Z typically = "retreat").
  static geometry_msgs::msg::Pose offsetInToolFrame(
    const geometry_msgs::msg::Pose & in,
    double dx, double dy, double dz);

  // Apply a TCP offset (m + rpy deg) to a pose expressed in base_link.
  // Used to translate a target task-frame pose to a TCP target.
  static geometry_msgs::msg::Pose applyTCPOffset(
    const geometry_msgs::msg::Pose & base_pose,
    const std::array<double, 3> & tcp_xyz,
    const std::array<double, 3> & tcp_rpy_deg);

  // ── Accessors ────────────────────────────────────────────────────────────
  std::shared_ptr<ToolRegistry>  tools() { return tools_; }
  std::shared_ptr<SceneClient>   scene() { return scene_; }
  std::shared_ptr<CartesianBackend> backend() { return backend_; }
  std::string getPlanningFrame() const;
  std::string getPlanningGroup() const { return planning_group_; }
  rclcpp::Logger getLogger() const;

private:
  void applyProfile(const MotionProfile & p);
  bool planAndExecuteWithRetries(const MotionProfile & p, const std::string & desc);

  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<CartesianBackend> backend_;
  std::shared_ptr<ToolRegistry> tools_;
  std::shared_ptr<SceneClient> scene_;
};

}  // namespace industrial_bt_framework
