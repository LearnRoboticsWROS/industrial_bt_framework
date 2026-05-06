// Default Cartesian backend implementation using MoveIt's computeCartesianPath.
// Works for any MoveIt-configured robot out of the box. No vendor-specific deps.

#include "industrial_bt_framework/cartesian_backend.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <pluginlib/class_list_macros.hpp>

using moveit::planning_interface::MoveGroupInterface;

namespace industrial_bt_framework {

class MoveItCartesianBackend : public CartesianBackend
{
public:
  void initialize(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<MoveGroupInterface> move_group,
    const std::string & planning_group,
    const YAML::Node & params) override
  {
    node_ = node;
    planning_group_ = planning_group;
    move_group_ = move_group;
    if (!move_group_) {
      move_group_ = std::make_shared<MoveGroupInterface>(node_, planning_group_);
    }
    if (params && params["eef_step"])
      eef_step_ = params["eef_step"].as<double>();
    if (params && params["jump_threshold"])
      jump_threshold_ = params["jump_threshold"].as<double>();
    if (params && params["min_fraction"])
      min_fraction_ = params["min_fraction"].as<double>();
    RCLCPP_INFO(node_->get_logger(),
      "MoveItCartesianBackend ready (eef_step=%.4f, jump=%.2f, min_fraction=%.3f)",
      eef_step_, jump_threshold_, min_fraction_);
  }

  bool executeLinear(
    const geometry_msgs::msg::Pose & target,
    const MotionProfile & profile,
    const std::string & desc) override
  {
    auto logger = node_->get_logger();
    RCLCPP_INFO(logger, "[%s] MoveIt Cartesian → (%.3f, %.3f, %.3f)",
                desc.c_str(), target.position.x, target.position.y, target.position.z);

    move_group_->setMaxVelocityScalingFactor(profile.velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(profile.acceleration_scaling);
    move_group_->setStartStateToCurrentState();

    // Robust step-based interpolation like the reference framework.
    while (rclcpp::ok()) {
      const auto cur = move_group_->getCurrentPose().pose;
      const Eigen::Vector3d p_cur(cur.position.x, cur.position.y, cur.position.z);
      const Eigen::Vector3d p_tgt(target.position.x, target.position.y, target.position.z);
      const Eigen::Vector3d dp = p_tgt - p_cur;
      const double dist = dp.norm();
      if (dist <= profile.cartesian_final_position_tol) {
        RCLCPP_INFO(logger, "[%s] Cartesian target reached.", desc.c_str());
        return true;
      }

      double step = std::min(dist, profile.cartesian_primary_step_m);
      double alpha = step / dist;

      Eigen::Quaterniond q_cur(cur.orientation.w, cur.orientation.x, cur.orientation.y, cur.orientation.z);
      Eigen::Quaterniond q_tgt(target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);
      q_cur.normalize(); q_tgt.normalize();
      Eigen::Quaterniond q_next = q_cur.slerp(alpha, q_tgt);
      q_next.normalize();
      const Eigen::Vector3d p_next = p_cur + alpha * dp;

      geometry_msgs::msg::Pose next;
      next.position.x = p_next.x();
      next.position.y = p_next.y();
      next.position.z = p_next.z();
      next.orientation.x = q_next.x();
      next.orientation.y = q_next.y();
      next.orientation.z = q_next.z();
      next.orientation.w = q_next.w();

      if (stepOnce(next, profile, desc)) continue;

      // Fallback with smaller step.
      step = std::min(dist, profile.cartesian_fallback_step_m);
      alpha = step / dist;
      q_next = q_cur.slerp(alpha, q_tgt);
      q_next.normalize();
      const Eigen::Vector3d p_next2 = p_cur + alpha * dp;
      next.position.x = p_next2.x();
      next.position.y = p_next2.y();
      next.position.z = p_next2.z();
      next.orientation.x = q_next.x();
      next.orientation.y = q_next.y();
      next.orientation.z = q_next.z();
      next.orientation.w = q_next.w();

      if (stepOnce(next, profile, desc)) continue;

      RCLCPP_ERROR(logger, "[%s] Cartesian fallback step failed.", desc.c_str());
      return false;
    }
    return false;
  }

private:
  bool stepOnce(
    const geometry_msgs::msg::Pose & waypoint,
    const MotionProfile & profile,
    const std::string & desc)
  {
    auto logger = node_->get_logger();
    move_group_->setStartStateToCurrentState();
    std::vector<geometry_msgs::msg::Pose> wps{waypoint};
    moveit_msgs::msg::RobotTrajectory traj;
    moveit_msgs::msg::MoveItErrorCodes err;
    const double eef = profile.cartesian_eef_step > 0.0 ? profile.cartesian_eef_step : eef_step_;
    const double jump = jump_threshold_;
    const double frac = move_group_->computeCartesianPath(wps, eef, jump, traj, true, &err);
    const double min_frac = profile.cartesian_min_fraction > 0.0 ? profile.cartesian_min_fraction : min_fraction_;
    if (frac < min_frac) {
      RCLCPP_WARN(logger, "[%s] computeCartesianPath fraction=%.3f < %.3f.",
                  desc.c_str(), frac, min_frac);
      return false;
    }
    MoveGroupInterface::Plan plan;
    plan.trajectory_ = traj;
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  std::shared_ptr<MoveGroupInterface> move_group_;
  double eef_step_ = 0.003;
  double jump_threshold_ = 0.0;
  double min_fraction_ = 0.999;
};

}  // namespace industrial_bt_framework

PLUGINLIB_EXPORT_CLASS(
  industrial_bt_framework::MoveItCartesianBackend,
  industrial_bt_framework::CartesianBackend)
