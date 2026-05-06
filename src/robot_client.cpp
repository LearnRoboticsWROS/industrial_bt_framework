#include "industrial_bt_framework/robot_client.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include <Eigen/Geometry>

#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

using moveit::planning_interface::MoveGroupInterface;

namespace industrial_bt_framework {

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

static Eigen::Quaterniond quatFromMsg(const geometry_msgs::msg::Quaternion & m)
{
  Eigen::Quaterniond q(m.w, m.x, m.y, m.z);
  q.normalize();
  return q;
}

static geometry_msgs::msg::Pose makePose(const Eigen::Vector3d & p, const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = p.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

static Eigen::Isometry3d isometryFromMsg(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  T.linear() = quatFromMsg(pose.orientation).toRotationMatrix();
  return T;
}

static Eigen::Isometry3d isometryFromXyzRpy(
  const std::array<double, 3> & xyz, const std::array<double, 3> & rpy_deg)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  T.linear() = quatFromRpyDeg(rpy_deg).toRotationMatrix();
  return T;
}

static void waitMs(int ms) { rclcpp::sleep_for(std::chrono::milliseconds(ms)); }

// ─────────────────────────────────────────────────────────────────────────────

RobotClient::RobotClient(
  rclcpp::Node::SharedPtr node,
  const std::string & planning_group,
  std::shared_ptr<CartesianBackend> cartesian_backend,
  std::shared_ptr<ToolRegistry> tools,
  std::shared_ptr<SceneClient> scene)
: node_(node),
  planning_group_(planning_group),
  backend_(std::move(cartesian_backend)),
  tools_(std::move(tools)),
  scene_(std::move(scene))
{
  move_group_ = std::make_shared<MoveGroupInterface>(node_, planning_group_);
}

void RobotClient::applyProfile(const MotionProfile & p)
{
  move_group_->setPlanningPipelineId("ompl");
  move_group_->setPlannerId("RRTConnectkConfigDefault");
  move_group_->setPlanningTime(p.planning_time_sec);
  move_group_->setNumPlanningAttempts(p.num_planning_attempts);
  move_group_->setMaxVelocityScalingFactor(p.velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(p.acceleration_scaling);
  move_group_->setGoalPositionTolerance(p.goal_position_tolerance);
  move_group_->setGoalOrientationTolerance(p.goal_orientation_tolerance);
  move_group_->clearPathConstraints();
}

bool RobotClient::planAndExecuteWithRetries(const MotionProfile & p, const std::string & desc)
{
  auto logger = node_->get_logger();
  for (int attempt = 1; attempt <= p.max_free_space_retries; ++attempt) {
    move_group_->setStartStateToCurrentState();
    MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      auto exec = move_group_->execute(plan);
      if (exec == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "[%s] Succeeded on attempt %d/%d.",
                    desc.c_str(), attempt, p.max_free_space_retries);
        return true;
      }
      RCLCPP_WARN(logger, "[%s] Execution failed on attempt %d/%d.",
                  desc.c_str(), attempt, p.max_free_space_retries);
    } else {
      RCLCPP_WARN(logger, "[%s] Planning failed on attempt %d/%d.",
                  desc.c_str(), attempt, p.max_free_space_retries);
    }
    waitMs(p.retry_sleep_ms);
  }
  RCLCPP_ERROR(logger, "[%s] Exhausted all %d retries.", desc.c_str(), p.max_free_space_retries);
  return false;
}

bool RobotClient::moveToNamedTarget(
  const std::string & target, const MotionProfile & profile, const std::string & desc)
{
  applyProfile(profile);
  move_group_->clearPoseTargets();
  move_group_->setNamedTarget(target);
  return planAndExecuteWithRetries(profile, desc);
}

bool RobotClient::moveToJointTarget(
  const std::vector<double> & joints_rad, const MotionProfile & profile, const std::string & desc)
{
  applyProfile(profile);
  move_group_->clearPoseTargets();
  move_group_->setJointValueTarget(joints_rad);
  return planAndExecuteWithRetries(profile, desc);
}

bool RobotClient::executeLinear(
  const geometry_msgs::msg::Pose & tcp_target_pose,
  const MotionProfile & profile,
  const std::string & desc)
{
  if (!backend_) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] No Cartesian backend configured.", desc.c_str());
    return false;
  }
  return backend_->executeLinear(tcp_target_pose, profile, desc);
}

bool RobotClient::moveToPose(
  const geometry_msgs::msg::Pose & pose,
  const MotionProfile & profile,
  const std::string & desc)
{
  applyProfile(profile);
  move_group_->clearPoseTargets();

  // Strategy A: seed IK at the current joint state so the IK resolver
  // picks the branch closest to where the robot currently is. Then do a
  // MoveJ (joint-space interpolation) to those joints — gives the short,
  // intuitive trajectory the industrial pattern wants, without OMPL
  // exploring the full configuration space.
  auto current_state = move_group_->getCurrentState(0.5);
  if (current_state) {
    const auto * jmg = current_state->getJointModelGroup(planning_group_);
    if (jmg) {
      moveit::core::RobotState target_state(*current_state);
      const double ik_timeout = 0.1;
      if (target_state.setFromIK(jmg, pose, ik_timeout)) {
        std::vector<double> joints;
        target_state.copyJointGroupPositions(jmg, joints);

        std::vector<double> cur_joints;
        current_state->copyJointGroupPositions(jmg, cur_joints);

        std::ostringstream os;
        os << "[" << desc << "] IK (current-state seed) -> joints_deg(";
        for (size_t i = 0; i < joints.size(); ++i) {
          if (i) os << ", ";
          os << (joints[i] * 180.0 / M_PI);
        }
        os << ") delta_deg(";
        for (size_t i = 0; i < joints.size() && i < cur_joints.size(); ++i) {
          if (i) os << ", ";
          os << ((joints[i] - cur_joints[i]) * 180.0 / M_PI);
        }
        os << ")";
        RCLCPP_INFO(node_->get_logger(), "%s", os.str().c_str());

        move_group_->setJointValueTarget(joints);
        return planAndExecuteWithRetries(profile, desc + " (IK+MoveJ)");
      }
      RCLCPP_WARN(node_->get_logger(),
        "[%s] setFromIK(seed=current) failed; falling back to setPoseTarget + OMPL.",
        desc.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "[%s] joint model group '%s' not found in current state.",
        desc.c_str(), planning_group_.c_str());
    }
  } else {
    RCLCPP_WARN(node_->get_logger(),
      "[%s] getCurrentState timed out; falling back to setPoseTarget + OMPL.",
      desc.c_str());
  }

  // Strategy B (fallback): delegate to MoveIt's pose-target planner.
  move_group_->setPoseTarget(pose);
  return planAndExecuteWithRetries(profile, desc + " (PoseTarget)");
}

geometry_msgs::msg::Pose RobotClient::getCurrentPose()
{
  return move_group_->getCurrentPose().pose;
}

geometry_msgs::msg::Pose RobotClient::offsetInToolFrame(
  const geometry_msgs::msg::Pose & in, double dx, double dy, double dz)
{
  Eigen::Isometry3d T = isometryFromMsg(in);
  Eigen::Vector3d delta_tool(dx, dy, dz);
  Eigen::Vector3d delta_base = T.linear() * delta_tool;
  Eigen::Isometry3d T_out = T;
  T_out.translation() += delta_base;
  return makePose(Eigen::Vector3d(T_out.translation()), Eigen::Quaterniond(T_out.rotation()));
}

geometry_msgs::msg::Pose RobotClient::applyTCPOffset(
  const geometry_msgs::msg::Pose & base_pose,
  const std::array<double, 3> & tcp_xyz,
  const std::array<double, 3> & tcp_rpy_deg)
{
  const Eigen::Isometry3d T_base = isometryFromMsg(base_pose);
  const Eigen::Isometry3d T_off  = isometryFromXyzRpy(tcp_xyz, tcp_rpy_deg);
  const Eigen::Isometry3d T_out  = T_base * T_off;
  return makePose(Eigen::Vector3d(T_out.translation()), Eigen::Quaterniond(T_out.rotation()));
}

std::string RobotClient::getPlanningFrame() const { return move_group_->getPlanningFrame(); }
rclcpp::Logger RobotClient::getLogger() const { return node_->get_logger(); }

}  // namespace industrial_bt_framework
