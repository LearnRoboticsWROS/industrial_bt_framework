#pragma once

#include "motion_profile.hpp"
#include "robot_client.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

namespace industrial_bt_framework {

// Blackboard key constants. Populated by bt_runner_node at startup.
inline constexpr const char * BB_ROBOT_CLIENT    = "robot_client";
inline constexpr const char * BB_MOTION_PROFILES = "motion_profiles";
inline constexpr const char * BB_ROS_NODE        = "ros_node";

// Convert vector in degrees to radians.
std::vector<double> degVecToRad(const std::vector<double> & deg);

// Parse semicolon-separated "a;b;c" into a fixed-size array or vector.
std::array<double, 3> parseXyz3(const std::string & s);
std::vector<double>   parseVec  (const std::string & s);

// ═════════════════════════════════════════════════════════════════════════════
// PRIMITIVE NODES
// ═════════════════════════════════════════════════════════════════════════════

class MoveToNamedTarget : public BT::SyncActionNode
{
public:
  MoveToNamedTarget(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class MoveToJointTarget : public BT::SyncActionNode
{
public:
  MoveToJointTarget(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class ExecuteCartesianSegment : public BT::SyncActionNode
{
public:
  ExecuteCartesianSegment(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// Joint-space plan to a Cartesian target pose. Seeds IK with the robot's
// current joint state so the nearest IK branch is chosen, then interpolates
// linearly in joint space — produces a short, smooth, deterministic motion
// even across large end-effector orientation changes. Falls back to MoveIt's
// setPoseTarget+OMPL planner if IK fails.
class MoveToPose : public BT::SyncActionNode
{
public:
  MoveToPose(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class StoreCurrentPose : public BT::SyncActionNode
{
public:
  StoreCurrentPose(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// Generalised wrist-target computation.
// Reads a task XYZ + TCP offset (xyz/rpy) and writes a TCP-target pose to the
// blackboard. Can either keep the robot's current orientation (use_current_orientation=true)
// or use a fixed RPY (fixed_rpy_deg).
class ComputeTCPTarget : public BT::SyncActionNode
{
public:
  ComputeTCPTarget(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// Offset a pose stored in the blackboard by (dx, dy, dz) expressed in its
// own tool frame. Output written under out_key.
class OffsetPoseInToolFrame : public BT::SyncActionNode
{
public:
  OffsetPoseInToolFrame(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// Offset a pose stored in the blackboard by (dx, dy, dz) expressed in the
// base (world) frame. Output written under out_key.
class OffsetPoseInBaseFrame : public BT::SyncActionNode
{
public:
  OffsetPoseInBaseFrame(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// ═════════════════════════════════════════════════════════════════════════════
// TOOL / SCENE NODES
// ═════════════════════════════════════════════════════════════════════════════

class ActivateTool : public BT::SyncActionNode
{
public:
  ActivateTool(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class ReleaseTool : public BT::SyncActionNode
{
public:
  ReleaseTool(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class RemoveSceneObject : public BT::SyncActionNode
{
public:
  RemoveSceneObject(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class TeleportSceneObject : public BT::SyncActionNode
{
public:
  TeleportSceneObject(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// ═════════════════════════════════════════════════════════════════════════════
// UTILITY NODES
// ═════════════════════════════════════════════════════════════════════════════

class WaitMs : public BT::SyncActionNode
{
public:
  WaitMs(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class LogMessage : public BT::SyncActionNode
{
public:
  LogMessage(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// ═════════════════════════════════════════════════════════════════════════════
// VISION / PERCEPTION NODES
// ═════════════════════════════════════════════════════════════════════════════

// Subscribe to a PoseArray topic, pick a pose by index, transform it to a
// target TF frame via tf2, and store the resulting geometry_msgs::msg::Pose
// in the blackboard under `out_key`. The PerceptionClient is cached per
// topic so multiple ticks share one subscription.
class GetDetectedObjectPose : public BT::SyncActionNode
{
public:
  GetDetectedObjectPose(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// Given a detected object pose expressed in base_link (Z axis = plane normal
// pointing toward the camera), compute a wrist3 target pose such that a
// suction-cup end-effector lands on the object with its axis anti-parallel
// to the surface normal. `tool_offset_z` is the distance from wrist3 to the
// suction-cup tip along the tool Z axis.
class ComputeSuctionPickPose : public BT::SyncActionNode
{
public:
  ComputeSuctionPickPose(const std::string & n, const BT::NodeConfig & c);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// Register every framework BT node with the given factory.
void registerAllNodes(BT::BehaviorTreeFactory & factory);

}  // namespace industrial_bt_framework
