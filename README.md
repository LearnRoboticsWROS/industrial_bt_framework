# 🤖 industrial_bt_framework

> A **robot-agnostic Behavior Tree framework** for ROS 2 industrial pick-and-place applications.

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](LICENSE)
[![BehaviorTree.CPP](https://img.shields.io/badge/BT-BehaviorTree.CPP-orange)](https://www.behaviortree.dev/)

---

## What it is

`industrial_bt_framework` is the **framework layer** of a three-tier industrial robotics architecture. It provides everything a generic pick-and-place application needs — Behavior Tree executor, reusable motion bricks, perception client, scene manager, Cartesian backend interface — and **knows nothing about a specific robot brand, gripper, or task**.

You build a concrete application by:
1. **Cloning this framework** as a dependency.
2. **Writing a small application package** (~ XML + YAML + 1 plugin) that adapts the framework to your robot.

The result: motion logic, vision integration, scene management, and tool services are all reusable across robots.

---

## Where it sits in the stack

```
┌──────────────────────────────────────────────────────┐
│ APPLICATION LAYER (e.g. fr3wml_industrial_bt)        │
│ Robot-specific YAML + BT XML + Cartesian backend     │
└──────────────────────────────────────────────────────┘
                         ▼
┌──────────────────────────────────────────────────────┐
│ industrial_bt_framework  ← YOU ARE HERE              │
│ Robot-agnostic BT bricks, executor, clients          │
└──────────────────────────────────────────────────────┘
                         ▼
┌──────────────────────────────────────────────────────┐
│ DRIVER LAYER (e.g. fairino_bridge)                   │
│ Vendor SDK wrapped in standard ROS 2 services        │
└──────────────────────────────────────────────────────┘
```

---

## What's inside

### 🌳 BT executors

| Executable | Description |
|---|---|
| `bt_runner_node` | Headless executor — loads YAML + BT XML, ticks the tree |
| `bt_runner_node_visual` | Same + Groot2 publisher (TCP `1666`) + `.btlog` file logger |
| `scene_manager_node` | Owns the planning scene, exposes `/scene/remove_object` and `/scene/set_object_pose` services, republishes attached objects |

### 🧱 Reusable BT bricks (16)

| Brick | Purpose |
|---|---|
| `MoveToNamedTarget` | PTP to a named SRDF target (e.g. `home`) |
| `MoveToJointTarget` | PTP to explicit joint angles (degrees) |
| `MoveToPose` | PTP to a Cartesian pose (IK seeded at current state) |
| `ExecuteCartesianSegment` | Linear motion via the active `CartesianBackend` plugin |
| `StoreCurrentPose` | Snapshot current end-effector pose into the blackboard |
| `ComputeTCPTarget` | Build a TCP-frame target pose from XYZ + tool offset |
| `OffsetPoseInBaseFrame` | Offset a pose in `base_link` (scalar `dx/dy/dz` or vec3 `xyz`) |
| `OffsetPoseInToolFrame` | Offset a pose along its own tool axes |
| `ActivateTool` / `ReleaseTool` | Call gripper/suction Trigger services via the registry |
| `RemoveSceneObject` / `TeleportSceneObject` | Manipulate planning scene objects |
| `WaitMs` | Sleep N ms |
| `LogMessage` | Log a message to the BT log |
| `GetDetectedObjectPose` | Subscribe to a `PoseArray` topic + tf2-transform the result |
| `ComputeSuctionPickPose` | From a detected normal-aligned pose, build the wrist3 target with the suction cup perpendicular to the surface |

### 🔌 Pluginlib `CartesianBackend` interface

Header: [`include/industrial_bt_framework/cartesian_backend.hpp`](include/industrial_bt_framework/cartesian_backend.hpp)

Vendors implement `executeLinear()` and register it via `pluginlib`. The framework includes a default `MoveItCartesianBackend` plugin built on `computeCartesianPath()`. Vendor-specific plugins (e.g. `FairinoMoveLBackend`) live in application packages.

### 🛠️ Utility clients

- `RobotClient` — facade over `MoveGroupInterface` for MoveJ + MoveL
- `ToolRegistry` — Trigger service registry for grippers and suction
- `SceneClient` — service client for the scene manager
- `PerceptionClient` — `PoseArray` subscription + `tf2_ros::Buffer` for vision input

---

## Installation

```bash
cd ~/your_ws/src
git clone https://github.com/LearnRoboticsWROS/industrial_bt_framework.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select industrial_bt_framework --symlink-install
source install/setup.bash
```

Dependencies: ROS 2 Humble, MoveIt 2, [BehaviorTree.CPP v4](https://www.behaviortree.dev/), `tf2_ros`, `cv_bridge` (only for the vision bricks).

---

## Quick start

The framework is not meant to run standalone — you launch it from an application package. See [`fr3wml_industrial_bt`](https://github.com/LearnRoboticsWROS/fr3wml_industrial_bt) for a complete example.

A minimal launch composes 3 nodes:

```python
scene_manager = Node(package="industrial_bt_framework", executable="scene_manager_node",
                     parameters=[scene_yaml])

bt_runner = Node(package="industrial_bt_framework", executable="bt_runner_node",
                 parameters=[bt_runner_yaml, motion_profiles_yaml,
                             {"bt_tree_file": "your_tree.xml",
                              "bt_tree_id":   "MainTree",
                              "cartesian_backend": "your_vendor/YourBackend"}])
```

---

## YAML configuration

Every BT brick reads its inputs from the blackboard, which is populated at startup by the runner from your YAML:

```yaml
/**:
  ros__parameters:
    planning_group:    "your_planning_group"
    home_named_target: "home"
    cartesian_backend: "your_vendor/YourBackend"

    tools:
      gripper_primary:
        activate_service: "/gripper/close"
        release_service:  "/gripper/idle"

    task_parameters:
      # Any key here lands on the BT blackboard under its short name.
      # BT XML reads it with {key}.
      pick_xyz: [0.20, -0.42, 0.46]
      retreat_m: 0.10
```

`motion_profiles.yaml` defines per-segment velocity/acceleration scaling and vendor MoveL speed.

---

## Adding your own BT brick

1. Add the class declaration to [`include/industrial_bt_framework/bt_nodes.hpp`](include/industrial_bt_framework/bt_nodes.hpp).
2. Implement `tick()` and `providedPorts()` in [`src/bt_nodes.cpp`](src/bt_nodes.cpp).
3. Register it in `registerAllNodes(factory)` at the bottom of the same file.

That's it — your brick is now available in any BT XML.

---

## Related repositories

| Layer | Repository |
|---|---|
| Application (FR3WML example) | [fr3wml_industrial_bt](https://github.com/LearnRoboticsWROS/fr3wml_industrial_bt) |
| Driver (Fairino robot) | [fairino_bridge](https://github.com/LearnRoboticsWROS/fairino_bridge_master_class) |
| Driver (Fairino gripper) | [fairino_gripper](https://github.com/LearnRoboticsWROS/fairino_bridge_gripper_master_class) |
| Jetson vision (YOLO) | [inference_running_jetson](https://github.com/LearnRoboticsWROS/inference_running_jetson) |
| Jetson vision (6D pose) | [sixd_pose_pcl](https://github.com/LearnRoboticsWROS/sixd_pose_pcl) |

---

## License

Apache 2.0. See [LICENSE](LICENSE).

---

Built with ❤️ for Learn Robotics with ROS.
