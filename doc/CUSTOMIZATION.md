# Customization Guide — industrial_bt_framework

This document explains **where and how to modify every degree of freedom** of
an industrial pick-and-place application built on top of the framework, without
touching framework code.

---

## 1. Architecture at a glance

```
┌─────────────────────────────────┐   ┌──────────────────────────────┐
│  Your bringup package           │──▶│  industrial_bt_framework      │
│   • MoveIt config (URDF/SRDF)   │   │   • bt_runner_node            │
│   • bt_runner.yaml              │   │   • scene_manager_node        │
│   • motion_profiles.yaml        │   │   • CartesianBackend plugins  │
│   • scene.yaml                  │   │   • Generic BT primitives     │
│   • BT tree (XML)               │   │                              │
│   • optional vendor backend     │   │                              │
└─────────────────────────────────┘   └──────────────────────────────┘
```

The framework contains no robot-, tool-, or part-specific code. Everything
is loaded from YAML + BT XML at launch.

---

## 2. Adapting to a new robot

| What changes | Where |
|---|---|
| URDF / SRDF / kinematics / controllers | Your own MoveIt config package (pass via launch arg `moveit_config_pkg`). |
| Planning group name | `bt_runner.yaml`: `planning_group`. |
| SRDF home target | `bt_runner.yaml`: `home_named_target`. |
| Cartesian motion backend | `bt_runner.yaml`: `cartesian_backend`. Default is `industrial_bt_framework/MoveItCartesianBackend` — works out of the box for any MoveIt robot. |
| Vendor-specific linear motion | Write a new `CartesianBackend` plugin in a separate package. See §5. |
| Wrist-to-flange (or tool) geometric offset | `cartesian_backend_params_file` (vendor plugin reads it). Never hard-coded in the framework. |

---

## 3. Adapting to a new application

| What changes | Where |
|---|---|
| The task (sequence, retries, parallelism) | `bt_trees/*.xml` — edit in Groot2. |
| Joint targets, pick/place XYZs, retreat distances, tool offsets | `bt_runner.yaml` → `task_parameters:`. Every key under `task_parameters` is auto-loaded into the BT blackboard under its short name; reference it from XML via `{key}`. |
| Motion speeds / tolerances / retries | `motion_profiles.yaml`, one profile per movement category. |
| End-effectors and their activation services | `bt_runner.yaml` → `tools:` (`activate_service`, `release_service`, std_srvs/Trigger). |
| Scene contents (tables, walls, parts) | `scene.yaml` — generic schema, add/remove entries in `object_ids` + `objects.<id>.*`. |
| Dynamic objects (moved at runtime) | Set `dynamic: true` on the object in `scene.yaml`. |
| ACM tweak (allow tool to touch grasped parts) | `scene.yaml`: `attach_acm_links`. |
| Scene settle time after scene ops | `task_parameters.scene_update_wait_ms`. |

---

## 4. BT primitive reference

| Node | Ports | Behavior |
|---|---|---|
| `MoveToNamedTarget` | `target`, `profile` | Plan + execute to SRDF named target. |
| `MoveToJointTarget` | `joints_deg` (semicolon-separated), `profile` | OMPL plan to joint target. |
| `ExecuteCartesianSegment` | `pose_key` (blackboard), `profile` | Delegate to the Cartesian backend. |
| `StoreCurrentPose` | `pose_key` | Capture current TCP pose to blackboard. |
| `ComputeTCPTarget` | `target_xyz`, `tcp_offset_xyz`, `tcp_offset_rpy_deg`, `use_current_orientation`, `fixed_rpy_deg`, `out_key` | Build a TCP-target Pose. |
| `OffsetPoseInToolFrame` | `in_key`, `dx`, `dy`, `dz`, `out_key` | Offset a pose along its own tool frame (e.g. retreats). |
| `ActivateTool`, `ReleaseTool` | `tool` (registry name) | Call the tool's Trigger service. |
| `RemoveSceneObject` | `object_id` | `/scene/remove_object`. |
| `TeleportSceneObject` | `object_id`, `pose_key` | `/scene/set_object_pose`. |
| `WaitMs`, `LogMessage` | `duration_ms` / `message` | Utility. |

All ports accept `{blackboard_key}` references, so tasks are fully data-driven.

---

## 5. Writing a new Cartesian backend plugin

Vendor controllers sometimes expose their own linear motion service. To use it:

1. Create a new ROS 2 package that depends on `industrial_bt_framework` and `pluginlib`.
2. Subclass `industrial_bt_framework::CartesianBackend`:

    ```cpp
    #include "industrial_bt_framework/cartesian_backend.hpp"
    #include <pluginlib/class_list_macros.hpp>

    class MyVendorBackend : public industrial_bt_framework::CartesianBackend
    {
    public:
      void initialize(rclcpp::Node::SharedPtr node,
                      std::shared_ptr<moveit::planning_interface::MoveGroupInterface>,
                      const std::string & planning_group,
                      const YAML::Node & params) override {
        // read params["cartesian_service_name"], params["flange_offset_xyz"], ...
      }
      bool executeLinear(const geometry_msgs::msg::Pose & tcp,
                         const industrial_bt_framework::MotionProfile & p,
                         const std::string & desc) override {
        // call your vendor service
      }
    };

    PLUGINLIB_EXPORT_CLASS(
      my_pkg::MyVendorBackend,
      industrial_bt_framework::CartesianBackend)
    ```

3. Export it via `plugin_description.xml`:

    ```xml
    <library path="my_pkg_backends">
      <class name="my_pkg/MyVendorBackend"
             type="my_pkg::MyVendorBackend"
             base_class_type="industrial_bt_framework::CartesianBackend"/>
    </library>
    ```

4. In `bt_runner.yaml` set:

    ```yaml
    cartesian_backend: "my_pkg/MyVendorBackend"
    cartesian_backend_params_file: "/path/to/my_vendor_params.yaml"
    ```

No framework change is needed to add a new robot.

---

## 6. What NOT to edit

The following is framework-internal and should not change from application to application:

- `bt_nodes.cpp` — BT primitive implementations.
- `robot_client.cpp` — execution model (Reentrant callbacks + MultiThreadedExecutor).
- `scene_manager_node.cpp` — force-republish logic.
- The `CartesianBackend` abstract base class.

If you find yourself wanting to edit these, it is usually a sign that you
should add a parameter, a new BT primitive, or a new backend plugin instead.
