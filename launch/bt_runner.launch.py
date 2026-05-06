#
# Reusable bt_runner launch template. Robot- and application-agnostic:
# bringup packages (e.g. fr3wml_industrial_bt) include this file and pass the
# robot-specific parameters via launch_arguments.
#
# Launch arguments:
#   moveit_config_pkg      name of MoveIt config package for the robot
#   robot_name             robot name passed to MoveItConfigsBuilder
#   bt_params_file         bt_runner.yaml (planning_group, backend, tools, task_parameters)
#   motion_profiles_file   motion_profiles.yaml
#   scene_params_file      scene parameters for scene_manager_node
#   bt_tree_file           BT XML tree
#   bt_tree_id             name of the root tree inside bt_tree_file
#   cartesian_backend_params_file  optional raw YAML for backend-specific params
#   use_visual             if "true", launch bt_runner_node_visual instead
#   zmq_server_port        port for Groot2 monitor (visual only)
#   use_rviz               if "true", launch RViz
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

try:
    from moveit_configs_utils import MoveItConfigsBuilder  # type: ignore
    _HAS_MOVEIT_BUILDER = True
except Exception:  # pragma: no cover
    _HAS_MOVEIT_BUILDER = False


def generate_launch_description():
    moveit_config_pkg = LaunchConfiguration("moveit_config_pkg")
    robot_name = LaunchConfiguration("robot_name")
    bt_params_file = LaunchConfiguration("bt_params_file")
    motion_profiles_file = LaunchConfiguration("motion_profiles_file")
    scene_params_file = LaunchConfiguration("scene_params_file")
    bt_tree_file = LaunchConfiguration("bt_tree_file")
    bt_tree_id = LaunchConfiguration("bt_tree_id")
    cartesian_backend_params_file = LaunchConfiguration("cartesian_backend_params_file")
    use_visual = LaunchConfiguration("use_visual")
    zmq_server_port = LaunchConfiguration("zmq_server_port")
    use_rviz = LaunchConfiguration("use_rviz")

    args = [
        DeclareLaunchArgument("moveit_config_pkg"),
        DeclareLaunchArgument("robot_name"),
        DeclareLaunchArgument("bt_params_file"),
        DeclareLaunchArgument("motion_profiles_file"),
        DeclareLaunchArgument("scene_params_file"),
        DeclareLaunchArgument("bt_tree_file"),
        DeclareLaunchArgument("bt_tree_id", default_value="MainTree"),
        DeclareLaunchArgument("cartesian_backend_params_file", default_value=""),
        DeclareLaunchArgument("use_visual", default_value="false"),
        DeclareLaunchArgument("zmq_server_port", default_value="1666"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]

    # The MoveIt boilerplate (move_group + RSP + RViz + controllers) is best
    # produced by each bringup's own launch, because MoveItConfigsBuilder needs
    # a compile-time package name. This template therefore only starts the
    # BT + scene_manager — the bringup launch adds move_group etc. around us.

    scene_manager = Node(
        package="industrial_bt_framework",
        executable="scene_manager_node",
        name="scene_manager_node",
        output="screen",
        parameters=[scene_params_file],
    )

    bt_runner_headless = Node(
        package="industrial_bt_framework",
        executable="bt_runner_node",
        name="bt_runner_node",
        output="screen",
        parameters=[
            bt_params_file,
            {
                "motion_profiles_file": motion_profiles_file,
                "bt_tree_file": bt_tree_file,
                "bt_tree_id": bt_tree_id,
                "cartesian_backend_params_file": cartesian_backend_params_file,
            },
        ],
        condition=IfCondition(PythonExpression(["'", use_visual, "' == 'false'"])),
    )

    bt_runner_visual = Node(
        package="industrial_bt_framework",
        executable="bt_runner_node_visual",
        name="bt_runner_node_visual",
        output="screen",
        parameters=[
            bt_params_file,
            {
                "motion_profiles_file": motion_profiles_file,
                "bt_tree_file": bt_tree_file,
                "bt_tree_id": bt_tree_id,
                "cartesian_backend_params_file": cartesian_backend_params_file,
                "zmq_server_port": zmq_server_port,
            },
        ],
        condition=IfCondition(use_visual),
    )

    # A small delay so the scene manager and BT runner start after move_group.
    delayed_scene = TimerAction(period=5.0, actions=[scene_manager])
    delayed_bt = TimerAction(period=10.0, actions=[bt_runner_headless, bt_runner_visual])

    _ = (moveit_config_pkg, robot_name, use_rviz)  # kept for bringup usage

    return LaunchDescription(args + [delayed_scene, delayed_bt])
