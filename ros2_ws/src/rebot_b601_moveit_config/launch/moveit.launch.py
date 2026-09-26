"""MoveIt 2 + ros2_control for the reBot B601-RS.

Starts robot_state_publisher, ros2_control (mock hardware by default), the arm / gripper /
joint-state controllers, move_group and, optionally, RViz. use_ros2_control:=false leaves
/joint_states and the controllers' actions to something else (cloth_task's arm_bridge on the
real arm).

    ros2 launch rebot_b601_moveit_config moveit.launch.py use_rviz:=true
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rebot_b601_moveit_config.description import build_urdf

PKG = "rebot_b601_moveit_config"


def _load_yaml(share, name):
    with open(os.path.join(share, "config", name)) as f:
        return yaml.safe_load(f)


def camera_xyz(mount: dict) -> tuple:
    """camera_mount.yaml xyz relative to gripper_end. With `measured_from: link6` the numbers
    start at the link6 flange instead (same axes): gripper_end is 0.16621 m further along x."""
    x, y, z = (float(v) for v in mount["xyz"])
    if mount.get("measured_from", "gripper_end") == "link6":
        x -= 0.16621  # j_gripper_end in the URDF: link6 → fingertips
    return (x, y, z)


def _setup(context):
    share = get_package_share_directory(PKG)
    hardware = LaunchConfiguration("hardware_plugin").perform(context)
    use_ros2_control = LaunchConfiguration("use_ros2_control").perform(context) == "true"
    with open(LaunchConfiguration("camera_mount").perform(context)) as f:
        mount = yaml.safe_load(f)

    robot_description = {
        "robot_description": build_urdf(
            os.path.join(share, "urdf", "reBot_Lite_RS_with_gripper.urdf"),
            hardware,
            camera_xyz=camera_xyz(mount),
            camera_rpy=tuple(mount["rpy"]),
            camera_parent=mount.get("parent", "gripper_end"),
        )
    }
    with open(os.path.join(share, "config", "rebot_b601.srdf")) as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}
    kinematics = {"robot_description_kinematics": _load_yaml(share, "kinematics.yaml")}
    planning = {
        "robot_description_planning": {
            **_load_yaml(share, "joint_limits.yaml"),
            **_load_yaml(share, "pilz_cartesian_limits.yaml"),
        }
    }
    pipelines = {
        "planning_pipelines": {
            "pipeline_names": ["ompl", "pilz_industrial_motion_planner"],
        },
        "default_planning_pipeline": "ompl",
        "ompl": _load_yaml(share, "ompl_planning.yaml"),
        "pilz_industrial_motion_planner": _load_yaml(
            share, "pilz_industrial_motion_planner_planning.yaml"
        ),
    }
    # MoveGroupSequenceAction is exposed as an extra move_group capability.
    capabilities = {
        "capabilities": pipelines["pilz_industrial_motion_planner"].pop("capabilities", "")
    }
    # the real arm (arm_bridge) has no FollowJointTrajectory gripper controller
    execution = _load_yaml(
        share, "moveit_controllers.yaml" if use_ros2_control else "moveit_controllers_real.yaml"
    )
    psm = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamic_scene": True,
    }

    use_rviz = LaunchConfiguration("use_rviz")
    common = [robot_description, robot_description_semantic, kinematics, planning]

    ros2_control = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[os.path.join(share, "config", "ros2_controllers.yaml")],
            remappings=[("~/robot_description", "/robot_description")],
            output="log",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "arm_controller", "gripper_controller"],
            output="log",
        ),
    ]
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="log",
        ),
        *(ros2_control if use_ros2_control else []),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[*common, pipelines, capabilities, execution, psm],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(share, "config", "moveit.rviz")],
            parameters=[*common],
            condition=IfCondition(use_rviz),
            output="log",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "use_ros2_control",
                default_value="true",
                description="false: /joint_states and controller actions come from elsewhere",
            ),
            DeclareLaunchArgument(
                "camera_mount",
                default_value=os.path.join(
                    get_package_share_directory(PKG), "config", "camera_mount.yaml"
                ),
                description="gripper_end → camera_link transform (yaml: xyz, rpy)",
            ),
            DeclareLaunchArgument(
                "hardware_plugin",
                default_value="mock_components/GenericSystem",
                description="ros2_control hardware plugin; mock by default",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
