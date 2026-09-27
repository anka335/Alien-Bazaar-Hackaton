"""The laundry boxes from their ArUco markers (D-047): OAK-D + box_detector.

    ros2 launch rover_nav boxes.launch.py                    # starts the OAK-D too
    ros2 launch rover_nav boxes.launch.py camera:=false      # an OAK-D already running
    ros2 launch rover_nav boxes.launch.py target_box:=dark   # + /boxes/target_distance

Watch the log: it prints each box it sees ("dark (order 3): 0.62 m ahead of the front …") and
markers not assigned to a box yet (dictionary + id) for config/boxes.yaml.
"""

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_nav import stack


def _setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    actions = []
    if arg("camera") == "true":
        with open(stack.share("config", "mounts.yaml")) as f:
            actions += stack.oak_actions(yaml.safe_load(f)["oak_mount"])
    target = arg("target_box")
    actions.append(
        Node(
            package="rover_nav",
            executable="box_detector",
            name="box_detector",
            output="screen",
            parameters=[
                stack.share("config", "boxes.yaml"),
                {"target_box": target} if target else {},
            ],
        )
    )
    return actions


def generate_launch_description():
    args = [
        ("camera", "true", "start the OAK-D (false: use a running one)"),
        (
            "target_box",
            "",
            "dark / colored / light: publish its distance on /boxes/target_distance",
        ),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
        + [OpaqueFunction(function=_setup)]
    )
