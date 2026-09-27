"""The hard-coded patrol (D-046): a 1 m square or circle, looking around at every stop.

    ros2 launch rover_nav patrol.launch.py                    # rover on, standing at the start
    ros2 launch rover_nav patrol.launch.py shape:=circle      # or size_m:=0.8, laps:=2, ...
    ros2 launch rover_nav patrol.launch.py fake_rover:=true   # without the rover
    ros2 run rover_nav patrol_ctl start                       # other terminal: go (stop: any time)

dry_run:=true computes everything and sends no drive commands. Any parameter of
config/patrol.yaml can be overridden on the command line.
"""

from dataclasses import fields

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_nav import stack
from rover_nav.patrol_logic import PatrolParams

DEFAULTS = PatrolParams()


def _value(name: str, text: str):
    default = getattr(DEFAULTS, name)
    if isinstance(default, bool):
        return text.lower() == "true"
    if isinstance(default, str):
        return text
    if isinstance(default, int):
        return int(text)
    return float(text)


def _setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    overrides = {f.name: _value(f.name, arg(f.name)) for f in fields(PatrolParams) if arg(f.name)}
    overrides["dry_run"] = arg("dry_run") == "true"
    actions = []
    if arg("fake_rover") == "true":
        actions.append(Node(package="rover_nav", executable="fake_rover", output="screen"))
    actions.append(
        Node(
            package="rover_nav",
            executable="patrol",
            name="patrol",
            output="screen",
            parameters=[stack.share("config", "patrol.yaml"), overrides],
        )
    )
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "fake_rover", default_value="false", description="no rover: a fake one"
        ),
        DeclareLaunchArgument(
            "dry_run", default_value="false", description="send no drive commands"
        ),
    ] + [
        DeclareLaunchArgument(f.name, default_value="", description="override config/patrol.yaml")
        for f in fields(PatrolParams)
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
