"""Map the room once (D-019): RTAB-Map on one RGB-D camera + the rover's odometry.

    ros2 launch rover_nav mapping.launch.py                   # OAK-D on the rover's front
    ros2 launch rover_nav mapping.launch.py nav_camera:=wrist # arm's D435i, static camera TF
    ros2 launch rover_nav mapping.launch.py nav_camera:=wrist camera:=external camera_tf:=arm
    # other terminal, drive slowly around the whole room:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/leo/cmd_vel

The database is saved when this stops (Ctrl+C): <maps_dir>/<map_name>.db. Running it again
extends the same map; new_map:=true starts from scratch (deletes the database).
Then save the 2D map for the keepout mask (while this still runs):
    ros2 run nav2_map_server map_saver_cli -f ~/rover_nav_maps/room \
        --ros-args -p map_subscribe_transient_local:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from rover_nav import stack


def _setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    database = os.path.join(os.path.expanduser(arg("maps_dir")), arg("map_name") + ".db")
    stack.check_choice("nav_camera", arg("nav_camera"), tuple(stack.TOPICS))
    topics = stack.TOPICS[arg("nav_camera")]
    actions = [
        *stack.camera_setup(
            arg("nav_camera"),
            arg("camera"),
            arg("camera_tf"),
            arg("camera_serial"),
            arg("camera_profile"),
        ),
        *stack.rtabmap_actions(False, database, arg("new_map") == "true", topics),
    ]
    if arg("use_rviz") == "true":
        actions.append(stack.rviz_action())
    return actions


def generate_launch_description():
    args = [
        ("nav_camera", "oak", "oak: OAK-D on the rover's front; wrist: the arm's RealSense D435i"),
        (
            "camera",
            "start",
            "start: run the camera driver; external: use a running one (camera.launch.py)",
        ),
        (
            "camera_tf",
            "static",
            "wrist only. static: pose from config/mounts.yaml; arm: from the arm stack",
        ),
        (
            "camera_serial",
            "''",
            "wrist only. RealSense serial, leading underscore; '' = the only one",
        ),
        (
            "camera_profile",
            "640,480,15",
            "wrist only. RealSense color/depth profile (as cloth_task)",
        ),
        ("maps_dir", stack.default_maps_dir(), "where the map database and map files live"),
        ("map_name", "room", "database <map_name>.db in maps_dir"),
        ("new_map", "false", "true: delete the database and map from scratch"),
        ("use_rviz", "true", "RViz with the map"),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
        + [OpaqueFunction(function=_setup)]
    )
