"""Drive autonomously on the saved map (D-019): RTAB-Map localization + Nav2 + keepout filter.

    ros2 launch rover_nav navigation.launch.py                # OAK-D on the rover's front
    ros2 launch rover_nav navigation.launch.py nav_camera:=wrist   # arm's D435i
    ros2 launch rover_nav navigation.launch.py nav_camera:=wrist camera:=external camera_tf:=arm

Map and navigate with the same nav_camera: the map remembers what that camera saw.

Goals: RViz "Nav2 Goal". A goal in the keepout half is refused. The keepout mask is
<maps_dir>/keepout.yaml (make_keepout); without it the rover may go anywhere on the map.
Static map: <maps_dir>/<map_name>_nav.yaml (the saved map + obstacles added with add_to_map)
when it exists, served on /map; RTAB-Map then only localizes (its grid moves to
/rtabmap/grid_map). Without that file Nav2 uses RTAB-Map's grid.
Localization needs a first match with the map: if the rover doesn't appear on the map, drive it
a little (teleop) where it was mapped.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_nav import stack

NAV_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "bt_navigator",
]
FILTER_NODES = ["filter_mask_server", "costmap_filter_info_server"]
RTABMAP_GRID = "/rtabmap/grid_map"


def _static_map_actions(static_map: str) -> list:
    """The edited map file on /map (map frame = RTAB-Map's: it was saved from RTAB-Map)."""
    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="static_map_server",
            output="screen",
            parameters=[{"yaml_filename": static_map, "topic_name": "/map", "frame_id": "map"}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_static_map",
            output="screen",
            parameters=[{"autostart": True, "node_names": ["static_map_server"]}],
        ),
    ]


def _nav2_actions(params: str) -> list:
    # cmd_vel chain: controller / behaviors -> cmd_vel_nav -> velocity_smoother -> /leo/cmd_vel
    def node(pkg, exe, remaps=()):
        return Node(
            package=pkg,
            executable=exe,
            name=exe,
            output="screen",
            parameters=[params],
            remappings=list(remaps),
        )

    return [
        node("nav2_controller", "controller_server", [("cmd_vel", "cmd_vel_nav")]),
        node("nav2_planner", "planner_server"),
        node("nav2_behaviors", "behavior_server", [("cmd_vel", "cmd_vel_nav")]),
        node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", stack.CMD_VEL)],
        ),
        node("nav2_bt_navigator", "bt_navigator"),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{"autostart": True, "node_names": NAV_NODES}],
        ),
    ]


def _keepout_actions(params: str, mask: str) -> list:
    if not os.path.isfile(mask):
        return [LogInfo(msg=f"no keepout mask {mask}: the whole map is allowed (make_keepout)")]
    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="filter_mask_server",
            output="screen",
            parameters=[params, {"yaml_filename": mask}],
        ),
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="costmap_filter_info_server",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_filters",
            output="screen",
            parameters=[{"autostart": True, "node_names": FILTER_NODES}],
        ),
    ]


def _setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    maps_dir = os.path.expanduser(arg("maps_dir"))
    database = os.path.join(maps_dir, arg("map_name") + ".db")
    mask = os.path.expanduser(arg("keepout")) or os.path.join(maps_dir, "keepout.yaml")
    static_map = os.path.expanduser(arg("static_map")) or os.path.join(
        maps_dir, arg("map_name") + "_nav.yaml"
    )
    use_file = os.path.isfile(static_map)
    params = stack.share("config", "nav2.yaml")
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
        *stack.rtabmap_actions(True, database, False, topics, RTABMAP_GRID if use_file else "/map"),
        *(
            _static_map_actions(static_map)
            if use_file
            else [LogInfo(msg=f"no {static_map}: Nav2 uses RTAB-Map's grid as the static map")]
        ),
        stack.obstacle_cloud_action(topics),
        *_keepout_actions(params, mask),
        *_nav2_actions(params),
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
            "wrist only. start: run the RealSense driver; external: use cloth_task's",
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
        ("keepout", "", "keepout mask yaml; '' = <maps_dir>/keepout.yaml"),
        (
            "static_map",
            "",
            "map yaml served on /map; '' = <maps_dir>/<map_name>_nav.yaml if it exists",
        ),
        ("use_rviz", "true", "RViz with the map, costmaps and the Nav2 Goal tool"),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
        + [OpaqueFunction(function=_setup)]
    )
