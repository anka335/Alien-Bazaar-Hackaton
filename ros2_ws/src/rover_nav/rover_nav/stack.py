"""Launch building blocks shared by mapping.launch.py and navigation.launch.py (D-015).

Two navigation cameras (nav_camera):
  oak:   OAK-D on the rover's front, fixed: leo/base_link -> oak -> oak_rgb_camera_optical_frame
         (the DepthAI driver's own description, mount from config/mounts.yaml). Default.
  wrist: the arm's RealSense D435i with the arm in `drive`: leo/base_link -> base_link (arm) ->
         ... -> camera_link -> camera_color_optical_frame (arm stack, or static here).
Frames above the camera: map -> leo/odom (RTAB-Map) -> leo/base_footprint (rover's odom_filter)
-> leo/base_link (rover's robot_state_publisher).
"""

import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

ROVER_BASE = "leo/base_footprint"
ROVER_ODOM = "leo/odom"
ROVER_LINK = "leo/base_link"
ARM_BASE = "base_link"  # the arm owns the plain names (D-015)
CMD_VEL = "/leo/cmd_vel"
ODOM_TOPIC = "/leo/merged_odom"

OBSTACLE_CLOUD = "/rover_nav/obstacle_cloud"

# Color, depth aligned to color, color camera info, per navigation camera
TOPICS = {
    # DepthAI driver in the /rover_nav namespace (its robot_description stays off the arm's)
    "oak": {
        "color": "/rover_nav/oak/rgb/image_rect",
        "depth": "/rover_nav/oak/stereo/image_raw",
        "info": "/rover_nav/oak/rgb/camera_info",
    },
    # The RealSense driver as cloth_task starts it (ros2_ws/src/cloth_task/launch/task.launch.py)
    "wrist": {
        "color": "/camera/camera/color/image_raw",
        "depth": "/camera/camera/aligned_depth_to_color/image_raw",
        "info": "/camera/camera/color/camera_info",
    },
}


def share(*parts: str) -> str:
    return os.path.join(get_package_share_directory("rover_nav"), *parts)


def default_maps_dir() -> str:
    return os.environ.get("ROVER_NAV_MAPS", os.path.expanduser("~/rover_nav_maps"))


def check_choice(name: str, value: str, choices: tuple[str, ...]) -> None:
    if value not in choices:
        raise RuntimeError(f"{name} must be one of {'|'.join(choices)} (got {value!r})")


def _static_tf(name: str, parent: str, child: str, xyz, rpy) -> Node:
    x, y, z = (str(float(v)) for v in xyz)
    roll, pitch, yaw = (str(float(v)) for v in rpy)
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x",
            x,
            "--y",
            y,
            "--z",
            z,
            "--roll",
            roll,
            "--pitch",
            pitch,
            "--yaw",
            yaw,
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
    )


def camera_setup(nav_camera: str, camera: str, camera_tf: str, serial: str, profile: str) -> list:
    """Driver + TF of the navigation camera, and the rover -> arm mount (always: the arm rides
    on the rover whichever camera navigates). `camera` and `camera_tf` apply to wrist only."""
    check_choice("nav_camera", nav_camera, tuple(TOPICS))
    with open(share("config", "mounts.yaml")) as f:
        mounts = yaml.safe_load(f)
    actions = [_static_tf("arm_mount_tf", ROVER_LINK, ARM_BASE, **mounts["arm_mount"])]
    if nav_camera == "oak":
        return actions + oak_actions(mounts["oak_mount"])
    return (
        actions
        + wrist_tf_actions(camera_tf, mounts)
        + wrist_camera_actions(camera, serial, profile)
    )


def oak_actions(mount: dict) -> list:
    """The DepthAI driver for the OAK-D, hung under leo/base_link at `mount`."""
    try:
        depthai_launch = os.path.join(
            get_package_share_directory("depthai_ros_driver"), "launch", "camera.launch.py"
        )
    except PackageNotFoundError:
        raise RuntimeError(
            "depthai_ros_driver is not installed: sudo apt install ros-jazzy-depthai-ros"
        ) from None
    (x, y, z), (roll, pitch, yaw) = mount["xyz"], mount["rpy"]
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(depthai_launch),
            launch_arguments={
                "name": "oak",
                "namespace": "rover_nav",
                "camera_model": "OAK-D",
                "parent_frame": ROVER_LINK,
                "cam_pos_x": str(x),
                "cam_pos_y": str(y),
                "cam_pos_z": str(z),
                "cam_roll": str(roll),
                "cam_pitch": str(pitch),
                "cam_yaw": str(yaw),
                "params_file": share("config", "oak.yaml"),
                "rectify_rgb": "true",
                "use_rviz": "false",
            }.items(),
        )
    ]


def wrist_tf_actions(camera_tf: str, mounts: dict) -> list:
    """Arm -> wrist camera only with camera_tf:=static (the arm stack's robot_state_publisher
    publishes it otherwise, and a frame can't have two parents)."""
    check_choice("camera_tf", camera_tf, ("static", "arm"))
    actions = []
    if camera_tf == "static":
        actions += [
            _static_tf(
                "camera_drive_pose_tf", ARM_BASE, "camera_link", **mounts["camera_in_drive_pose"]
            ),
            # REP-103 optical frame, as in rebot_b601_moveit_config's description
            _static_tf(
                "camera_optical_tf",
                "camera_link",
                "camera_color_optical_frame",
                (0, 0, 0),
                (-1.5707963267948966, 0, -1.5707963267948966),
            ),
        ]
    return actions


def wrist_camera_actions(camera: str, serial: str, profile: str) -> list:
    """camera:=start runs the RealSense driver exactly like cloth_task; camera:=external uses
    one that is already running (cloth_task's). Never two drivers on one camera."""
    check_choice("camera", camera, ("start", "external"))
    if camera == "external":
        return []
    try:
        rs_launch = os.path.join(
            get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
        )
    except PackageNotFoundError:
        raise RuntimeError(
            "realsense2_camera is not installed: sudo apt install ros-jazzy-realsense2-camera"
        ) from None
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                "camera_name": "camera",
                "camera_namespace": "camera",
                "serial_no": serial,
                "initial_reset": "true",
                "enable_color": "true",
                "enable_depth": "true",
                "rgb_camera.color_profile": profile,
                "depth_module.depth_profile": profile,
                "align_depth.enable": "true",
                "enable_sync": "true",
                "enable_gyro": "false",
                "enable_accel": "false",
                "publish_tf": "false",  # camera frames come from the arm (or tf_actions)
            }.items(),
        )
    ]


def rtabmap_params(localization: bool) -> dict:
    """RTAB-Map on one RGB-D camera + the rover's odometry from TF, 2D (3 DoF) map."""
    return {
        "frame_id": ROVER_BASE,
        "odom_frame_id": ROVER_ODOM,  # odometry from TF (odom_filter), no odom topic to sync
        "map_frame_id": "map",
        "subscribe_rgbd": True,
        "subscribe_depth": False,
        "subscribe_rgb": False,
        "wait_for_transform": 0.3,
        "publish_tf": True,
        # RTAB-Map's own parameters are strings
        "Reg/Strategy": "0",  # visual registration
        "Reg/Force3DoF": "true",  # the rover drives on a flat floor
        "RGBD/LinearUpdate": "0.05",  # add a map node every 5 cm / 0.05 rad of motion
        "RGBD/AngularUpdate": "0.05",
        "Grid/Sensor": "1",  # occupancy grid from depth
        "Grid/CellSize": "0.05",
        "Grid/RangeMax": "3.0",  # stereo depth gets noisy beyond ~3 m (both cameras)
        "Grid/MaxGroundHeight": "0.05",  # relative to leo/base_footprint (the floor)
        "Grid/MaxObstacleHeight": "0.9",  # above the rover + arm nothing can be hit
        "Grid/NormalsSegmentation": "false",  # plain height thresholds (3 DoF, flat floor)
        "Grid/RayTracing": "true",  # clear free space between the camera and obstacles
        "Mem/IncrementalMemory": "false" if localization else "true",
        "Mem/InitWMWithAllNodes": "true" if localization else "false",
    }


def rtabmap_actions(localization: bool, database: str, new_map: bool, topics: dict) -> list:
    os.makedirs(os.path.dirname(database), exist_ok=True)
    if localization and not os.path.isfile(database):
        raise RuntimeError(f"no map database {database}: run mapping.launch.py first")
    return [
        Node(
            package="rtabmap_sync",
            executable="rgbd_sync",
            name="rgbd_sync",
            parameters=[{"approx_sync": True, "approx_sync_max_interval": 0.05}],
            remappings=[
                ("rgb/image", topics["color"]),
                ("depth/image", topics["depth"]),
                ("rgb/camera_info", topics["info"]),
                ("rgbd_image", "/rover_nav/rgbd_image"),
            ],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[rtabmap_params(localization), {"database_path": database}],
            remappings=[("rgbd_image", "/rover_nav/rgbd_image")],
            arguments=["-d"] if new_map and not localization else [],  # -d: delete the database
        ),
    ]


def obstacle_cloud_action(topics: dict) -> Node:
    """Sparse point cloud from the depth image for Nav2's obstacle layers."""
    return Node(
        package="rtabmap_util",
        executable="point_cloud_xyz",
        name="obstacle_cloud",
        parameters=[{"decimation": 4, "voxel_size": 0.05, "max_depth": 3.0, "approx_sync": False}],
        remappings=[
            ("depth/image", topics["depth"]),
            ("depth/camera_info", topics["info"]),
            ("cloud", OBSTACLE_CLOUD),
        ],
    )


def rviz_action() -> Node:
    return Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("nav2_bringup"), "rviz", "nav2_default_view.rviz"
            ),
        ],
        output="log",
    )
