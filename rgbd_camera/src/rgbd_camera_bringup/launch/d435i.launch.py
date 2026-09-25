"""Launch a RealSense D435i with aligned depth and IMU streams."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_no = LaunchConfiguration("serial_no")

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": "camera",
            "camera_namespace": "camera",
            "device_type": "d435i",
            "serial_no": serial_no,
            "initial_reset": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "rgb_camera.color_profile": "640,480,30",
            "depth_module.depth_profile": "640,480,30",
            "align_depth.enable": "true",
            "enable_sync": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_no",
                default_value="''",
                description="Optional RealSense serial number prefixed with an underscore",
            ),
            realsense,
        ]
    )
