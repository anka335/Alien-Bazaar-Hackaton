"""Only the OAK-D navigation camera (D-020), to start it before mapping / navigation.

    ros2 launch rover_nav camera.launch.py                       # wait for "Camera ready!"
    ros2 launch rover_nav navigation.launch.py camera:=external  # other terminal
    ros2 launch rover_nav mapping.launch.py camera:=external

Started together with the rest (RTAB-Map loading its database, Nav2, RViz), the OAK-D sometimes
connects and says "Camera ready!" but never streams, on the USB 2 connection of the test laptop.
Started alone it streams at once, and mapping / navigation can then be restarted without
touching it.
"""

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from rover_nav import stack


def _setup(context):
    return stack.oak_actions(stack.oak_mount())


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
