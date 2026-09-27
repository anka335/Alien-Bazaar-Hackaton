"""rover_standin: plays the Leo Rover for tests of the robot bridge's mobile base, no hardware.

    ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 run cloth_task rover_standin \
        --ros-args -p csv_path:=/tmp/rover_standin.csv

Never started by a launch file, and never run on the rover's domain: it publishes on /leo/*.

  /leo/cmd_vel       in: geometry_msgs/Twist, linear.x (m/s) and angular.z (rad/s) are used
  /leo/merged_odom   out: nav_msgs/Odometry at 100 Hz, leo/odom -> leo/base_footprint

The odometry integrates a differential-drive model and stops CMD_TIMEOUT_S after the last Twist,
like the firmware. Every received Twist is appended to the CSV at `csv_path`, one row per Twist
and no header: receive time from time.monotonic() (system-wide on Linux, so comparable across
processes), linear.x, angular.z. Killing the process is how "rover absent" is produced.
"""

from __future__ import annotations

import contextlib
import csv
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# The Leo Rover firmware stops the wheels 0.5 s after the last /leo/cmd_vel
CMD_TIMEOUT_S = 0.5


class DiffDrive:
    """Differential-drive pose on the floor from (vx, wz) commands; times in s on one clock."""

    def __init__(self, t0: float, timeout_s: float = CMD_TIMEOUT_S):
        self.timeout_s = timeout_s
        self.x = self.y = self.yaw = 0.0  # m, m, rad in the odometry frame
        self.vx = self.wz = 0.0  # the velocity now
        self._t = t0
        self._cmd = (0.0, 0.0)
        self._t_cmd = -math.inf

    def command(self, vx: float, wz: float, t: float) -> None:
        """A Twist arrived at t: it replaces the previous command from t on."""
        self.step(t)
        self._cmd = (float(vx), float(wz))
        self._t_cmd = t
        self.vx, self.wz = self._cmd

    def step(self, t: float) -> None:
        """Advances to t; the command runs until CMD_TIMEOUT_S after it arrived."""
        until = self._t_cmd + self.timeout_s
        dt = min(t, until) - self._t
        if dt > 0.0:
            vx, wz = self._cmd
            mid = self.yaw + wz * dt / 2.0
            self.x += vx * math.cos(mid) * dt
            self.y += vx * math.sin(mid) * dt
            self.yaw = math.remainder(self.yaw + wz * dt, math.tau)
        self._t = max(self._t, t)
        self.vx, self.wz = self._cmd if t < until else (0.0, 0.0)


class CmdLog:
    """Appends (receive time, vx, wz) rows to a CSV, flushed per row."""

    def __init__(self, path: str):
        self._f = open(path, "a", newline="")  # noqa: SIM115 (closed by close())
        self._w = csv.writer(self._f)

    def write(self, t: float, vx: float, wz: float) -> None:
        self._w.writerow([repr(float(t)), repr(float(vx)), repr(float(wz))])
        self._f.flush()

    def close(self) -> None:
        self._f.close()


class RoverStandin(Node):
    def __init__(self):
        super().__init__("rover_standin")
        self.declare_parameter("cmd_vel_topic", "/leo/cmd_vel")
        self.declare_parameter("odom_topic", "/leo/merged_odom")
        self.declare_parameter("csv_path", "rover_standin.csv")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("odom_frame", "leo/odom")
        self.declare_parameter("base_frame", "leo/base_footprint")
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.odom_frame, self.base_frame = str(p("odom_frame")), str(p("base_frame"))
        self.drive = DiffDrive(t0=time.monotonic())
        self.log = CmdLog(str(p("csv_path")))
        self._pub = self.create_publisher(Odometry, str(p("odom_topic")), 10)
        self.create_subscription(Twist, str(p("cmd_vel_topic")), self._on_twist, 10)
        self.create_timer(1.0 / float(p("rate_hz")), self._publish)
        self.get_logger().info(
            f"rover stand-in: {p('cmd_vel_topic')} -> {p('odom_topic')}, "
            f"Twists logged to {p('csv_path')}"
        )

    def _on_twist(self, msg: Twist) -> None:
        t = time.monotonic()
        self.log.write(t, msg.linear.x, msg.angular.z)
        self.drive.command(msg.linear.x, msg.angular.z, t)

    def _publish(self) -> None:
        d = self.drive
        d.step(time.monotonic())
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = d.x
        msg.pose.pose.position.y = d.y
        msg.pose.pose.orientation.z = math.sin(d.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(d.yaw / 2.0)
        msg.twist.twist.linear.x = d.vx
        msg.twist.twist.angular.z = d.wz
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoverStandin()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.log.close()
        with contextlib.suppress(KeyboardInterrupt):
            node.destroy_node()
            rclpy.try_shutdown()
