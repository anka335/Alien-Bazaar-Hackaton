"""preflight_monitor: records what the pre-flight checks from the ROS side, publishes nothing.

    python3 -m cloth_task.preflight_monitor --csv monitor.csv

One CSV row per message, flushed per row, no header, receive time from time.monotonic()
(system-wide on Linux, so comparable across processes):

  joint_states,<t>,<header stamp, s>    /joint_states (sensor_msgs/JointState): the gap monitor
  cmd_vel,<t>,<linear.x>,<angular.z>    /leo/cmd_vel (geometry_msgs/Twist), also while the rover
                                        stand-in is dead, when it cannot log them itself
  odom,<t>                              /leo/merged_odom (nav_msgs/Odometry): rover presence

Only for the pre-flight's private domain (ROS_DOMAIN_ID=77, localhost discovery).
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PreflightMonitor(Node):
    def __init__(self, path: str):
        super().__init__("preflight_monitor")
        self._f = open(path, "w", newline="")  # noqa: SIM115 (closed by close())
        self._w = csv.writer(self._f)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 50)
        self.create_subscription(Twist, "/leo/cmd_vel", self._on_twist, 50)
        self.create_subscription(Odometry, "/leo/merged_odom", self._on_odom, 50)

    def _row(self, kind: str, *values: float) -> None:
        t = time.monotonic()
        self._w.writerow([kind, repr(t), *(repr(float(v)) for v in values)])
        self._f.flush()

    def _on_joints(self, msg: JointState) -> None:
        self._row("joint_states", msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _on_twist(self, msg: Twist) -> None:
        self._row("cmd_vel", msg.linear.x, msg.angular.z)

    def _on_odom(self, _msg: Odometry) -> None:
        self._row("odom")

    def close(self) -> None:
        self._f.close()


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--csv", required=True, help="output CSV")
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = PreflightMonitor(args.csv)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        with contextlib.suppress(KeyboardInterrupt):
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
