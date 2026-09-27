"""patrol node: runs patrol_logic on the rover (D-046). Started by patrol.launch.py.

Nothing moves until `start`:
    ros2 run rover_nav patrol_ctl start
    ros2 run rover_nav patrol_ctl stop      # stop now (the patrol ends)
    ros2 run rover_nav patrol_ctl status

    in   /leo/merged_odom   nav_msgs/Odometry     rover odometry (wheels + IMU)
    out  /leo/cmd_vel       geometry_msgs/Twist   20 Hz while running, zero when stopped
    out  /patrol/phase      std_msgs/String       latched: idle / what it's doing / done / stopped
    out  /patrol/scanning   std_msgs/Bool         true while it pauses during a look-around (the
                                                  moment for a detector to look; not wired yet)
    srv  /patrol/start, /patrol/stop               std_srvs/Trigger
"""

from __future__ import annotations

import math
from dataclasses import fields

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from rover_nav.patrol_logic import PatrolLogic, PatrolParams, Pose2D


class PatrolNode(Node):
    def __init__(self):
        super().__init__("patrol")
        defaults = PatrolParams()
        values = {}
        for f in fields(PatrolParams):
            default = getattr(defaults, f.name)
            if isinstance(default, bool | str | int):
                value = self.declare_parameter(f.name, default).value
            else:
                value = float(self.declare_parameter(f.name, float(default)).value)
            values[f.name] = value
        self.declare_parameter("rate_hz", 20.0)
        self.dry_run = self.declare_parameter("dry_run", False).value
        self.params = PatrolParams(**values)
        self.logic = PatrolLogic(self.params)
        self.running = False
        self.ended = False
        self._last_phase = None
        self._last_scanning = None

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, "/leo/cmd_vel", 10)
        self.phase_pub = self.create_publisher(String, "/patrol/phase", latched)
        self.scan_pub = self.create_publisher(Bool, "/patrol/scanning", 10)
        self.create_subscription(Odometry, "/leo/merged_odom", self._on_odom, 20)
        self.create_service(Trigger, "/patrol/start", self._on_start)
        self.create_service(Trigger, "/patrol/stop", self._on_stop)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._tick)
        self._phase("idle")
        p = self.params
        what = (
            f"{p.size_m} m square"
            if p.shape == "square"
            else (f"{p.size_m} m circle, {p.circle_stops} stops")
        )
        self.get_logger().info(
            f"ready: {what}, {p.laps or 'endless'} lap(s), look around every "
            f"{p.scan_step_deg:.0f} deg for {p.dwell_s} s at each stop, {p.speed} m/s"
            + (" [DRY RUN]" if self.dry_run else "")
            + ". Waiting for: ros2 run rover_nav patrol_ctl start"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.logic.odom(Pose2D(p.x, p.y, yaw), self._now())

    def _on_start(self, _req, res):
        if self.ended:
            res.success, res.message = False, "patrol already ended: restart the launch"
        elif self.running:
            res.success, res.message = False, "already running"
        else:
            self.running = True
            res.success, res.message = True, "started"
            self.get_logger().info("start")
        return res

    def _on_stop(self, _req, res):
        self.running, self.ended = False, True
        self.stop_rover()
        self._phase("stopped")
        self.get_logger().warn("stopped by request")
        res.success, res.message = True, "stopped"
        return res

    def _tick(self) -> None:
        if not self.running:
            return
        v, w = self.logic.step(self._now())
        self._send(v, w)
        self._phase(self.logic.message)
        scanning = self.logic.scanning
        if scanning != self._last_scanning:
            self._last_scanning = scanning
            self.scan_pub.publish(Bool(data=scanning))
        if self.logic.done:
            self.running, self.ended = False, True
            self.stop_rover()

    def _phase(self, text: str) -> None:
        if text != self._last_phase:
            self._last_phase = text
            self.phase_pub.publish(String(data=text))
            if text not in ("idle",):
                self.get_logger().info(text)

    def _send(self, v: float, w: float) -> None:
        if self.dry_run:
            return
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def stop_rover(self) -> None:
        for _ in range(3):
            self._send(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_rover()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
