"""fake_rover: stands in for the Leo Rover without hardware (patrol.launch.py fake_rover:=true).

Integrates /leo/cmd_vel into /leo/merged_odom (nav_msgs/Odometry, 50 Hz) and the
leo/odom → leo/base_footprint TF, starting at the pose given by the parameters `x`, `y`,
`yaw_deg` (default: the start (0, 2.5) facing south). Like the rover's firmware it stops when
commands are older than 0.5 s. No physics: what is commanded is exactly what happens.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

INPUT_TIMEOUT_S = 0.5  # the rover firmware's controller.input_timeout


class FakeRover(Node):
    def __init__(self):
        super().__init__("fake_rover")
        self.x = self.declare_parameter("x", 0.0).value
        self.y = self.declare_parameter("y", 2.5).value
        self.yaw = math.radians(self.declare_parameter("yaw_deg", -90.0).value)
        self.v = self.w = 0.0
        self.last_cmd = None
        self.odom_pub = self.create_publisher(Odometry, "/leo/merged_odom", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "/leo/cmd_vel", self._on_cmd, 10)
        self.dt = 0.02
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"fake rover at ({self.x:.2f}, {self.y:.2f}), yaw {math.degrees(self.yaw):.0f} deg"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self.v, self.w = msg.linear.x, msg.angular.z
        self.last_cmd = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self.last_cmd is None or (now - self.last_cmd).nanoseconds * 1e-9 > INPUT_TIMEOUT_S:
            self.v = self.w = 0.0
        self.yaw = math.atan2(
            math.sin(self.yaw + self.w * self.dt), math.cos(self.yaw + self.w * self.dt)
        )
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        stamp = now.to_msg()
        qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, "leo/odom"
        odom.child_frame_id = "leo/base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(odom)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = (
            stamp,
            "leo/odom",
            "leo/base_footprint",
        )
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeRover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
