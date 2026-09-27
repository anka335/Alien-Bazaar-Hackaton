"""Control the patrol from any terminal (D-046).

ros2 run rover_nav patrol_ctl start
ros2 run rover_nav patrol_ctl stop
ros2 run rover_nav patrol_ctl status     # what it's doing now
"""

import sys

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

COMMANDS = ("start", "stop", "status")


def main(argv=None) -> int:
    argv = [a for a in (sys.argv[1:] if argv is None else argv) if not a.startswith("--ros-args")]
    if not argv or argv[0] not in COMMANDS:
        print(f"usage: patrol_ctl {{{'|'.join(COMMANDS)}}}", file=sys.stderr)
        return 2
    rclpy.init()
    node = rclpy.create_node("patrol_ctl")
    try:
        if argv[0] == "status":
            got = {}
            qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(String, "/patrol/phase", lambda m: got.update(p=m.data), qos)
            end = node.get_clock().now().nanoseconds + 3_000_000_000
            while "p" not in got and node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            print(got.get("p", "no patrol running"))
            return 0 if "p" in got else 1
        client = node.create_client(Trigger, f"/patrol/{argv[0]}")
        if not client.wait_for_service(timeout_sec=5.0):
            print("no patrol running (start: ros2 launch rover_nav patrol.launch.py)")
            return 1
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        res = future.result()
        print(res.message if res is not None else "no answer")
        return 0 if res is not None and res.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
