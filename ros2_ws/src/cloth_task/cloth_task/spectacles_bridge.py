"""spectacles_bridge: the robot bridge for Spectacles teleop (teleop v1 over a WebSocket).

    ros2 launch cloth_task task.launch.py hardware:=real enable_motors:=true run_task:=false \
        spectacles:=true
    ngrok http 127.0.0.1:9100        # by hand, with your own token: the launch never starts it

Serves the lens on `host`:`port` (127.0.0.1:9100) through cloth_task.spectacles_link and runs
one cloth_task.spectacles_session.Session:

  /joint_states                 in: joint1..6, the measurement the session solves from
  /arm_bridge/driver_fault      in: std_msgs/Bool, latched; true = the driver is faulted
  /arm_bridge/teleop            std_srvs/SetBool: called once at start with true, never turned off
  /arm_bridge/teleop_command    out: joint1..6 (rad) + "gripper" (opening 0..1)

Every stop (release, link timeout, replacement lens, driver fault, stale measurement) is the
session ceasing to publish: arm_bridge then holds its last setpoint. Until teleop mode is on and
a joint state has arrived, every lens is refused.
"""

from __future__ import annotations

import contextlib
import sys
import threading

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from cloth_task.core import ARM_JOINTS
from cloth_task.motion import Waiter
from cloth_task.spectacles_link import LensLink
from cloth_task.spectacles_session import Session

TELEOP_SERVICE = "/arm_bridge/teleop"
DRIVER_FAULT_TOPIC = "/arm_bridge/driver_fault"


def _import_rebot(rebot_dir: str):
    sys.path.insert(0, rebot_dir)
    from rebot_b601 import config, kinematics

    return config, kinematics


class SpectaclesBridge(Node):
    def __init__(self):
        super().__init__("spectacles_bridge")
        self.declare_parameter("rebot_dir", "")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 9100)
        self.declare_parameter("tick_hz", 50.0)
        self.declare_parameter("teleop_wait_s", 60.0)  # arm_bridge connects to the arm first
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        C, K = _import_rebot(p("rebot_dir"))
        self.lock = threading.Lock()
        self._pending_q: np.ndarray | None = None
        self.session = Session(K, C.JOINT_LIMITS_RAD, self._send_joints, self._send_gripper)
        self.teleop_wait_s = float(p("teleop_wait_s"))

        cb = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(JointState, "/arm_bridge/teleop_command", 10)
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10, callback_group=cb
        )
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, DRIVER_FAULT_TOPIC, self._on_driver_fault, latched, callback_group=cb
        )
        self.create_timer(1.0 / float(p("tick_hz")), self._tick, callback_group=cb)
        self._teleop_srv = self.create_client(SetBool, TELEOP_SERVICE, callback_group=cb)
        self.link = LensLink(
            self.session,
            self.lock,
            host=str(p("host")),
            port=int(p("port")),
            log=self.get_logger().info,
        )

    # --- session I/O ---

    def _send_joints(self, q: np.ndarray) -> None:
        self._pending_q = np.asarray(q, dtype=float)

    def _send_gripper(self, opening: float) -> None:
        """The session sends joints, then the gripper: one command carries both."""
        if self._pending_q is None:
            return
        msg = JointState(name=[*ARM_JOINTS, "gripper"])
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(v) for v in self._pending_q] + [float(opening)]
        self._pending_q = None
        self._cmd_pub.publish(msg)

    def _on_joints(self, msg: JointState) -> None:
        pos = dict(zip(msg.name, msg.position, strict=False))
        if not all(j in pos for j in ARM_JOINTS):
            return
        with self.lock:
            self.session.on_joint_state([pos[j] for j in ARM_JOINTS])

    def _on_driver_fault(self, msg: Bool) -> None:
        with self.lock:
            self.session.on_driver_fault(bool(msg.data))
        if msg.data:
            self.get_logger().error("driver fault: the lens cannot command the arm")

    def _tick(self) -> None:
        with self.lock:
            self.session.tick()

    # --- start-up ---

    def enable_teleop(self) -> None:
        """Turns arm_bridge's teleop mode on once and leaves it on. Blocks; the executor must
        be spinning in another thread."""
        ok, why = False, f"{TELEOP_SERVICE} not available (is hardware:=real up?)"
        if self._teleop_srv.wait_for_service(timeout_sec=self.teleop_wait_s):
            try:
                res = Waiter(self._teleop_srv.call_async(SetBool.Request(data=True))).wait(5.0)
                ok, why = res.success, res.message
            except TimeoutError:
                why = f"{TELEOP_SERVICE} did not answer"
        with self.lock:
            self.session.on_teleop_mode(ok)
        if ok:
            self.get_logger().info(
                f"teleop mode on: lens on ws://{self.link.host}:{self.link.port}"
            )
        else:
            self.get_logger().error(f"teleop mode not enabled ({why}): every lens is refused")


def main(args=None):
    rclpy.init(args=args)
    node = SpectaclesBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    link = threading.Thread(target=node.link.run, daemon=True)
    try:
        link.start()
        if not node.link.ready.wait(5.0):
            raise SystemExit(f"cannot listen on {node.link.host}:{node.link.port}")
        node.enable_teleop()
        spin.join()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.link.stop()
        link.join(timeout=2.0)
        executor.shutdown()
        with contextlib.suppress(KeyboardInterrupt):
            node.destroy_node()
            rclpy.try_shutdown()
