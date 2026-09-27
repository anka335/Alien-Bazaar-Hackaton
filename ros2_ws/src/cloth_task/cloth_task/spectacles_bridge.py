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

With `rover` true (the mobile base, left clutch):

  /leo/cmd_vel                  out: geometry_msgs/Twist, linear.x = vx, angular.z = wz
  /leo/merged_odom              in: nav_msgs/Odometry, only its arrival (rover presence) is used

Every stop (release, link timeout, replacement lens, driver fault, stale measurement) is the
session ceasing to publish: arm_bridge then holds its last setpoint. Until teleop mode is on and
a joint state has arrived, every lens is refused. The base is commanded on each tick while
driving, and gets one zero Twist when driving ends and one at shutdown (rclpy's signal handlers
are off so the context is still up for it). The node refuses to start if a base limit
(`base_max_vx`, `base_max_reverse`, `base_max_wz`) is not positive or exceeds protocol v1's.
"""

from __future__ import annotations

import contextlib
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from cloth_task.core import ARM_JOINTS
from cloth_task.motion import Waiter
from cloth_task.spectacles_link import LensLink
from cloth_task.spectacles_session import Session, check_base_limits

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
        self.declare_parameter("rover", False)
        self.declare_parameter("cmd_vel_topic", "/leo/cmd_vel")
        self.declare_parameter("odom_topic", "/leo/merged_odom")
        self.declare_parameter("base_max_vx", 0.20)
        self.declare_parameter("base_max_reverse", 0.10)
        self.declare_parameter("base_max_wz", 0.6)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        limits = [float(p(n)) for n in ("base_max_vx", "base_max_reverse", "base_max_wz")]
        check_base_limits(*limits)  # before anything is created
        rover = bool(p("rover"))
        C, K = _import_rebot(p("rebot_dir"))
        self.lock = threading.Lock()
        self._pending_q: np.ndarray | None = None
        self._base_pub = (
            self.create_publisher(Twist, str(p("cmd_vel_topic")), 10) if rover else None
        )
        self.session = Session(
            K,
            C.JOINT_LIMITS_RAD,
            self._send_joints,
            self._send_gripper,
            send_base=self._send_base,
            rover=rover,
            max_vx=limits[0],
            max_reverse=limits[1],
            max_wz=limits[2],
        )
        self.teleop_wait_s = float(p("teleop_wait_s"))

        cb = ReentrantCallbackGroup()
        if rover:
            self.create_subscription(
                Odometry, str(p("odom_topic")), self._on_odometry, 10, callback_group=cb
            )
            self.get_logger().info(
                f"rover on: {p('cmd_vel_topic')} (limits +{limits[0]} / -{limits[1]} m/s, "
                f"{limits[2]} rad/s), presence from {p('odom_topic')}"
            )
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

    def _send_base(self, vx: float, wz: float) -> None:
        """Called under the lock, from the executor or the WebSocket thread."""
        if self._base_pub is None:
            return
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._base_pub.publish(msg)

    def _on_odometry(self, _msg: Odometry) -> None:
        with self.lock:
            self.session.on_odometry()

    def shutdown_base(self) -> None:
        """One zero Twist (with `rover` true); the base never drives again."""
        with self.lock:
            self.session.on_shutdown()

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


def install_stop_signals() -> None:
    """SIGINT and SIGTERM raise KeyboardInterrupt in the main thread. Set explicitly: a node
    started from a background job (`cmd &` in a script, then ros2 launch) inherits SIGINT as
    ignored, and Python keeps an inherited SIG_IGN."""
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)


def main(args=None):
    # Own SIGINT/SIGTERM handling: rclpy's would shut the context down first, and then the zero
    # Twist at shutdown could not be published. Both raise KeyboardInterrupt here instead.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_stop_signals()
    try:
        node = SpectaclesBridge()
    except ValueError as e:  # a base limit out of range
        rclpy.logging.get_logger("spectacles_bridge").fatal(f"cannot start: {e}")
        rclpy.try_shutdown()
        raise SystemExit(1) from e
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    link = threading.Thread(target=node.link.run, daemon=True)
    try:
        link.start()
        if not node.link.ready.wait(5.0):
            raise SystemExit(f"cannot listen on {node.link.host}:{node.link.port}")
        node.enable_teleop()
        # Not spin.join(): a KeyboardInterrupt inside Thread.join() leaves the spin thread
        # half-joined on Python 3.12, and the process then aborts at exit
        while spin.is_alive():
            time.sleep(0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C arrives twice under launch: the second must not cut the zero Twist short
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.shutdown_base()
        node.link.stop()
        link.join(timeout=2.0)
        executor.shutdown()
        spin.join(timeout=2.0)  # a daemon thread left in rclpy at exit aborts the process
        with contextlib.suppress(KeyboardInterrupt):
            node.destroy_node()
            rclpy.try_shutdown()
