"""leader_teleop: drive the real arm with the StarArm102 / reBot Arm 102 leader ("Spark") and
save checkpoints (named poses) without moving the robot by hand.

    ros2 launch cloth_task task.launch.py hardware:=real enable_motors:=true run_task:=false
    ros2 run cloth_task leader_teleop               # leader on /dev/rebot_leader, or --port
    ros2 run cloth_task leader_teleop --calibrate   # once: set the leader's zero (LeRobot's way)

At the prompt: a name + Enter saves the ARM's current pose into poses.yaml (box_view, bin_1, …),
Enter alone prints where the arm is, q quits (the arm holds where it is).

The leader is read at --rate Hz and mapped with LeRobot's conventions (cloth_task.leader); the
arm_bridge follows at a limited speed, refuses unsafe poses and holds if the stream stops.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from cloth_task.core import ARM_JOINTS, ConfigError, save_pose
from cloth_task.leader import BAUDRATE, JOINTS, SERVO_IDS, to_follower
from cloth_task.motion import BASE_FRAME, TCP_FRAME, Waiter
from cloth_task.pose_tools import default_poses_file

DEFAULT_PORT = "/dev/rebot_leader" if os.path.exists("/dev/rebot_leader") else "/dev/ttyUSB0"
BIG_JUMP_DEG = 20.0


def _import_servo_sdk():
    """motorbridge-smart-servo lives in ros2_ws/.pydeps (scripts/setup_deps.sh)."""
    ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(default_poses_file()))))
    sys.path.append(os.path.join(ws, ".pydeps"))
    try:
        import motorbridge_smart_servo
    except ImportError as e:
        raise SystemExit(f"motorbridge-smart-servo missing: run {ws}/scripts/setup_deps.sh") from e
    return motorbridge_smart_servo


class Leader:
    def __init__(self, port: str):
        sdk = _import_servo_sdk()
        try:
            self.bus = sdk.FashionStarServo(port, baudrate=BAUDRATE)
        except Exception as e:
            hint = (
                " (no access: run sudo ros2_ws/scripts/setup_hardware.sh, it adds a udev rule)"
                if "ermission" in str(e)
                else ""
            )
            raise SystemExit(f"cannot open the leader on {port}: {e}{hint}") from e
        missing = [f"{n} (id {i})" for n, i in SERVO_IDS.items() if not self.bus.ping(i)]
        if missing:
            self.bus.close()
            raise SystemExit(f"leader servos not answering: {', '.join(missing)} (power? cable?)")
        for i in SERVO_IDS.values():  # LeRobot's configure(): torque off, fresh turn counter
            self.bus.unlock(i)
            time.sleep(0.01)
        for i in SERVO_IDS.values():
            self.bus.reset_multi_turn(i)

    def calibrate(self) -> None:
        for i in SERVO_IDS.values():
            self.bus.unlock(i)
            time.sleep(0.01)
            self.bus.set_origin_point(i)

    def read(self) -> dict[str, float]:
        result = self.bus.sync_monitor(list(SERVO_IDS.values()))
        id_to_name = {i: n for n, i in SERVO_IDS.items()}
        raw = {}
        for servo_id, monitor in result.items():
            if monitor is None:
                raise RuntimeError(f"leader servo {id_to_name[servo_id]} (id {servo_id}) silent")
            raw[id_to_name[servo_id]] = monitor.angle_deg
        return raw

    def close(self) -> None:
        self.bus.close()


class Teleop(Node):
    def __init__(self, leader: Leader, rate_hz: float, gripper_open_deg: float, flip=frozenset()):
        super().__init__("leader_teleop")
        self.leader, self.rate_hz, self.gripper_open_deg = leader, rate_hz, gripper_open_deg
        self.flip = frozenset(flip)
        self.cmd_pub = self.create_publisher(JointState, "/arm_bridge/teleop_command", 10)
        self.teleop_srv = self.create_client(SetBool, "/arm_bridge/teleop")
        self.joints: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.running = threading.Event()
        self.read_errors = 0

    def _on_joints(self, msg: JointState) -> None:
        self.joints.update(zip(msg.name, msg.position, strict=False))

    def arm_deg(self) -> list[float] | None:
        if not all(j in self.joints for j in ARM_JOINTS):
            return None
        return [math.degrees(self.joints[j]) for j in ARM_JOINTS]

    def tcp(self):
        try:
            t = self.tf_buffer.lookup_transform(BASE_FRAME, TCP_FRAME, Time()).transform.translation
        except TransformException:
            return None
        return t.x, t.y, t.z

    def leader_target(self) -> tuple[list[float], float]:
        return to_follower(self.leader.read(), self.gripper_open_deg, self.flip)

    def set_teleop(self, on: bool) -> str:
        if not self.teleop_srv.wait_for_service(timeout_sec=5.0):
            raise SystemExit("/arm_bridge/teleop not available: is the hardware:=real launch up?")
        res = Waiter(self.teleop_srv.call_async(SetBool.Request(data=on))).wait(5.0)
        if not res.success:
            raise SystemExit(res.message)
        return res.message

    def stream(self) -> None:
        period = 1.0 / self.rate_hz
        while self.running.is_set():
            t0 = time.monotonic()
            try:
                q_deg, opening = self.leader_target()
                self.read_errors = 0
            except Exception as e:  # the bridge holds once the stream stops
                self.read_errors += 1
                if self.read_errors == 1:
                    self.get_logger().error(f"leader read failed (arm holds): {e}")
                time.sleep(period)
                continue
            msg = JointState(name=[*ARM_JOINTS, "gripper"])
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = [math.radians(v) for v in q_deg] + [opening]
            self.cmd_pub.publish(msg)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))


def _fmt(q) -> str:
    return "[" + ", ".join(f"{v:7.1f}" for v in q) + "]"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="leader_teleop", description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--calibrate", action="store_true", help="set the leader's zero, then exit")
    # a sync read of the 7 servos takes ~4.5 ms at 1 Mbaud
    ap.add_argument("--rate", type=float, default=200.0, help="leader read / command rate, Hz")
    ap.add_argument("--file", default=None, help="poses file (default: cloth_task poses.yaml)")
    ap.add_argument("--gripper-open-deg", type=float, default=240.0, help="driver GRIPPER_OPEN_DEG")
    ap.add_argument(
        "--flip",
        action="append",
        default=[],
        choices=JOINTS[:6],
        help="reverse one arm joint's direction (repeatable) if it moves the wrong way",
    )
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])
    path = args.file or default_poses_file()

    leader = Leader(args.port)
    if args.calibrate:
        input(
            "Leader calibration: move the leader to its ZERO pose = the same shape as the robot "
            "folded at home (all joints 0), gripper CLOSED. Press Enter to set it... "
        )
        leader.calibrate()
        leader.close()
        print("leader zero set (stored in the servos)")
        return 0

    rclpy.init()
    node = Teleop(leader, args.rate, args.gripper_open_deg, frozenset(args.flip))
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    streamer = None
    try:
        deadline = time.monotonic() + 10.0
        while node.arm_deg() is None:
            if time.monotonic() > deadline:
                raise SystemExit("no /joint_states: start the hardware:=real launch first")
            time.sleep(0.1)
        q_leader, opening = node.leader_target()
        q_arm = node.arm_deg()
        diff = [abs(a - b) for a, b in zip(q_leader, q_arm, strict=True)]
        print(f"leader {_fmt(q_leader)} deg, gripper {opening:.2f}")
        print(f"arm    {_fmt(q_arm)} deg")
        if max(diff) > BIG_JUMP_DEG:
            worst = ARM_JOINTS[diff.index(max(diff))]
            print(
                f"!! the leader is {max(diff):.0f}° away from the arm ({worst}): the arm will move "
                "there at teleop speed. Better: put the leader in the arm's shape first."
            )
        if input("Enter = engage teleop, q = quit: ").strip().lower() == "q":
            return 0
        node.running.set()
        streamer = threading.Thread(target=node.stream, daemon=True)
        streamer.start()
        time.sleep(0.1)
        print(node.set_teleop(True), "- move the leader. Hand on the e-stop.")
        print("name + Enter = save the arm's pose, Enter = show pose, q = quit")
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if line.lower() in ("q", "quit", "exit"):
                break
            q = node.arm_deg()
            if not line:
                tcp = node.tcp()
                where = f", TCP ({tcp[0]:.3f}, {tcp[1]:.3f}, {tcp[2]:.3f}) m" if tcp else ""
                print(f"arm {_fmt(q)} deg{where}")
                continue
            try:
                save_pose(path, line, q)
            except ConfigError as e:
                print(f"not saved: {e}")
                continue
            print(f"saved {line!r} = {_fmt(q)} deg → {path}")
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.running.clear()
        if streamer is not None:
            streamer.join(timeout=1.0)
            try:
                print(node.set_teleop(False), "- the arm holds where it is")
            except SystemExit as e:
                print(e)
        leader.close()
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
