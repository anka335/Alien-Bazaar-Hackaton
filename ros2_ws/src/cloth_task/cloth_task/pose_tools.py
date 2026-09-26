"""Named-pose tools.

    ros2 run cloth_task record_pose box_view            # current /joint_states → poses.yaml
    ros2 run cloth_task record_pose box_view --deg 0 87.7 72.1 -74.4 0 0   # typed in
    ros2 run cloth_task go_to_pose ready box_view --speed 0.3              # move through them
    ros2 run cloth_task go_to_pose --list

The default file is cloth_task's config/poses.yaml. With `colcon build --symlink-install` the
installed file is a symlink, so recording writes straight into the source tree (commit it).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from cloth_task.core import ARM_JOINTS, ConfigError, check_speed, load_poses, save_pose
from cloth_task.motion import BASE_FRAME, TCP_FRAME, MoveItClient, error_name


def default_poses_file() -> str:
    path = os.path.join(get_package_share_directory("cloth_task"), "config", "poses.yaml")
    return os.path.realpath(path)  # follow the symlink-install link into src/


class _Spinner:
    """Spin a node in a background thread; stop() ends the spin before the node is destroyed."""

    def __init__(self, node: Node):
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.executor.shutdown()
        self.thread.join(timeout=2.0)


def _read_joints(node: Node, timeout_s: float) -> tuple[float, ...]:
    got: list[dict[str, float]] = []

    def on_joints(msg: JointState) -> None:
        q = dict(zip(msg.name, msg.position, strict=True))
        if all(j in q for j in ARM_JOINTS):
            got.append(q)

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    deadline = time.monotonic() + timeout_s
    while not got:
        if time.monotonic() > deadline:
            raise TimeoutError(
                f"no /joint_states with {', '.join(ARM_JOINTS)} in {timeout_s:.0f} s "
                "(is the robot / MoveIt launch running, same ROS_DOMAIN_ID?)"
            )
        time.sleep(0.05)
    return tuple(got[-1][j] for j in ARM_JOINTS)


def _tcp_xyz(tf_buffer: Buffer) -> tuple[float, float, float] | None:
    try:
        t = tf_buffer.lookup_transform(BASE_FRAME, TCP_FRAME, Time()).transform.translation
    except TransformException:
        return None
    return t.x, t.y, t.z


def record_main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="record_pose", description=__doc__.split("\n\n")[0])
    ap.add_argument("name", help="pose name, e.g. box_view")
    ap.add_argument(
        "--deg",
        nargs=len(ARM_JOINTS),
        type=float,
        metavar="J",
        help="joint1..joint6 in degrees instead of reading /joint_states "
        "(e.g. from `python -m rebot_b601 state` on the real arm)",
    )
    ap.add_argument("--file", default=None, help="poses file (default: cloth_task poses.yaml)")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])
    path = args.file or default_poses_file()

    tcp = None
    if args.deg is not None:
        q_deg = list(args.deg)
    else:
        rclpy.init()
        node = Node("record_pose")
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)  # only to print where the TCP is
        spinner = _Spinner(node)
        try:
            q_deg = [math.degrees(v) for v in _read_joints(node, args.timeout)]
            deadline = time.monotonic() + 2.0
            while tcp is None and time.monotonic() < deadline:
                tcp = _tcp_xyz(tf_buffer)
                time.sleep(0.05)
        except TimeoutError as e:
            print(f"error: {e}", file=sys.stderr)
            return 1
        finally:
            spinner.stop()
            node.destroy_node()
            rclpy.try_shutdown()

    try:
        save_pose(path, args.name, q_deg)
    except ConfigError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(f"saved {args.name!r} = [{', '.join(f'{v:.2f}' for v in q_deg)}] deg → {path}")
    if tcp is not None:
        x, y, z = tcp
        print(f"TCP (gripper_end) at ({x:.3f}, {y:.3f}, {z:.3f}) m in base_link")
    if "/install/" in path:
        print("warning: this is the installed copy; rebuild with --symlink-install or pass --file")
    return 0


def go_main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="go_to_pose", description="Move through named poses.")
    ap.add_argument("names", nargs="*", help="poses in order, e.g. ready box_view")
    ap.add_argument("--speed", type=float, default=0.3, help="velocity scaling 0.1 … 1.0")
    ap.add_argument("--file", default=None, help="poses file (default: cloth_task poses.yaml)")
    ap.add_argument("--list", action="store_true", help="print the known poses and exit")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv or sys.argv)[1:])
    path = args.file or default_poses_file()
    poses = load_poses(path)

    if args.list or not args.names:
        print(path)
        for name, q in poses.items():
            print(f"  {name}: [{', '.join(f'{math.degrees(v):.1f}' for v in q)}]")
        return 0
    try:
        speed = check_speed(args.speed)
        missing = [n for n in args.names if n not in poses]
        if missing:
            raise ConfigError(f"unknown pose(s) {missing}; known: {sorted(poses)}")
    except ConfigError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    rclpy.init()
    node = Node("go_to_pose")
    motion = MoveItClient(node)
    spinner = _Spinner(node)
    try:
        motion.wait_ready(timeout_s=30.0)
        for name in args.names:
            print(f"→ {name}")
            code = motion.move_joints(poses[name], speed)
            if code != MoveItErrorCodes.SUCCESS:
                print(f"error: {name}: move_group {error_name(code)}", file=sys.stderr)
                return 1
        (x, y, z), _ = motion.tcp_pose()
        print(f"done; TCP at ({x:.3f}, {y:.3f}, {z:.3f}) m")
        return 0
    except KeyboardInterrupt:
        motion.halt()
        return 130
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    finally:
        spinner.stop()
        node.destroy_node()
        rclpy.try_shutdown()
