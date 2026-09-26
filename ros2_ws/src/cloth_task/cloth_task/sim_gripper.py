"""sim_gripper: GripperCommand for the mock arm, with scripted misses to test the retry loop.

Serves /gripper_controller/gripper_cmd (control_msgs/GripperCommand, position = joint_left in m)
and moves both mock fingers through the ros2_control gripper_controller. Closing counts as a
grasp: the first `misses` closes find nothing (fingers close fully), later ones stop on a cloth
`cloth_thickness_m` thick. arm_bridge serves the same action on the real gripper.
"""

from __future__ import annotations

import contextlib
import threading

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from cloth_task.motion import GRIPPER_ACTION, Waiter

FINGERS = ("joint_left", "joint_right")


class SimGripper(Node):
    def __init__(self):
        super().__init__("sim_gripper")
        self.declare_parameter("misses", 1)
        self.declare_parameter("cloth_thickness_m", 0.004)
        self.declare_parameter("move_s", 0.8)
        self.misses = int(self.get_parameter("misses").value)
        self.thickness = float(self.get_parameter("cloth_thickness_m").value)
        self.move_s = float(self.get_parameter("move_s").value)
        self.position = 0.0
        self.closes = 0
        self._lock = threading.Lock()
        cb = ReentrantCallbackGroup()
        self._fjt = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory",
            callback_group=cb,
        )  # fmt: skip
        ActionServer(self, GripperCommand, GRIPPER_ACTION, self._execute, callback_group=cb)
        self.get_logger().info(f"sim gripper: the first {self.misses} grasp(s) will miss")

    def _execute(self, goal_handle):
        target = max(0.0, float(goal_handle.request.command.position))
        with self._lock:
            closing = target < self.position
            if closing:
                self.closes += 1
                if self.closes > self.misses:
                    target = max(target, self.thickness)  # stopped by the cloth
            stalled = closing and target > goal_handle.request.command.position
            ok = self._move_fingers(target)
            self.position = target
        res = GripperCommand.Result(position=target, stalled=stalled, reached_goal=not stalled)
        if not ok:
            goal_handle.abort()
            return res
        what = "closed" if closing else "opened"
        self.get_logger().info(f"{what} to {target * 1000:.1f} mm (grasp #{self.closes})")
        goal_handle.succeed()
        return res

    def _move_fingers(self, pos: float) -> bool:
        if not self._fjt.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("gripper_controller not available")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(FINGERS)
        sec = int(self.move_s)
        goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[pos, pos],
                time_from_start=Duration(sec=sec, nanosec=int((self.move_s - sec) * 1e9)),
            )
        ]
        gh = Waiter(self._fjt.send_goal_async(goal)).wait(5.0)
        if not gh.accepted:
            return False
        res = Waiter(gh.get_result_async()).wait(self.move_s + 5.0)
        return res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def main(args=None):
    rclpy.init(args=args)
    node = SimGripper()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with contextlib.suppress(KeyboardInterrupt):  # Ctrl+C arrives twice under launch
            node.destroy_node()
            rclpy.try_shutdown()
