"""MoveIt from Python through the move_group action (no moveit_py on Jazzy apt).

Blocking calls, meant for a worker thread while a MultiThreadedExecutor spins the node.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from cloth_task.core import ARM_JOINTS, Quat, Vec3

PLANNING_GROUP = "arm"
FINGER_JOINT = "joint_left"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"
ARM_CONTROLLER_ACTION = "/arm_controller/follow_joint_trajectory"
BASE_FRAME = "base_link"
TCP_FRAME = "gripper_end"
STILL_WINDOW_S = 0.25
RESULT_TIMEOUT_S = 180.0  # no move takes this long even at 0.1 speed; move_group gone → error

ERROR_NAMES = {
    v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)
}


def error_name(code: int) -> str:
    return ERROR_NAMES.get(code, str(code))


class MotionError(RuntimeError):
    pass


class Waiter:
    """Block a worker thread on an rclpy future that the executor completes."""

    def __init__(self, future):
        self.future = future
        self._done = threading.Event()
        future.add_done_callback(lambda _: self._done.set())

    def wait(self, timeout_s: float | None = None):
        if not self._done.wait(timeout_s):
            raise TimeoutError("future timed out")
        return self.future.result()


def down_constraint(
    q: Quat, max_tilt_deg: float, max_roll_deg: float = 180.0
) -> OrientationConstraint:
    """gripper_end orientation within max_tilt of `q` and within max_roll about its own approach
    axis (X). XYZ Euler tolerances are about the target frame's axes: X is the roll, Y and Z
    the tilt; X = 180° leaves the roll free. The Y/Z tolerances form a box, and the approach
    axis ends up acos(cos y · cos z) from `q`'s, so each gets max_tilt/√2: then the total tilt
    stays within max_tilt (both at 20° would allow 28°)."""
    tilt = math.radians(max_tilt_deg) / math.sqrt(2)
    oc = OrientationConstraint()
    oc.header.frame_id = BASE_FRAME
    oc.link_name = TCP_FRAME
    oc.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    oc.absolute_x_axis_tolerance = max(math.radians(max_roll_deg), 1e-3)
    oc.absolute_y_axis_tolerance = max(tilt, 1e-3)
    oc.absolute_z_axis_tolerance = max(tilt, 1e-3)
    oc.weight = 1.0
    return oc


def _position_constraint(xyz: Vec3, tolerance_m: float) -> PositionConstraint:
    """gripper_end inside a sphere of `tolerance_m` around `xyz` (base_link)."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    pose.orientation.w = 1.0
    region = BoundingVolume(
        primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance_m])],
        primitive_poses=[pose],
    )
    pc = PositionConstraint(link_name=TCP_FRAME, constraint_region=region, weight=1.0)
    pc.header.frame_id = BASE_FRAME
    return pc


class GripperClient:
    """control_msgs/GripperCommand on GRIPPER_ACTION (arm_bridge on the real arm, sim_gripper in
    simulation). position = joint_left in m, 0 = closed."""

    def __init__(self, node: Node):
        self._action = ActionClient(
            node, GripperCommand, GRIPPER_ACTION, callback_group=ReentrantCallbackGroup()
        )

    def wait_ready(self, timeout_s: float = 30.0) -> None:
        if not self._action.wait_for_server(timeout_sec=timeout_s):
            raise MotionError(f"gripper action {GRIPPER_ACTION} not available")

    def command(self, position_m: float, effort: float, timeout_s: float = 10.0):
        """Blocks; returns the GripperCommand result (position, effort, stalled, reached_goal)."""
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = float(position_m), float(effort)
        gh = Waiter(self._action.send_goal_async(goal)).wait(timeout_s)
        if not gh.accepted:
            raise MotionError("gripper rejected the goal")
        res = Waiter(gh.get_result_async()).wait(timeout_s)
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            raise MotionError(f"gripper goal ended with status {res.status}")
        return res.result


class MoveItClient:
    def __init__(self, node: Node):
        self.node = node
        cb = ReentrantCallbackGroup()
        self._action = ActionClient(node, MoveGroup, "move_action", callback_group=cb)
        # the arm controller (ros2_control or arm_bridge) may come up after move_group
        self._controller = ActionClient(
            node, FollowJointTrajectory, ARM_CONTROLLER_ACTION, callback_group=cb
        )
        self._exec_event_pub = node.create_publisher(String, "/trajectory_execution_event", 10)
        node.create_subscription(
            JointState, "/joint_states", self._on_joints, 50, callback_group=cb
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.joints: deque[tuple[float, dict[str, float]]] = deque(maxlen=200)
        self._lock = threading.Lock()
        self._goal_handle = None
        self._halted = threading.Event()

    def _on_joints(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):  # malformed: skip it, don't kill the callback
            return
        q = dict(zip(msg.name, msg.position, strict=True))
        if all(j in q for j in ARM_JOINTS):
            self.joints.append((time.monotonic(), q))

    def wait_ready(self, timeout_s: float = 60.0) -> None:
        if not self._action.wait_for_server(timeout_sec=timeout_s):
            raise MotionError("move_group action server not available")
        if not self._controller.wait_for_server(timeout_sec=timeout_s):
            raise MotionError(f"arm controller {ARM_CONTROLLER_ACTION} not available")
        deadline = time.monotonic() + timeout_s
        while not self.joints or self.tcp_pose() is None:
            if time.monotonic() > deadline:
                raise MotionError("no /joint_states or TF for the arm")
            time.sleep(0.1)

    def current_joints(self) -> tuple[float, ...]:
        q = self.joints[-1][1]
        return tuple(q[j] for j in ARM_JOINTS)

    def finger_position(self) -> float | None:
        """joint_left, m: 0 = closed. None if /joint_states carries no gripper joint."""
        return self.joints[-1][1].get(FINGER_JOINT) if self.joints else None

    def tcp_pose(self, frame: str = TCP_FRAME) -> tuple[Vec3, Quat] | None:
        try:
            t = self.tf_buffer.lookup_transform(BASE_FRAME, frame, Time()).transform
        except TransformException:
            return None
        p, r = t.translation, t.rotation
        return (p.x, p.y, p.z), (r.x, r.y, r.z, r.w)

    # --- halting ---

    def halt(self) -> None:
        """Same as MoveGroupInterface::stop(), plus a cancel of our own goal. Every later move
        is refused until clear_halt()."""
        self._halted.set()
        self._exec_event_pub.publish(String(data="stop"))
        with self._lock:
            gh = self._goal_handle
        if gh is not None:
            gh.cancel_goal_async()

    def clear_halt(self) -> None:
        self._halted.clear()

    @property
    def halted(self) -> bool:
        return self._halted.is_set()

    # --- moves; each returns the MoveIt error code (1 = SUCCESS) ---

    def _request(self, velocity_scale: float, planning_time_s: float) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.allowed_planning_time = planning_time_s
        # The Python equivalent of move_group.setMaxVelocityScalingFactor(speed).
        req.max_velocity_scaling_factor = velocity_scale
        req.max_acceleration_scaling_factor = velocity_scale
        req.start_state.is_diff = True
        return req

    def move_joints(
        self, q: tuple[float, ...], velocity_scale: float, planning_time_s: float = 5.0
    ) -> int:
        """Pilz PTP: a straight line in joint space, predictable for known poses."""
        req = self._request(velocity_scale, planning_time_s)
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"
        req.num_planning_attempts = 1
        req.goal_constraints.append(
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=name,
                        position=pos,
                        tolerance_above=1e-3,
                        tolerance_below=1e-3,
                        weight=1.0,
                    )
                    for name, pos in zip(ARM_JOINTS, q, strict=True)
                ]
            )
        )
        return self._execute(req)

    def move_pose(
        self,
        xyz: Vec3,
        q: Quat,
        velocity_scale: float,
        *,
        max_tilt_deg: float,
        max_roll_deg: float,
        position_tolerance_m: float,
        keep_down_on_path: bool,
        planning_time_s: float,
        planning_attempts: int,
    ) -> int:
        """OMPL (collision-aware, uses the Octomap when there is one) to a gripper_end pose,
        orientation within max_tilt_deg / max_roll_deg of `q`; optionally the same tilt limit
        (roll free) along the path."""
        req = self._request(velocity_scale, planning_time_s)
        req.pipeline_id = "ompl"
        req.planner_id = "RRTConnectkConfigDefault"
        req.num_planning_attempts = planning_attempts
        req.goal_constraints.append(
            Constraints(
                position_constraints=[_position_constraint(xyz, position_tolerance_m)],
                orientation_constraints=[down_constraint(q, max_tilt_deg, max_roll_deg)],
            )
        )
        if keep_down_on_path:
            req.path_constraints = Constraints(
                orientation_constraints=[down_constraint(q, max_tilt_deg)]
            )
        return self._execute(req)

    def move_linear(
        self,
        xyz: Vec3,
        velocity_scale: float,
        planning_time_s: float = 5.0,
        orientation: Quat | None = None,
    ) -> int:
        """Pilz LIN: the TCP on a straight line to `xyz` (MoveIt's computeCartesianPath
        equivalent: no sweep on the way down/up), orientation interpolated from the current one
        to `orientation` (default: keep the current one)."""
        pose = self.tcp_pose()
        if pose is None:
            raise MotionError("no TF for the TCP")
        q = pose[1] if orientation is None else orientation
        req = self._request(velocity_scale, planning_time_s)
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "LIN"
        req.num_planning_attempts = 1
        oc = down_constraint(q, 0.5, 0.5)  # the goal orientation (LIN uses it exactly)
        req.goal_constraints.append(
            Constraints(
                position_constraints=[_position_constraint(xyz, 1e-4)],
                orientation_constraints=[oc],
            )
        )
        return self._execute(req)

    def _execute(self, req: MotionPlanRequest) -> int:
        if self.halted:
            return MoveItErrorCodes.PREEMPTED
        goal = MoveGroup.Goal(request=req)
        goal.planning_options.plan_only = False
        gh = Waiter(self._action.send_goal_async(goal)).wait(10.0)
        if not gh.accepted:
            raise MotionError("move_group rejected the goal")
        with self._lock:
            self._goal_handle = gh
        if self.halted:  # halt() came while the goal was being accepted
            gh.cancel_goal_async()
        try:
            res = Waiter(gh.get_result_async()).wait(RESULT_TIMEOUT_S)
        except TimeoutError as e:
            gh.cancel_goal_async()
            raise MotionError(
                f"no result from move_group in {RESULT_TIMEOUT_S:.0f} s (did it die?)"
            ) from e
        finally:
            with self._lock:
                self._goal_handle = None
        if res.status == GoalStatus.STATUS_CANCELED:
            return MoveItErrorCodes.PREEMPTED
        return res.result.error_code.val

    def wait_still(self, still_velocity: float, timeout_s: float) -> float | None:
        """Wait until no arm joint moved faster than still_velocity for STILL_WINDOW_S.
        Returns the time (monotonic) the arm became still, or None on timeout. Uses positions,
        not velocities: mock hardware reports zero velocity."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            samples = list(self.joints)
            if samples and samples[-1][0] - samples[0][0] >= STILL_WINDOW_S:
                t_end = samples[-1][0]
                window = [s for s in samples if s[0] >= t_end - STILL_WINDOW_S]
                q0, q1 = window[0][1], window[-1][1]
                dt = window[-1][0] - window[0][0]
                if dt > 0 and max(abs(q1[j] - q0[j]) / dt for j in ARM_JOINTS) < still_velocity:
                    return window[0][0]
            time.sleep(0.02)
        return None
