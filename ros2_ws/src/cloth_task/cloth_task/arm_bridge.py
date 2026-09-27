"""arm_bridge: the real reBot B601-RS for MoveIt, in place of ros2_control + mock hardware.

Wraps the team's `rebot_b601` driver (its 50 Hz control loop, soft limits, tracking-error,
temperature and lost-feedback checks all stay active) and exposes what MoveIt and the task use:

  /joint_states                                 joint1..6 + joint_left/right, 50 Hz
  /arm_controller/follow_joint_trajectory       control_msgs/FollowJointTrajectory, timed as
                                                MoveIt planned it (stretched if too fast)
  /gripper_controller/gripper_cmd               control_msgs/GripperCommand, joint_left in m
  /arm_bridge/teleop                            std_srvs/SetBool: follow a leader arm (on/off)
  /arm_bridge/teleop_command                    sensor_msgs/JointState: joint1..6 (rad) +
                                                "gripper" (opening 0..1), from leader_teleop
                                                or spectacles_bridge
  /arm_bridge/driver_fault                      std_msgs/Bool, latched: false at start, true
                                                once the driver faults (until it reconnects)

Teleop: each command moves the driver's hold setpoint towards it at most `teleop_max_dps` (just
under the motors' own 86°/s limit), every step checked with rebot_b601's pose_is_safe (table,
base); no commands = the arm holds. The driver loop runs at `control_hz` (100 Hz; the driver's
default is 50). MoveIt goals are refused while teleop is on.

enable_motors:=false (the default) only reads the encoders: the arm stays limp and every goal is
rejected. On shutdown with the motors on and no fault it lifts clear, goes through `park_via_deg`
and home, then switches the torque off (at home the arm cannot fall). With a fault it leaves the
motors as they are: support the arm and use the power switch.
"""

from __future__ import annotations

import contextlib
import glob
import math
import os
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from cloth_task.core import ARM_JOINTS
from cloth_task.motion import GRIPPER_ACTION

FINGERS = ("joint_left", "joint_right")
START_TOLERANCE_RAD = 0.05  # first trajectory point vs where the arm is commanded now


def _import_rebot(rebot_dir: str):
    """rebot_b601 from the repo; motorbridge from rebot_b601/.venv, appended after the system
    site-packages so ROS keeps its own numpy."""
    sys.path.insert(0, rebot_dir)
    for site in glob.glob(os.path.join(rebot_dir, ".venv", "lib", "python3*", "site-packages")):
        sys.path.append(site)
    from rebot_b601 import arm as rebot_arm
    from rebot_b601 import config as rebot_config
    from rebot_b601 import kinematics as rebot_kin

    return rebot_arm, rebot_config, rebot_kin


class TimedTrajectory:
    """Duck-types rebot_b601.arm.Trajectory (sample / duration / t0) with MoveIt's timing."""

    def __init__(self, times: np.ndarray, points: np.ndarray, t0: float):
        self.times, self.points, self.t0 = times, points, t0
        self.duration = float(times[-1])

    def sample(self, t: float) -> np.ndarray:
        return np.array(
            [np.interp(t, self.times, self.points[:, j]) for j in range(self.points.shape[1])]
        )


def retime(times: np.ndarray, points: np.ndarray, vmax: np.ndarray) -> tuple[np.ndarray, float]:
    """Stretch `times` so no segment is faster than vmax (rad/s per joint). Returns
    (times, factor); factor 1 = unchanged."""
    dt = np.diff(times)
    dq = np.abs(np.diff(points, axis=0))
    with np.errstate(divide="ignore", invalid="ignore"):
        need = np.where(dt[:, None] > 0, dq / dt[:, None] / vmax, np.where(dq > 0, np.inf, 0))
    factor = float(max(1.0, np.max(need))) if need.size else 1.0
    if not math.isfinite(factor):
        raise ValueError("trajectory has two different points at the same time")
    return times * factor, factor


def unsafety(K, q, z_min: float, base_radius: float = 0.10, base_height: float = 0.22) -> float:
    """How far q violates rebot_b601's pose_is_safe (same points, same limits): metres below
    z_min plus metres inside the base column. 0 = safe. Lets teleop tell "getting out of an
    unsafe pose" from "going deeper"."""
    pts = K.link_points(q)
    violation = max(0.0, z_min - float(pts[3:, 2].min()))  # pose_is_safe skips the first three
    for x, y, z in pts[-6:]:  # the tool points
        r = math.hypot(x, y)
        if r < base_radius and z < base_height:
            violation += min(base_radius - r, base_height - z)
    return violation


def make_ros_arm(rebot, C):
    """rebot_b601's Arm plus execute_timed(), which runs a MoveIt trajectory in the driver's own
    control loop (so its safety checks apply) instead of the driver's min-jerk planner."""
    ArmError = rebot.ArmError

    class RosArm(rebot.Arm):
        def execute_timed(self, times, points, cancel: threading.Event) -> float:
            """Blocks until done; raises ArmError. Returns the time-stretch factor used."""
            self._require()
            for q in points:
                self._check_limits(q, "trajectory")
            self._check_path(points, n_samples=max(60, len(points)))
            vmax = np.radians(C.JOINT_SPEED_DPS) * self.max_speed_scale
            times, factor = retime(times, points, vmax)
            if not self._move_lock.acquire(blocking=False):
                raise ArmError("the arm is busy with another move")
            try:
                with self._lock:
                    start = self._q_cmd.copy()
                    off = float(np.max(np.abs(points[0] - start)))
                    if off > START_TOLERANCE_RAD:
                        raise ArmError(f"trajectory starts {math.degrees(off):.1f}° from the arm")
                    points = points.copy()
                    points[0] = start
                    self._abort = None
                    self._done.clear()
                    self._traj = TimedTrajectory(times, points, time.monotonic())
                deadline = time.monotonic() + float(times[-1]) + 5.0
                while not self._done.wait(0.02):
                    if cancel.is_set():
                        self.stop()
                        raise ArmError("canceled")
                    if time.monotonic() > deadline:
                        self.stop()
                        raise ArmError("move timed out")
                if self._abort:
                    raise ArmError(f"move aborted: {self._abort}")
                return factor
            finally:
                self._move_lock.release()

    return RosArm


class ArmBridge(Node):
    def __init__(self):
        super().__init__("arm_bridge")
        self.declare_parameter("rebot_dir", "")
        self.declare_parameter("enable_motors", False)
        self.declare_parameter("z_min", 0.005)  # rebot path check: lowest link point, m
        self.declare_parameter("park_via_deg", [0.0, 20.0, 15.0, 0.0, 0.0, 0.0])
        self.declare_parameter("park_speed", 0.2)
        # rebot_b601's SimBackend instead of CAN: this bridge's whole code path, no hardware
        self.declare_parameter("driver_sim", False)
        # teleop: every joint up to this, deg/s. The motors' own limit is MOTOR_VLIM (1.5 rad/s =
        # 86°/s): faster commands only lag and trip the driver's tracking-error fault
        self.declare_parameter("teleop_max_dps", 80.0)
        self.declare_parameter("control_hz", 100.0)  # driver loop: setpoints to the motors
        # task runs: starting folded at home, first unfold to park_via with the driver's own
        # planner (its path check applies), the reverse of the parking move. MoveIt's box
        # collision shapes flag the folded home pose, so it would refuse to plan out of it.
        self.declare_parameter("unfold_on_start", False)
        # teleop-only runs (spectacles): rclpy's MultiThreadedExecutor busy-waits while teleop
        # commands stream and published /joint_states up to 150 ms late. Single-threaded blocks
        # on a running trajectory, so only where no MoveIt goal is accepted.
        self.declare_parameter("single_threaded", False)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.rebot, self.C, self.K = _import_rebot(p("rebot_dir"))
        self.C.Z_MIN = float(p("z_min"))  # grasping on the table needs the TCP below 3 cm
        self.enabled = bool(p("enable_motors"))
        self.park_via = list(p("park_via_deg"))
        self.park_speed = float(p("park_speed"))
        vlim_dps = math.degrees(self.C.MOTOR_VLIM_RAD_S)
        self.teleop_max_rad_s = math.radians(min(float(p("teleop_max_dps")), 0.95 * vlim_dps))
        control_hz = float(p("control_hz"))

        self.arm = make_ros_arm(self.rebot, self.C)(hz=control_hz)
        sim = bool(p("driver_sim"))
        if sim:
            # SimBackend moves each joint at 1.2 × JOINT_SPEED_DPS (36°/s for joints 2 and 3):
            # the 80°/s teleop cap outran it and tripped the tracking-error fault within 2 s
            sim_rad_s = 1.2 * np.radians(np.asarray(self.C.JOINT_SPEED_DPS, dtype=float))
            self.teleop_max_rad_s = np.minimum(self.teleop_max_rad_s, 0.9 * sim_rad_s)
        mode = "motors ON" if self.enabled else "READ-ONLY (motors limp)"
        where = "the driver's SIMULATED arm" if sim else f"the arm on {self.C.CAN_CHANNEL}"
        self.get_logger().info(f"connecting to {where}: {mode}")
        st = self.arm.connect(enable=self.enabled, simulate=sim)
        self.get_logger().info(
            f"arm connected: joints {st['joints_deg']} deg, gripper {st['gripper_opening']}, "
            f"max speed scale {self.arm.max_speed_scale}, control loop {control_hz:.0f} Hz"
        )
        folded = abs(st["joints_deg"][1]) < 5 and abs(st["joints_deg"][2]) < 5  # shoulder, elbow
        if self.enabled and p("unfold_on_start") and folded:
            self.get_logger().info(f"folded at home: unfolding to {self.park_via} deg first")
            try:
                self.arm.move_joints(self.park_via, speed_scale=self.park_speed)
            except self.rebot.ArmError as e:
                self.get_logger().error(f"unfold failed ({e}); holding at home")

        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._fault_pub = self.create_publisher(Bool, "/arm_bridge/driver_fault", latched)
        self._fault_sent: bool | None = None
        self._publish_fault()
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / 50.0, self._publish_joints, callback_group=cb)
        self._cancel = threading.Event()
        ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self._execute_trajectory,
            goal_callback=self._accept_if_enabled,
            cancel_callback=self._on_cancel,
            callback_group=cb,
        )
        ActionServer(
            self,
            GripperCommand,
            GRIPPER_ACTION,
            self._execute_gripper,
            goal_callback=self._accept_if_enabled,
            callback_group=cb,
        )
        self._teleop = False
        self._teleop_lock = threading.Lock()
        self._teleop_last_step = time.monotonic()
        self._last_grip: tuple[float, float] | None = None  # (time, opening)
        self._refused_logged = 0.0
        self.create_service(SetBool, "/arm_bridge/teleop", self._on_teleop, callback_group=cb)
        self.create_subscription(
            JointState, "/arm_bridge/teleop_command", self._on_teleop_command, 10, callback_group=cb
        )

    # --- state ---

    def _finger_m(self, grip_pos: float) -> float:
        opening = min(max(grip_pos / math.radians(self.C.GRIPPER_OPEN_DEG), 0.0), 1.0)
        return opening * self.K.FINGER_TRAVEL_M

    def _publish_fault(self) -> None:
        faulted = self.arm._fault is not None
        if faulted != self._fault_sent:
            self._fault_pub.publish(Bool(data=faulted))
            self._fault_sent = faulted

    def _publish_joints(self) -> None:
        self._publish_fault()
        meas = self.arm._meas
        if meas is None:
            return
        f = self._finger_m(meas.grip_pos)
        msg = JointState(name=[*ARM_JOINTS, *FINGERS])
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(v) for v in meas.q] + [f, f]
        msg.velocity = [float(v) for v in meas.dq] + [0.0, 0.0]
        self._js_pub.publish(msg)

    # --- actions ---

    def _accept_if_enabled(self, _goal) -> GoalResponse:
        if self._teleop:
            self.get_logger().warn("goal rejected: teleop is on (stop leader_teleop first)")
            return GoalResponse.REJECT
        if not self.enabled:
            self.get_logger().warn("goal rejected: read-only (launch with enable_motors:=true)")
            return GoalResponse.REJECT
        if self.arm._fault:
            self.get_logger().error(f"goal rejected: arm fault: {self.arm._fault}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal) -> CancelResponse:
        self._cancel.set()
        return CancelResponse.ACCEPT

    def _execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        res = FollowJointTrajectory.Result()
        try:
            idx = [traj.joint_names.index(j) for j in ARM_JOINTS]
        except ValueError:
            res.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            res.error_string = f"need joints {ARM_JOINTS}, got {list(traj.joint_names)}"
            goal_handle.abort()
            return res
        times = np.array(
            [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in traj.points]
        )
        points = np.array([[p.positions[i] for i in idx] for p in traj.points])
        if len(points) < 2:
            times, points = np.array([0.0, max(times[-1], 0.1)]), np.vstack([points[0], points[0]])
        self._cancel.clear()
        try:
            factor = self.arm.execute_timed(times, points, self._cancel)
        except self.rebot.ArmError as e:
            self.get_logger().error(f"trajectory failed: {e}")
            res.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            res.error_string = str(e)
            if self._cancel.is_set():
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return res
        if factor > 1.01:
            self.get_logger().warn(
                f"trajectory ran {factor:.2f}× slower than planned (driver speed cap "
                f"{self.arm.max_speed_scale}); lower execution_speed"
            )
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return res

    def _execute_gripper(self, goal_handle):
        cmd = goal_handle.request.command
        opening = min(max(cmd.position / self.K.FINGER_TRAVEL_M, 0.0), 1.0)
        res = GripperCommand.Result()
        try:
            self.arm.set_gripper(opening, wait=True)
        except self.rebot.ArmError as e:
            self.get_logger().error(f"gripper failed: {e}")
            goal_handle.abort()
            return res
        time.sleep(0.3)  # let the fingers settle on the cloth
        res.position = self._finger_m(self.arm._meas.grip_pos)
        res.reached_goal = abs(res.position - cmd.position) < 0.002
        res.stalled = not res.reached_goal
        self.get_logger().info(
            f"gripper → {cmd.position * 1000:.1f} mm: at {res.position * 1000:.1f} mm"
            + (" (stalled: something between the fingers?)" if res.stalled else "")
        )
        goal_handle.succeed()
        return res

    # --- teleop ---

    def _on_teleop(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if req.data:
            why = (
                "read-only (launch with enable_motors:=true)"
                if not self.enabled
                else f"arm fault: {self.arm._fault}"
                if self.arm._fault
                else "a trajectory is running"
                if self.arm._traj is not None
                else None
            )
            if why:
                res.success, res.message = False, f"teleop refused: {why}"
                self.get_logger().warn(res.message)
                return res
            with self._teleop_lock:
                self._teleop_last_step = time.monotonic()
            self._teleop = True
        else:
            self._teleop = False
            self.arm.stop()  # hold where it is
        res.success, res.message = True, f"teleop {'ON' if req.data else 'OFF'}"
        self.get_logger().info(res.message)
        return res

    def _on_teleop_command(self, msg: JointState) -> None:
        """Apply each leader command as it arrives (no timer in between): move the driver's hold
        setpoint towards it by at most teleop_max_dps × the time since the last command. No
        commands = no change = the arm holds."""
        if not self._teleop:
            return
        pos = dict(zip(msg.name, msg.position, strict=False))
        if not all(j in pos for j in ARM_JOINTS):
            return
        C, K, arm = self.C, self.K, self.arm
        goal = np.clip(
            np.array([pos[j] for j in ARM_JOINTS]),
            C.JOINT_LIMITS_RAD[:, 0],
            C.JOINT_LIMITS_RAD[:, 1],
        )
        with self._teleop_lock:  # callbacks may run in parallel (reentrant group)
            now = time.monotonic()
            dt = min(now - self._teleop_last_step, 0.05)  # after a gap: one small step
            self._teleop_last_step = now
            with arm._lock:
                if arm._traj is not None or arm._fault:
                    return
                hold = arm._hold.copy()
            max_step = self.teleop_max_rad_s * dt
            candidate = hold + np.clip(goal - hold, -max_step, max_step)
            # safe: go. Already unsafe (e.g. started low): only steps that make it less unsafe,
            # so the leader can back out but never go deeper
            bad = unsafety(K, candidate, C.Z_MIN)
            if bad > 0 and bad >= unsafety(K, hold, C.Z_MIN):
                if now - self._refused_logged > 1.0:
                    why = K.pose_is_safe(candidate, z_min=C.Z_MIN)[1]
                    self.get_logger().warn(f"leader pose refused, holding: {why}")
                    self._refused_logged = now
                return
            with arm._lock:
                if arm._traj is None:
                    arm._hold = candidate
            opening = pos.get("gripper")
            if opening is not None and (
                self._last_grip is None
                or (abs(opening - self._last_grip[1]) > 0.01 and now - self._last_grip[0] > 0.02)
            ):
                try:
                    arm.set_gripper(min(max(opening, 0.0), 1.0), wait=False)
                    self._last_grip = (now, opening)
                except self.rebot.ArmError as e:
                    self.get_logger().warn(f"gripper: {e}")

    # --- shutdown ---

    def park_and_disconnect(self) -> None:
        log = self.get_logger()
        arm = self.arm
        self._teleop = False
        if not arm.connected:
            return
        # Ctrl+C reaches this process twice (terminal + ros2 launch): the second one must not cut
        # the shutdown (parking, or closing the bus) half-way. The hardware e-stop stops it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if arm._fault:
            log.error(
                f"arm fault ({arm._fault}): leaving the motors as they are. SUPPORT THE ARM and "
                "switch the power off."
            )
            return
        if not self.enabled:
            arm.disconnect(go_home=False)  # motors were never enabled: nothing can fall
            return
        log.warn("PARKING the arm (Ctrl+C ignored until done; use the hardware e-stop to stop)")
        arm.stop()  # whatever was running: hold first
        try:
            st = arm.status()
            if max(abs(v) for v in st["joints_deg"]) < 3.0:  # already home: no detour
                arm.disconnect(go_home=True, speed_scale=self.park_speed)
                log.info("arm at home, torque off")
                return
            tcp = st["tcp_xyz_m"]
            if tcp[2] < 0.15:
                log.info("parking: lifting clear first")
                arm.move_relative(dz=0.10, linear=True, speed_scale=self.park_speed)
            log.info("parking: via pose, then home, then torque off")
            arm.move_joints(self.park_via, speed_scale=self.park_speed)
            arm.disconnect(go_home=True, speed_scale=self.park_speed)
            log.info("arm parked at home, torque off")
        except self.rebot.ArmError as e:
            log.error(f"parking failed ({e}): motors left ON holding. Support the arm.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmBridge()
    except Exception as e:  # connection problems: say what to check, don't dump a trace
        rclpy.logging.get_logger("arm_bridge").fatal(f"cannot start: {e}")
        rclpy.try_shutdown()
        raise SystemExit(1) from e
    single = bool(node.get_parameter("single_threaded").value)
    executor = SingleThreadedExecutor() if single else MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.park_and_disconnect()
        with contextlib.suppress(KeyboardInterrupt):  # Ctrl+C arrives twice under launch
            node.destroy_node()
            rclpy.try_shutdown()
