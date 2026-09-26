"""task_supervisor_node: the state machine that runs the cloth pick-and-place task.

    INIT ─► GO_TO_VIEW ─┬─► DETECT ─► APPROACH ─► TRIAL_GRASP ─► VALIDATE ─► HOLDING ─► PLACE
         └► SEARCH ─────┘     ▲                                  │ empty             │
                              ├─── back to the look pose ◄───────┘ (max_attempts)    │
                              └─── next cloth (cycles) ◄─────────────────────────────┘
    no cloth left (after the first) ─► DONE; grasp:=false stops at AT_APPROACH; failure ─► ERROR

* INIT: checks the launch arguments (execution_speed, place_target, mode, start_pose), loads
  the task config and the named poses, adds the box to the planning scene.
* GO_TO_VIEW: through the `ready` pose (unfolds the arm from the folded home pose) to the
  recorded start pose over the box (Pilz PTP).
* SEARCH / HALTING (phase 1, kept for later): sweep over the table, halt on the first True on
  /cloth_detection_status.
* DETECT: with the arm still, enable the detector (/cloth_detector/enable) and take the median
  of `detect.samples` /cloth_target_pose messages from images taken after it stopped,
  transformed to base_link with tf2 at each image's time.
* APPROACH (phase 2): gripper_end to `approach.height_m` above the cloth, approach axis within
  `approach.max_tilt_deg` of straight down (goal and, if the start pose allows, path
  constraint). OMPL plans around everything in the planning scene (box, Octomap).
* TRIAL_GRASP (phase 3): open, straight down (Pilz LIN) to `grasp.depth_m` below the cloth
  surface (not below `grasp.floor_z_m`), close.
* VALIDATE (phase 4): straight up `grasp.lift_m`, read the finger position: at least
  `grasp.empty_below_m` = cloth in the gripper → HOLDING. Less = missed: open, back to the look
  pose, detect again, up to `grasp.max_attempts`.
* PLACE (phase 5): to the place target (place_target:=1|2|3, or :=color: by the detected
  class), no downward path constraint; open, wait, then the next cycle from the look pose.
  `cycles` = how many cloths (0 = until no cloth is detected); then `ready` and DONE.

Status: /task_supervisor/phase (std_msgs/String, latched). Markers: /task_markers.
"""

from __future__ import annotations

import contextlib
import json
import math
import threading
import time
from collections import deque

import rclpy
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped with tf2
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

from cloth_task.core import (
    BoxConfig,
    ConfigError,
    Phase,
    TargetRejected,
    Vec3,
    approach_heights,
    approach_pose,
    check_mode,
    check_place_target,
    check_speed,
    down_orientation,
    grasp_point,
    holding_cloth,
    lifted,
    load_poses,
    load_task_config,
    majority,
    median_point,
    place_target_id,
    resolve_pose,
    slerp,
    sweep_sequence,
    tilt_from_down_deg,
    turn_about_approach,
    vertical_keep_roll,
)
from cloth_task.motion import (
    BASE_FRAME,
    GripperClient,
    MotionError,
    MoveItClient,
    Waiter,
    error_name,
)

DESCENT_TURNS_DEG = (0.0, 90.0, -90.0)  # finger roll tried for the descent (box walls only)
LIN_SLOWDOWNS = (1.0, 0.5, 0.25)  # straight moves: speed factors tried on a Pilz limit refusal
APPROACH_RETRY_CODES = {  # planning failures: a lower approach point may still be reachable
    MoveItErrorCodes.PLANNING_FAILED,
    MoveItErrorCodes.NO_IK_SOLUTION,
    MoveItErrorCodes.TIMED_OUT,
    MoveItErrorCodes.INVALID_MOTION_PLAN,
    MoveItErrorCodes.GOAL_IN_COLLISION,
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED,
    MoveItErrorCodes.FAILURE,  # OMPL's answer when it finds no goal state at all
}


class TaskError(RuntimeError):
    pass


class DetectionError(TaskError):
    """A bad look (samples disagree, too few, no color class when sorting by color): the
    attempt counts as failed and is retried from the look pose."""


class NoCloth(TaskError):
    """Nothing detected from the look pose: an error on the first cloth, "done" after it."""


class TaskSupervisor(Node):
    def __init__(self):
        super().__init__("task_supervisor_node")
        any_number = ParameterDescriptor(dynamic_typing=True)  # accept 1 as well as 1.0
        self.declare_parameter("execution_speed", 0.3, any_number)
        self.declare_parameter("place_target", 1, any_number)
        self.declare_parameter("task_config", "")
        self.declare_parameter("poses_file", "")
        self.declare_parameter("mode", "fixed_view")
        self.declare_parameter("start_pose", "box_view")
        self.declare_parameter("grasp", True)  # false: stop above the cloth (bring-up)
        self.declare_parameter("cycles", 1)  # cloths to pick and place; 0 = until none is seen

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.speed = check_speed(p("execution_speed"))
        self.mode = check_mode(p("mode"))
        self.cfg = load_task_config(p("task_config"))
        self.place_choice = check_place_target(self.cfg, p("place_target"))
        self.cycles = int(p("cycles"))
        if self.cycles < 0:
            raise ConfigError(f"cycles must be >= 0, got {self.cycles}")
        self.poses_file = p("poses_file")
        self.poses = load_poses(self.poses_file)
        usable = (
            set(self.cfg.place.color_targets.values())
            if self.place_choice == "color"
            else {self.place_choice}
        )
        for tid in usable:  # named-pose targets must exist before anything moves
            target = self.cfg.place_targets[tid]
            if isinstance(target, str):
                resolve_pose(self.poses, target, self.poses_file)
        self.ready_q = (
            resolve_pose(self.poses, self.cfg.ready_pose, self.poses_file)
            if self.cfg.ready_pose
            else None
        )
        self.do_grasp = bool(p("grasp"))
        self.start_pose = p("start_pose")
        self.start_q = (
            resolve_pose(self.poses, self.start_pose, self.poses_file)
            if self.mode == "fixed_view"
            else None
        )

        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._phase_pub = self.create_publisher(String, "/task_supervisor/phase", latched)
        # one JSON snapshot of where the task stands, for status_web (and anyone else)
        self._status_pub = self.create_publisher(String, "/task_supervisor/status", latched)
        self._marker_pub = self.create_publisher(MarkerArray, "/task_markers", 10)
        self._scene = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene", callback_group=cb
        )
        self.motion = MoveItClient(self)
        self.gripper = GripperClient(self)
        self._detector = self.create_client(SetBool, "/cloth_detector/enable", callback_group=cb)
        self.create_subscription(
            Bool, "/cloth_detection_status", self._on_detection, 10, callback_group=cb
        )
        self.create_subscription(
            PoseStamped, "/cloth_target_pose", self._on_target_pose, 10, callback_group=cb
        )
        self.create_subscription(
            String, "/cloth_detector/color", self._on_color, 10, callback_group=cb
        )

        self._lock = threading.Lock()
        self.stopping = threading.Event()  # Ctrl+C: the task stops at its next step
        self._poses: deque[PoseStamped] = deque(maxlen=50)
        self._colors: deque[tuple[float, str]] = deque(maxlen=50)  # (received, class)
        self._t_detect: float | None = None
        self._tcp_detect: Vec3 | None = None

        self.status: dict = {
            "mode": self.mode,
            "place_target": self.place_choice,
            "cycles": self.cycles,
            "execution_speed": self.speed,
            "max_attempts": self.cfg.grasp.max_attempts,
            "placed": 0,
            "attempt": 0,
            "cloth": None,
            "color": None,
            "target": None,
            "error": None,
            "started": time.time(),
        }
        self.phase = Phase.INIT
        self._set_phase(Phase.INIT)
        self.get_logger().info(
            f"mode={self.mode} start_pose={self.start_pose if self.start_q else '-'} "
            f"grasp={self.do_grasp} cycles={self.cycles or 'until empty'} "
            f"execution_speed={self.speed} place_target={self.place_choice}"
        )

    # --- subscriptions ---

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self._poses.append(msg)

    def _on_color(self, msg: String) -> None:
        if msg.data.strip():
            self._colors.append((time.monotonic(), msg.data.split()[0]))  # "colored 0.93"

    def _on_detection(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            if self.phase is not Phase.SEARCH:
                return
            self._t_detect = time.monotonic()
            tcp = self.motion.tcp_pose()
            self._tcp_detect = tcp[0] if tcp else None
            self._set_phase(Phase.HALTING)
        self.get_logger().info("cloth detected: halting the arm")
        self.motion.halt()

    # --- run ---

    def run(self) -> None:
        log = self.get_logger()
        try:
            log.info("waiting for move_group, /joint_states and TF")
            self.motion.wait_ready()
            if self.do_grasp:
                self.gripper.wait_ready()
            self._add_scene()
            if self.mode == "fixed_view":
                self._go_to_view()
            else:
                self._phase1_search()
            look_q = self.motion.current_joints()  # retries and next cloths look from here
            placed = 0
            while self.cycles == 0 or placed < self.cycles:
                if placed:
                    self._set_phase(Phase.GO_TO_VIEW)
                    log.info(f"cloth {placed + 1}: back to the look pose")
                    self._move_joints(look_q, "look pose")
                try:
                    color = self._pick(look_q)
                except NoCloth as e:
                    if not placed:
                        raise
                    log.info(f"no more cloth ({e}); {placed} placed")
                    break
                if color is None and not self.do_grasp:
                    return  # grasp:=false: stays above the cloth
                self._phase5_place(color)
                placed += 1
                self._update_status(placed=placed)
            self._go_to_ready()
            self._set_phase(Phase.DONE)
            log.info(f"DONE: {placed} cloth(s) picked and placed")
        except (TaskError, MotionError, TargetRejected) as e:
            log.error(f"{self.phase}: {e}")
            self._update_status(error=f"{self.phase}: {e}")
            self._set_phase(Phase.ERROR)
        except Exception as e:  # a bug must be reported, not kill the node silently
            log.error(f"{self.phase}: unexpected {e!r}")
            self._update_status(error=f"{self.phase}: unexpected {e!r}")
            self._set_phase(Phase.ERROR)

    def _settle(self, what: str) -> None:
        """The bridge reports a move done when its commanded trajectory ends; the real arm may
        still be catching up (the driver tolerates 12° of tracking error). Wait for it."""
        if self.motion.wait_still(self.cfg.still_velocity, self.cfg.still_timeout_s) is None:
            raise TaskError(f"{what}: the arm did not settle within {self.cfg.still_timeout_s} s")

    def _move_joints(self, q, what: str, velocity_scale: float | None = None) -> int:
        if self.stopping.is_set():
            raise TaskError("stopped (Ctrl+C)")
        code = self.motion.move_joints(
            q, velocity_scale or self.speed, planning_time_s=self.cfg.planning_time_s
        )
        if code != MoveItErrorCodes.SUCCESS and not self.motion.halted:
            raise TaskError(f"{what}: move_group {error_name(code)}")
        return code

    def _go_to_ready(self) -> None:
        if self.ready_q is not None:
            self.get_logger().info(f"moving to the {self.cfg.ready_pose!r} pose")
            self._move_joints(self.ready_q, f"pose {self.cfg.ready_pose!r}")

    def _go_to_view(self) -> None:
        self._set_phase(Phase.GO_TO_VIEW)
        self._go_to_ready()
        self.get_logger().info(f"moving to the start pose {self.start_pose!r}")
        self._move_joints(self.start_q, f"pose {self.start_pose!r}")

    # --- phase 1 (search mode) ---

    def _phase1_search(self) -> None:
        s = self.cfg.search
        self._go_to_ready()
        scale = self.speed * s.sweep_speed_factor
        self._set_phase(Phase.SEARCH)
        self.get_logger().info(
            f"phase 1: sweeping {len(s.waypoints)} look poses at velocity scale {scale:.2f}"
            f" ({'until found' if s.max_sweeps == 0 else f'max {s.max_sweeps} sweeps'})"
        )
        self._set_detector(True)  # the real detector only looks while switched on
        try:
            for n, q in sweep_sequence(s.waypoints, s.max_sweeps):
                if self.motion.halted:
                    break
                self._move_joints(q, f"sweep {n}", velocity_scale=scale)
                if self.motion.halted:
                    break
            else:
                raise TaskError(f"no cloth found after {s.max_sweeps} sweeps")
        finally:
            self._set_detector(False)
        t_still = self.motion.wait_still(self.cfg.still_velocity, self.cfg.still_timeout_s)
        if t_still is None:
            raise TaskError("arm did not come to rest after the stop")
        tcp_now = self.motion.tcp_pose()
        tcp_drift = (
            math.dist(tcp_now[0], self._tcp_detect) if tcp_now and self._tcp_detect else math.nan
        )
        self.get_logger().info(
            f"phase 1 done: arm halted {max(0.0, t_still - self._t_detect) * 1000:.0f} ms after "
            f"detection, TCP travel {tcp_drift * 1000:.1f} mm"
        )
        self.motion.clear_halt()

    # --- detection ---

    def _pick(self, look_q) -> str | None:
        """Detect → approach → grasp → lift → check, up to grasp.max_attempts. Returns the
        detected color class once the cloth is held (None with grasp:=false, stopped above)."""
        log = self.get_logger()
        attempts = self.cfg.grasp.max_attempts
        for attempt in range(1, attempts + 1):
            self._update_status(attempt=attempt)
            if attempt > 1:
                self._set_phase(Phase.GO_TO_VIEW)
                log.info(f"attempt {attempt}/{attempts}: back to the look pose")
                self._move_joints(look_q, "look pose")
            try:
                cloth, color = self._detect()
                if self.place_choice == "color" and color not in self.cfg.place.color_targets:
                    raise DetectionError(  # decide the bin before grasping, not after the lift
                        f"place_target:=color but no usable class ({color!r})"
                    )
            except DetectionError as e:
                log.warn(f"attempt {attempt}/{attempts}: bad look ({e}), looking again")
                continue
            self._phase2_approach(cloth)
            if not self.do_grasp:
                log.info("grasp:=false: stopping above the cloth")
                return None
            if self._grasp_and_check(cloth, attempt):
                self._set_phase(Phase.HOLDING)
                log.info(f"cloth held after {attempt} attempt(s)")
                return color or ""
        raise TaskError(f"no cloth in the gripper after {attempts} attempts (see above)")

    def _detect(self) -> tuple[Vec3, str | None]:
        """Median cloth point in base_link + the majority color class of the same samples."""
        self._set_phase(Phase.DETECT)
        d = self.cfg.detect
        if self.motion.wait_still(self.cfg.still_velocity, self.cfg.still_timeout_s) is None:
            raise TaskError("arm is not still, cannot take a detection")
        cutoff = self.get_clock().now()  # only images taken from here on
        cutoff_mono = time.monotonic()
        self.get_logger().info(f"waiting for {d.samples} /cloth_target_pose samples")
        self._set_detector(True)
        try:
            deadline = time.monotonic() + d.timeout_s
            while True:
                fresh = [m for m in list(self._poses) if Time.from_msg(m.header.stamp) >= cutoff]
                if len(fresh) >= d.samples:
                    break
                if time.monotonic() > deadline:
                    msg = (
                        f"got {len(fresh)}/{d.samples} cloth detections in {d.timeout_s:.0f} s "
                        "with the arm still: is the cloth in view of the look pose?"
                    )
                    raise NoCloth(msg) if not fresh else DetectionError(msg)
                time.sleep(0.05)
        finally:
            self._set_detector(False)
        points = []
        for msg in fresh[-d.samples :]:
            try:
                p = self.motion.tf_buffer.transform(
                    msg, BASE_FRAME, timeout=Duration(seconds=0.5)
                ).pose.position
            except TransformException as e:
                raise TaskError(
                    f"cannot transform {msg.header.frame_id} → {BASE_FRAME}: {e}"
                ) from e
            points.append((p.x, p.y, p.z))
        try:
            cloth = median_point(points, d.max_spread_m)
        except ValueError as e:
            raise DetectionError(str(e)) from e
        color = majority([c for t, c in list(self._colors) if t >= cutoff_mono])
        self.get_logger().info(
            "cloth at ({:.3f}, {:.3f}, {:.3f}) m in {} (median of {}, from {}), class {}".format(
                *cloth, BASE_FRAME, len(points), fresh[-1].header.frame_id, color or "?"
            )
        )
        self._update_status(cloth=[round(v, 3) for v in cloth], color=color)
        return cloth, color

    # --- phase 2 ---

    def _set_detector(self, on: bool) -> None:
        if not self._detector.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("/cloth_detector/enable not available; using whatever arrives")
            return
        Waiter(self._detector.call_async(SetBool.Request(data=on))).wait(5.0)

    def _phase2_approach(self, cloth: Vec3) -> Vec3:
        a = self.cfg.approach
        self._set_phase(Phase.APPROACH)
        start = self.motion.tcp_pose()
        start_tilt = tilt_from_down_deg(start[1]) if start else math.inf
        t0 = time.monotonic()
        tried = []
        # far out or on a high pile, 10 cm above the cloth may be out of reach: lower the
        # approach point, then allow more tilt (lowest tilt first: better for the grasp)
        tilts = sorted({a.max_tilt_deg, a.max_tilt_fallback_deg})
        attempts = [(tilt, height) for tilt in tilts for height in approach_heights(a)]
        for tilt, height in attempts:
            # the path constraint makes planning much harder: only on the first, normal try
            first = (tilt, height) == attempts[0]
            keep_down = a.keep_down_on_path and first and start_tilt <= tilt
            xyz, q = approach_pose(cloth, a, height)
            self._publish_markers(cloth, xyz)
            self.get_logger().info(
                "phase 2: approach to ({:.3f}, {:.3f}, {:.3f}) m, {:.0f} cm above the cloth, "
                "tilt ≤ {:.0f}°{}".format(
                    *xyz, height * 100, tilt, ", path constrained" if keep_down else ""
                )
            )
            code = self.motion.move_pose(
                xyz,
                q,
                self.speed,
                max_tilt_deg=tilt,
                max_roll_deg=a.max_roll_deg,
                position_tolerance_m=a.position_tolerance_m,
                keep_down_on_path=keep_down,
                planning_time_s=a.planning_time_s,
                planning_attempts=a.planning_attempts,
            )
            if code == MoveItErrorCodes.SUCCESS:
                break
            tried.append(f"{height * 100:.0f} cm/{tilt:.0f}°: {error_name(code)}")
            if code not in APPROACH_RETRY_CODES:  # not a planning failure: don't try lower
                break
        else:
            code = None
        if code != MoveItErrorCodes.SUCCESS:
            raise TaskError(
                f"{'; '.join(tried)} (out of reach? "
                "keep the cloth within ~0.40 m of the base and below ~0.15 m)"
            )
        self._settle("approach")
        tcp = self.motion.tcp_pose()
        err = math.dist(tcp[0], xyz)
        self._set_phase(Phase.AT_APPROACH)
        self.get_logger().info(
            f"phase 2 done in {time.monotonic() - t0:.1f} s: TCP {err * 1000:.1f} mm from the "
            f"approach point, tilt {tilt_from_down_deg(tcp[1]):.1f}°, joints (deg) "
            + ", ".join(f"{math.degrees(v):.1f}" for v in self.motion.current_joints())
        )
        return xyz

    # --- phases 3 and 4 ---

    def _gripper(self, position_m: float, what: str):
        res = self.gripper.command(position_m, self.cfg.grasp.effort)
        self.get_logger().info(
            f"gripper {what}: fingers at {res.position * 1000:.1f} mm"
            + (" (stalled)" if res.stalled else "")
        )
        return res

    def _move_linear(self, xyz: Vec3, orientation=None) -> int:
        """Pilz LIN; slower again when Pilz refuses on a joint velocity/acceleration limit
        (PLANNING_FAILED: near the edge of the workspace a Cartesian speed needs big joint
        accelerations). Collisions and unreachable targets are not retried."""
        scale = self.speed * self.cfg.grasp.linear_speed
        for factor in LIN_SLOWDOWNS:
            code = self.motion.move_linear(
                xyz, scale * factor, self.cfg.planning_time_s, orientation
            )
            if code != MoveItErrorCodes.PLANNING_FAILED:
                break
            self.get_logger().warn(f"straight move refused at {scale * factor:.2f}: slower")
        return code

    def _linear(self, xyz: Vec3, what: str, orientation=None) -> None:
        code = self._move_linear(xyz, orientation)
        if code != MoveItErrorCodes.SUCCESS:
            raise TaskError(f"{what}: move_group {error_name(code)}")
        self._settle(what)

    def _grasp_and_check(self, cloth: Vec3, attempt: int) -> bool:
        """Phase 3 + 4. True = cloth in the gripper, lifted."""
        g = self.cfg.grasp
        log = self.get_logger()
        self._set_phase(Phase.TRIAL_GRASP)
        self._gripper(g.open_m, "open")
        above, q_above = self.motion.tcp_pose()
        target = grasp_point(cloth, above, g)
        dive_mm = (cloth[2] - target[2]) * 1000
        log.info(
            f"phase 3: straight down to z = {target[2]:.3f} m ({dive_mm:.0f} mm under the top), "
            f"straightening from {tilt_from_down_deg(q_above):.0f}° to vertical"
        )
        # Vertical at the bottom is best: tilted, the side of the open fingers is lower than the
        # tips. Far out, vertical is out of reach: then half the approach tilt, then all of it
        # (still collision-checked). If the open fingers would hit a wall, turn them 90°.
        vertical = vertical_keep_roll(q_above)
        tilts = (("vertical", vertical), ("half-tilted", slerp(q_above, vertical, 0.5)),
                 ("tilted", q_above))  # fmt: skip
        codes = []
        turns = DESCENT_TURNS_DEG if self.cfg.box is not None else (0.0,)  # walls: only in scene
        for (name, q_goal), turn in ((t, r) for t in tilts for r in turns):
            code = self._move_linear(target, turn_about_approach(q_goal, turn))
            if code == MoveItErrorCodes.SUCCESS:
                if name != "vertical" or turn:
                    log.info(f"descended {name}, fingers turned {turn:+.0f}°")
                break
            codes.append(f"{name} {turn:+.0f}°: {error_name(code)}")
        else:
            raise TaskError(
                "descent: no straight path down (" + "; ".join(codes) + "). NO_IK_SOLUTION = out "
                "of reach: keep the cloth within ~0.40 m of the arm base; collisions = the open "
                "fingers against a wall or the table (a smaller grasp.open_m helps)"
            )
        self._settle("descent")  # never close the gripper while the arm still moves
        closed = self._gripper(g.close_m, "close")
        self._set_phase(Phase.VALIDATE)
        log.info(f"phase 4: lifting {g.lift_m * 100:.0f} cm")
        self._linear(lifted(target, g), "lift", q_above)  # back to the approach tilt
        if not g.check_fingers:
            log.info(f"attempt {attempt}: grasp check off (grasp.check_fingers): carrying on")
            return True
        finger = self.motion.finger_position()
        finger = closed.position if finger is None else finger
        if holding_cloth(finger, g):
            log.info(
                f"attempt {attempt}: CLOTH IN THE GRIPPER (fingers at {finger * 1000:.1f} mm "
                f"≥ {g.empty_below_m * 1000:.1f} mm)"
            )
            return True
        log.warn(
            f"attempt {attempt}: MISSED (fingers at {finger * 1000:.1f} mm < "
            f"{g.empty_below_m * 1000:.1f} mm): opening and trying again"
        )
        self._gripper(g.open_m, "open")
        return False

    # --- phase 5 ---

    def _phase5_place(self, color: str | None) -> None:
        pl = self.cfg.place
        log = self.get_logger()
        self._set_phase(Phase.PLACE)
        try:
            tid = place_target_id(self.cfg, self.place_choice, color)
            self._update_status(target=tid)
        except ValueError as e:
            raise TaskError(str(e)) from e
        target = self.cfg.place_targets[tid]
        why = f" (class {color})" if self.place_choice == "color" else ""
        t0 = time.monotonic()
        if isinstance(target, str):
            log.info(f"phase 5: carrying to place target {tid}{why}: pose {target!r}")
            self._move_joints(self.poses[target], f"place target {tid} (pose {target!r})")
        else:
            log.info("phase 5: carrying to place target {}{}: ({:.3f}, {:.3f}, {:.3f}) m".format(
                tid, why, *target))  # fmt: skip
            # no downward path constraint on the way (the plan: "clear the 90° constraint"),
            # only a loose tilt limit at the target so the cloth drops off the fingers
            code = self.motion.move_pose(
                target,
                down_orientation(math.atan2(target[1], target[0])),
                self.speed,
                max_tilt_deg=pl.max_tilt_deg,
                max_roll_deg=pl.max_roll_deg,
                position_tolerance_m=pl.position_tolerance_m,
                keep_down_on_path=False,
                planning_time_s=pl.planning_time_s,
                planning_attempts=pl.planning_attempts,
            )
            if code != MoveItErrorCodes.SUCCESS:
                raise TaskError(
                    f"place target {tid}: move_group {error_name(code)} (out of reach? record a "
                    f"pose there and use its name in place_targets)"
                )
        self._settle(f"place target {tid}")
        self._gripper(self.cfg.grasp.open_m, "release")
        time.sleep(pl.release_wait_s)
        log.info(f"phase 5 done in {time.monotonic() - t0:.1f} s: cloth released at target {tid}")

    # --- planning scene and markers ---

    def _add_scene(self) -> None:
        if self.cfg.box is None:
            return
        if not self._scene.wait_for_service(timeout_sec=10.0):
            raise TaskError("/apply_planning_scene not available")
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(box_walls(self.cfg.box))
        res = Waiter(self._scene.call_async(ApplyPlanningScene.Request(scene=scene))).wait(10.0)
        if not res.success:
            raise TaskError("could not add the box to the planning scene")
        b = self.cfg.box
        self.get_logger().info(
            f"planning scene: box at ({b.center[0]:.2f}, {b.center[1]:.2f}) m, "
            f"{b.size[0] * 100:.0f}×{b.size[1] * 100:.0f}×{b.height * 100:.0f} cm"
        )

    def _publish_markers(self, cloth: Vec3, approach: Vec3) -> None:
        stamp = self.get_clock().now().to_msg()
        sphere = Marker(ns="supervisor", id=0, type=Marker.SPHERE, action=Marker.ADD)
        sphere.pose.position = Point(x=cloth[0], y=cloth[1], z=cloth[2])
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.02
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.5, 0.0, 1.0
        arrow = Marker(ns="supervisor", id=1, type=Marker.ARROW, action=Marker.ADD)
        arrow.points = [Point(x=v[0], y=v[1], z=v[2]) for v in (approach, cloth)]
        arrow.pose.orientation.w = 1.0
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.016, 0.02
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.2, 0.6, 1.0, 1.0
        for m in (sphere, arrow):
            m.header.frame_id, m.header.stamp = BASE_FRAME, stamp
        self._marker_pub.publish(MarkerArray(markers=[sphere, arrow]))

    def _set_phase(self, phase: Phase) -> None:
        if self.stopping.is_set() and phase is not Phase.ERROR:
            raise TaskError("stopped (Ctrl+C)")
        self.phase = phase
        self._phase_pub.publish(String(data=phase.value))
        self._update_status(phase=phase.value, phase_since=time.time())

    def _update_status(self, **fields) -> None:
        self.status.update(fields)
        self._status_pub.publish(String(data=json.dumps(self.status)))


def box_walls(b: BoxConfig) -> CollisionObject:
    """Four walls (no floor: the box stands on the table, which is already in the URDF)."""
    obj = CollisionObject(id="cloth_box", operation=CollisionObject.ADD)
    obj.header.frame_id = BASE_FRAME
    (cx, cy), (sx, sy), h, w = b.center, b.size, b.height, b.wall
    walls = [
        ((cx + (sx - w) / 2, cy), (w, sy)),
        ((cx - (sx - w) / 2, cy), (w, sy)),
        ((cx, cy + (sy - w) / 2), (sx, w)),
        ((cx, cy - (sy - w) / 2), (sx, w)),
    ]
    for (x, y), (dx, dy) in walls:
        obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[dx, dy, h]))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, h / 2
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)
    return obj


def main(args=None):
    # Own SIGINT handling: rclpy's would shut the context down first, and then the halt below
    # could not reach move_group. Ctrl+C must stop the current move, not let it run to the end.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = TaskSupervisor()
    except ConfigError as e:
        rclpy.logging.get_logger("task_supervisor_node").fatal(str(e))
        rclpy.try_shutdown()
        raise SystemExit(1) from e
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=node.run, daemon=True).start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl+C: halting the arm")
        node.stopping.set()
        node.motion.halt()  # "stop" to move_group + cancel our goal; later moves are refused
        with contextlib.suppress(Exception):
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:  # let the stop and the cancel go out
                executor.spin_once(timeout_sec=0.05)
    except ExternalShutdownException:
        pass
    finally:
        with contextlib.suppress(KeyboardInterrupt):  # Ctrl+C arrives twice under launch
            node.destroy_node()
            rclpy.try_shutdown()
