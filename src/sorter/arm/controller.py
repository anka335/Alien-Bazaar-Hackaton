"""`ArmController` for the SO-101 (D-014): named joint poses, straight top-down picks, hold.

Motions are streamed from the calling thread: min-jerk joint or straight-line trajectories,
`arm.control_hz` goal writes per second, then a wait until the arm is still. `hold()` may be
called from any thread: it freezes the goals at the measured position, and the motion in
progress raises `EStopped` at its next write.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable, Sequence

import numpy as np

from sorter.arm import kinematics as kin
from sorter.arm.bus import (
    ACCELERATION,
    GOAL_SPEED,
    P_COEFFICIENT,
    PRESENT_VOLTAGE,
    TICKS,
    TORQUE_LIMIT,
    Bus,
)
from sorter.arm.config import ArmConfig, ZoneConfig
from sorter.core.errors import ArmError, EStopped, TargetRejected
from sorter.core.types import ArmPoint, ColorClass, PickResult, Pose, Zone

log = logging.getLogger(__name__)

RAD_PER_TICK = 2 * math.pi / TICKS
LOOK_POSES = {Zone.BOX: "look_box", Zone.BACKGROUND: "look_bg"}
POSE_NAMES = (
    "rest",
    "home",
    "look_box",
    "look_bg",
    "place_bg",
    *(f"bin_{c.value}" for c in ColorClass),
)


def min_jerk(s: float) -> float:
    return s * s * s * (10 - 15 * s + 6 * s * s)


def inside(poly: Sequence[tuple[float, float]], x: float, y: float) -> bool:
    """Point in polygon (even-odd rule)."""
    n, hit = len(poly), False
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


class So101Arm:
    def __init__(
        self,
        cfg: ArmConfig,
        poses: dict[str, list[float]],
        zones: dict[Zone, ZoneConfig],
        bus_factory: Callable[[], Bus],
        sleep: Callable[[float], None] = time.sleep,
    ):
        self.cfg = cfg
        self.poses = poses
        self.zones = zones
        self._bus_factory = bus_factory
        self._sleep = sleep
        self.bus: Bus | None = None
        self._lock = threading.RLock()  # one goal write / hold at a time
        self._held = threading.Event()
        self._at: str | None = None  # named pose the arm is at, if it hasn't moved since
        self._grip_goal: int | None = None
        self._q_goal: np.ndarray | None = None  # last commanded joints; sag is not fed back
        self._mid = np.zeros(kin.N_JOINTS)
        self._limits = kin.LIMITS.copy()
        self._grip_range = (0, TICKS - 1)
        self._signs = np.asarray(cfg.signs, dtype=float)
        self._offsets = np.radians(np.asarray(cfg.offsets_deg, dtype=float))

    # --- connection and joint mapping ---

    def connect(self) -> None:
        """Open the bus and read the calibrated ranges. Torque stays as it is."""
        if self.bus is not None:
            return
        bus = self._bus_factory()
        margin = math.radians(self.cfg.limit_margin_deg)
        mids, limits = [], []
        for i, sid in enumerate(self.cfg.ids):
            lo, hi = bus.read_range(sid)
            if not 0 <= lo < hi < TICKS:
                raise ArmError(f"servo {sid}: bad EEPROM range {lo}..{hi}; calibrate with LeRobot")
            mid = (lo + hi) / 2
            a = self._signs[i] * (lo - mid) * RAD_PER_TICK + self._offsets[i]
            b = self._signs[i] * (hi - mid) * RAD_PER_TICK + self._offsets[i]
            mids.append(mid)
            limits.append((min(a, b) + margin, max(a, b) - margin))
        self._mid = np.array(mids)
        self._limits = np.array(limits)
        self._grip_range = bus.read_range(self.cfg.gripper_id)
        self.bus = bus
        volts = bus.read(self.cfg.ids[0], PRESENT_VOLTAGE) / 10
        log.info("SO-101 on the bus, %.1f V", volts)

    def _bus(self) -> Bus:
        if self.bus is None:
            raise ArmError("arm not started")
        return self.bus

    def q_from_raw(self, raw: Sequence[int]) -> np.ndarray:
        return self._signs * (np.asarray(raw, dtype=float) - self._mid) * RAD_PER_TICK + (
            self._offsets
        )

    def raw_from_q(self, q: Sequence[float]) -> list[int]:
        q = np.clip(np.asarray(q, dtype=float), self._limits[:, 0], self._limits[:, 1])
        raw = (q - self._offsets) / (self._signs * RAD_PER_TICK) + self._mid
        return [round(r) for r in raw]

    def grip_raw(self, opening: float) -> int:
        lo, hi = self._grip_range
        if self.cfg.gripper.reversed:
            lo, hi = hi, lo
        return round(lo + float(np.clip(opening, 0, 1)) * (hi - lo))

    def grip_opening(self, raw: int) -> float:
        lo, hi = self._grip_range
        if self.cfg.gripper.reversed:
            lo, hi = hi, lo
        return float(np.clip((raw - lo) / (hi - lo), 0, 1))

    def read_q(self) -> np.ndarray:
        return self.q_from_raw(self._bus().read_positions(self.cfg.ids))

    def read_gripper(self) -> float:
        return self.grip_opening(self._bus().read_positions([self.cfg.gripper_id])[0])

    def tcp(self, q: Sequence[float]) -> np.ndarray:
        """Fingertip point (mm) for joints q."""
        T = kin.fk(q)
        return T[:3, 3] + self.cfg.tcp_extend_mm * T[:3, 2]

    # --- torque ---

    def enable(self) -> None:
        """Torque on, holding the current position (goal = present first, so nothing jumps)."""
        bus = self._bus()
        ids = [*self.cfg.ids, self.cfg.gripper_id]
        with self._lock:
            present = bus.read_positions(ids)
            for sid in self.cfg.ids:
                bus.write(sid, ACCELERATION, 0)  # no ramp in the servo: goals are streamed
                bus.write(sid, GOAL_SPEED, 0)
                if self.cfg.p_gain is not None:  # LeRobot sets 16 (factory 32): stiffer = less sag
                    bus.write(sid, P_COEFFICIENT, self.cfg.p_gain)
            bus.write(self.cfg.gripper_id, TORQUE_LIMIT, self.cfg.gripper.torque_limit)
            bus.write_goals(dict(zip(ids, present, strict=True)))
            bus.set_torque(ids, True)
            self._grip_goal = present[-1]
            self._q_goal = self.q_from_raw(present[:-1])

    def disable(self) -> None:
        """Torque off: the arm goes limp. Only at `rest`, or while a person holds the arm."""
        self._bus().set_torque([*self.cfg.ids, self.cfg.gripper_id], False)
        self._at = None
        self._q_goal = None

    # --- streaming ---

    def _send(self, q: np.ndarray) -> None:
        with self._lock:
            if self._held.is_set():
                raise EStopped("arm is held; recover() first")
            goals = dict(zip(self.cfg.ids, self.raw_from_q(q), strict=True))
            if self._grip_goal is not None:
                goals[self.cfg.gripper_id] = self._grip_goal
            self._bus().write_goals(goals)
            self._q_goal = np.asarray(q, dtype=float)

    def _check_held(self) -> None:
        if self._held.is_set():
            raise EStopped("arm is held; recover() first")

    def _stream(self, path: Callable[[float], np.ndarray], duration_s: float) -> None:
        """Send path(min_jerk(t / duration)) at control_hz, then settle at path(1)."""
        self._check_held()
        self._at = None
        n = max(1, math.ceil(duration_s * self.cfg.control_hz))
        t0 = time.monotonic()
        for k in range(1, n + 1):
            self._send(path(min_jerk(k / n)))
            wait = t0 + k / self.cfg.control_hz - time.monotonic()
            if wait > 0:
                self._sleep(wait)

    def _wait_still(self) -> np.ndarray:
        """Wait until the joints stop moving (at most `settle_s`). Returns the arm joints.

        Servos start moving only some tens of ms after a new goal, so the first
        `settle_min_s` of reads never count as still."""
        ids = [*self.cfg.ids, self.cfg.gripper_id]
        deadline = time.monotonic() + self.cfg.settle_s
        min_reads = math.ceil(self.cfg.settle_min_s * self.cfg.control_hz)
        prev, still, reads = None, 0, 0
        while True:
            self._check_held()
            raw = np.array(self._bus().read_positions(ids))
            reads += 1
            still = still + 1 if prev is not None and np.all(np.abs(raw - prev) <= 2) else 0
            prev = raw
            settled = still >= self.cfg.still_ticks and reads >= min_reads
            if settled or time.monotonic() > deadline:
                return self.q_from_raw(raw[:-1])
            self._sleep(1 / self.cfg.control_hz)

    def _settle(self, q_goal: np.ndarray | None, strict: bool = True) -> np.ndarray:
        """Wait until the arm is still. For a strict goal, command away the sag: the servos are
        P-controlled, so under load they stop short; up to `sag_passes` times the remaining
        error is added to the command. ArmError if a joint still misses by `max_error_deg`."""
        q = self._wait_still()
        if q_goal is None or not strict:
            return q
        for i in range(self.cfg.sag_passes):
            err = q_goal - q
            log.debug("settle pass %d: error %s°", i, np.degrees(err).round(2))
            if np.degrees(np.abs(err)).max() <= self.cfg.sag_tol_deg:
                break
            base = self._q_goal if self._q_goal is not None else q_goal
            step = np.clip(self.cfg.sag_gain * err, -math.radians(20), math.radians(20))
            self._send(base + step)
            q = self._wait_still()
        err = np.degrees(np.abs(q - q_goal))
        log.debug("settled: error %s°", np.degrees(q_goal - q).round(2))
        if err.max() > self.cfg.max_error_deg:
            j = int(np.argmax(err))
            raise ArmError(f"{kin.JOINT_NAMES[j]} stopped {err[j]:.0f}° from its goal (blocked?)")
        return q

    def move_joints(self, q_goal: Sequence[float], speed_scale: float = 1.0) -> None:
        q1 = np.clip(np.asarray(q_goal, dtype=float), self._limits[:, 0], self._limits[:, 1])
        q0 = self._q_goal if self._q_goal is not None else self.read_q()
        v = math.radians(self.cfg.joint_speed_deg_s) * speed_scale
        duration = max(0.3, 1.875 * float(np.abs(q1 - q0).max()) / v)  # 1.875: min-jerk peak
        self._stream(lambda s: q0 + s * (q1 - q0), duration)
        self._settle(q1)

    def set_gripper(self, opening: float) -> float:
        """Move the gripper, wait `gripper.close_s`, return the measured opening."""
        self._check_held()
        self._grip_goal = self.grip_raw(opening)
        self._send(self._q_goal if self._q_goal is not None else self.read_q())
        self._sleep(self.cfg.gripper.close_s)
        return self.read_gripper()

    def goto(self, name: str, speed_scale: float = 1.0) -> None:
        if name not in self.poses:
            raise ArmError(f"pose {name!r} not taught; run `python -m sorter.arm.teach`")
        if self._at == name:
            return
        self.move_joints(self.poses[name], speed_scale)
        self._at = name

    # --- straight top-down moves ---

    def _frame_target(self, p: np.ndarray) -> np.ndarray:
        return p + np.array([0.0, 0.0, self.cfg.tcp_extend_mm])  # tool pointing down

    def solve_down(
        self,
        p: Sequence[float],
        seed: Sequence[float],
        max_tilt_deg: float,
        other_seeds: bool = True,
    ):
        """IK for the fingertips at p (mm), tool down within max_tilt_deg. None if unreachable.

        `other_seeds` also tries the home pose and a generic elbow-up pose as seeds; waypoints of
        a line don't, so consecutive solutions stay on the same branch."""
        p = np.asarray(p, dtype=float)
        best = None
        seeds = (
            (seed, self.poses.get("home"), [0.0, 0.0, 0.5, 1.0, 0.0]) if other_seeds else (seed,)
        )
        for s in seeds:
            if s is None:
                continue
            r = kin.ik_down(
                self._frame_target(p),
                s,
                roll=math.radians(self.cfg.roll_deg),
                limits=self._limits,
            )
            if r.pos_err_mm < 2.0 and r.tilt_deg <= max_tilt_deg:
                if best is None or r.tilt_deg < best.tilt_deg - 1:
                    best = r
                if r.tilt_deg < 2:
                    break
        return best

    def plan_line(
        self, p0: Sequence[float], p1: Sequence[float], seed: Sequence[float], max_tilt_deg: float
    ) -> list[np.ndarray]:
        """Joint waypoints of a straight fingertip line p0 → p1. TargetRejected if any point is
        unreachable or below the table."""
        p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
        if min(p0[2], p1[2]) < self.cfg.table_z_mm:
            raise TargetRejected(f"line goes below the table ({min(p0[2], p1[2]):.0f} mm)")
        n = max(1, math.ceil(float(np.linalg.norm(p1 - p0)) / self.cfg.step_mm))
        qs, q = [], np.asarray(seed, dtype=float)
        for k in range(n + 1):
            p = p0 + (p1 - p0) * k / n
            r = self.solve_down(p, q, max_tilt_deg, other_seeds=False)
            if r is None:
                x, y, z = p
                raise TargetRejected(f"({x:.0f}, {y:.0f}, {z:.0f}) mm is out of reach")
            q = r.q
            qs.append(q)
        return qs

    def follow(self, qs: list[np.ndarray], length_mm: float, strict: bool = True) -> np.ndarray:
        """Stream joint waypoints (equally spaced along a line of `length_mm`)."""
        wp = np.array(qs)

        def path(s: float) -> np.ndarray:
            x = s * (len(wp) - 1)
            i = min(int(x), len(wp) - 2) if len(wp) > 1 else 0
            if len(wp) == 1:
                return wp[0]
            return wp[i] + (x - i) * (wp[i + 1] - wp[i])

        duration = max(0.3, length_mm / self.cfg.linear_speed_mm_s)
        self._stream(path, duration)
        return self._settle(wp[-1], strict=strict)

    # --- ArmController ---

    def start(self) -> None:
        self.connect()
        self._held.clear()
        self.enable()
        log.info("arm enabled, holding position")

    def shutdown(self) -> None:
        if self.bus is None:
            return
        self._held.clear()  # shutting down is an explicit request to move to rest
        try:
            self.goto("rest", speed_scale=0.5)
        except Exception:
            log.exception("can't reach rest: motors stay on (holding) instead of dropping the arm")
            self.hold()
        else:
            self.disable()
            log.info("arm at rest, motors off")
        finally:
            self.bus.close()
            self.bus = None

    def home(self) -> None:
        self.goto("home")

    def look(self, zone: Zone) -> None:
        self._check_held()
        name = LOOK_POSES[zone]
        if self._at == name:
            return
        self.goto(name)
        self.set_gripper(self.cfg.gripper.look)  # fixed finger pixels in the look frame
        self._at = name

    def pick(self, target: ArmPoint, zone: Zone) -> PickResult:
        self._check_held()
        zc = self.zones.get(zone)
        if zc is None or not zc.workspace_mm:
            raise TargetRejected(f"no workspace for {zone}; run the calibration tool")
        if not inside(zc.workspace_mm, target.x, target.y):
            raise TargetRejected(
                f"({target.x:.0f}, {target.y:.0f}) is outside the {zone} workspace"
            )
        above = np.array([target.x, target.y, target.z + zc.approach_mm])
        grasp = np.array([target.x, target.y, max(target.z - zc.grasp_depth_mm, zc.z_floor_mm)])
        tilt = self.cfg.grasp_max_tilt_deg
        q_above = self.solve_down(above, self.read_q(), tilt)
        if q_above is None:
            raise TargetRejected(f"approach point {above.round()} mm is out of reach")
        down = self.plan_line(above, grasp, q_above.q, tilt)  # all planned before moving
        length = float(above[2] - grasp[2])

        self.move_joints(q_above.q)
        self.set_gripper(self.cfg.gripper.open)
        self.follow(down, length, strict=False)  # cloth or the floor may stop it: fine
        opening = self.set_gripper(self.cfg.gripper.closed)
        self.follow(down[::-1], length)
        likely_empty = opening < self.cfg.gripper.empty_below
        log.info("pick in %s: gripper opening %.2f%s", zone, opening, " (empty?)" * likely_empty)
        return PickResult(gripper_opening=opening, likely_empty=likely_empty)

    def place_on_background(self) -> None:
        self.goto("place_bg")
        self.set_gripper(self.cfg.gripper.open)

    def drop_to_bin(self, color: ColorClass) -> None:
        """Bins are off the table: go there and back through the high `home` pose, so the swing
        clears everything on the table."""
        self.goto("home")
        self.goto(f"bin_{color.value}")
        self.set_gripper(self.cfg.gripper.open)
        self.goto("home")

    def ee_pose(self) -> Pose:
        return kin.fk(self.read_q())

    def joints(self) -> tuple[float, ...]:
        return tuple(float(x) for x in self.read_q())

    def hold(self) -> None:
        self._held.set()
        if self.bus is None:
            return
        with self._lock:
            try:
                ids = [*self.cfg.ids, self.cfg.gripper_id]
                present = self.bus.read_positions(ids)
                self.bus.write_goals(dict(zip(ids, present, strict=True)))
                self._q_goal = self.q_from_raw(present[:-1])
                self._grip_goal = present[-1]
            except ArmError:
                log.exception("hold: can't freeze the goals")
        self._at = None
        log.warning("arm held")

    def recover(self) -> None:
        self._held.clear()
        if self.bus is None:
            self.start()
        q = self.read_q()
        p = self.tcp(q)
        try:
            up = self.plan_line(p, p + [0, 0, self.cfg.recover_lift_mm], q, 60.0)
            self.follow(up, self.cfg.recover_lift_mm)
        except TargetRejected:
            log.info("recover: no straight lift from here, going to place_bg directly")
        self.goto("place_bg", speed_scale=0.5)
        self.set_gripper(self.cfg.gripper.open)
        self.home()
        log.info("arm recovered")
