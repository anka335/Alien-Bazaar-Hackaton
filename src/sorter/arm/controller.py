"""ArmController on top of an ArmDriver: named poses, pick / drop, hold, recover.

Contract: docs/architecture.md → Arm controller. The same controller runs the real arm and the
simulator; only the driver differs.
"""

from __future__ import annotations

import logging
import math
import threading
from collections.abc import Sequence

import numpy as np
from rebot_b601 import config as rc

from sorter.arm import kinematics as kin
from sorter.arm.config import LOOK_POSES, POSE_NAMES, ArmConfig, ZoneConfig
from sorter.arm.driver import TRACKING_FAULT, ArmDriver
from sorter.core.errors import ArmError, EStopped, TargetRejected
from sorter.core.types import ArmPoint, ColorClass, PickResult, Pose, Zone

log = logging.getLogger(__name__)

MIN_SPEED_SCALE = 0.05


def speed_ceiling() -> float:
    """The speed_scale at which the fastest joint's peak speed reaches the velocity limit
    programmed into the motors (`REBOT_MOTOR_VLIM`, 1.5 rad/s): ~1.43."""
    return rc.MOTOR_VLIM_RAD_S / float(np.radians(rc.JOINT_SPEED_DPS).max())


def in_polygon(x: float, y: float, poly: Sequence[tuple[float, float]]) -> bool:
    """Even-odd rule; points on the edge count as inside only by chance."""
    inside = False
    n = len(poly)
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


class Controller:
    def __init__(
        self,
        driver: ArmDriver,
        cfg: ArmConfig,
        poses: dict[str, list[float]],
        zones: dict[Zone, ZoneConfig],
    ):
        missing = [p for p in POSE_NAMES if p not in poses]
        if missing:
            raise ValueError(
                f"poses {missing} missing in config (rig.yaml → poses); "
                "for the sim layout: python -m sorter.sim.layout"
            )
        if set(zones) != set(LOOK_POSES):
            names = " and ".join(f"`{z.value}`" for z in LOOK_POSES)
            raise ValueError(f"rig.yaml → zones needs {names}")
        self.driver = driver
        self.cfg = cfg
        self.poses = {name: np.asarray(poses[name], dtype=float) for name in POSE_NAMES}
        self.zones = zones
        self._held = threading.Event()
        self._at: str | None = None  # the named pose the arm is at, if any
        self._drop_plans: dict[str, tuple | None] = {}  # drops from above from home; None: none
        self._max_speed = min(cfg.max_speed_scale, speed_ceiling())
        if self._max_speed < cfg.max_speed_scale:
            log.warning(
                "arm.max_speed_scale %.2f is above what the motors follow; using %.2f",
                cfg.max_speed_scale,
                self._max_speed,
            )
        self._speed = min(max(cfg.speed_scale, MIN_SPEED_SCALE), self._max_speed)

    # --- helpers ---

    def _check_held(self) -> None:
        if self._held.is_set():
            raise EStopped("arm is held; recover() first")

    def _run(self, wps: np.ndarray) -> None:
        """Run a planned path. A tracking fault (a joint fell behind: late or blocked) is
        cleared once and the rest of the path run at half speed from where the arm stopped; a
        second one, or any other fault, raises."""
        self._check_held()
        self._at = None
        try:
            self.driver.execute(wps, self._speed)
        except EStopped:
            raise
        except ArmError:
            fault = self.driver.fault()
            if fault is None or TRACKING_FAULT not in fault:
                raise
            log.warning("%s; cleared, going on at half speed", fault)
            self.driver.clear_fault()
            self._check_held()
            q = np.asarray(self.driver.joints(), dtype=float)
            k = int(np.argmin(np.linalg.norm(wps - q, axis=1)))  # where on the path it stopped
            rest = np.vstack([q, wps[min(k + 1, len(wps) - 1) :]])
            self.driver.execute(rest, max(self._speed / 2, MIN_SPEED_SCALE))
        self._check_held()  # a hold that arrived as the motion ended

    def _plan_joints(self, q0: Sequence[float], q1: Sequence[float]) -> np.ndarray:
        c = self.cfg
        return kin.plan_joints(
            q0,
            q1,
            z_min_mm=c.z_min_mm,
            keep_out=c.keep_out_mm,
            keep_out_margin_mm=c.keep_out_margin_mm,
            link5_points=c.link5_points_mm,
        )

    def plan_move(
        self, q0: Sequence[float], q1: Sequence[float], via_home: bool = True
    ) -> np.ndarray:
        """Joint waypoints from `q0` to `q1`: straight if that's clear, else turning joint 1
        first (the arm swings round as it is, then reaches) or last, else (`via_home`) the same
        way to home and on from there. ArmError if nothing is clear."""
        q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
        try:
            return self._plan_direct(q0, q1)
        except ArmError:
            home = self.poses["home"]
            if not via_home or np.allclose(q0, home) or np.allclose(q1, home):
                raise
        return np.vstack([self._plan_direct(q0, home), self._plan_direct(home, q1)[1:]])

    def _plan_direct(self, q0: np.ndarray, q1: np.ndarray) -> np.ndarray:
        swing_first, swing_last = q0.copy(), q1.copy()
        swing_first[0], swing_last[0] = q1[0], q0[0]
        err: ArmError | None = None
        for wps in ([q0, q1], [q0, swing_first, q1], [q0, swing_last, q1]):
            try:
                for a, b in zip(wps, wps[1:], strict=False):
                    self._plan_joints(a, b)
            except ArmError as e:
                err = err or e
                continue
            return np.array(wps)
        assert err is not None
        raise err

    def _plan_to(
        self, q0: Sequence[float], xyz_mm: Sequence[float], approach, linear: bool = False
    ) -> np.ndarray:
        c = self.cfg
        return kin.plan_to(
            q0,
            xyz_mm,
            approach,
            linear=linear,
            z_min_mm=c.z_min_mm,
            keep_out=c.keep_out_mm,
            keep_out_margin_mm=c.keep_out_margin_mm,
            link5_points=c.link5_points_mm,
        )

    def _go(self, name: str) -> None:
        """Straight joint move to a named pose; via home if that one would hit something (from
        rest, say, the gripper sweeps past the base)."""
        self._check_held()
        try:
            wps = self.plan_move(self.driver.joints(), self.poses[name])
        except ArmError:
            if "home" in (name, self._at):
                raise
            log.info("no straight move to %s: via home", name)
            self._run(self.plan_move(self.driver.joints(), self.poses["home"]))
            self._at = "home"
            wps = self.plan_move(self.driver.joints(), self.poses[name])
        self._run(wps)
        self._at = name

    def _gripper(self, opening: float) -> float:
        self._check_held()
        return self.driver.set_gripper(opening)

    def _lift(self) -> None:
        """Straight up to `safe_z_mm`, keeping the tool orientation. Best effort."""
        q = self.driver.joints()
        T = kin.fk_tcp(q)
        if T[2, 3] >= self.cfg.safe_z_mm - 1:
            return
        try:
            wps = self._plan_to(q, (T[0, 3], T[1, 3], self.cfg.safe_z_mm), T[:3, 0], linear=True)
        except TargetRejected as e:
            log.warning("no straight lift from the current pose (%s); moving on", e)
            return
        self._run(wps)

    # --- ArmController ---

    def start(self) -> None:
        self.driver.connect()
        self._at = None
        log.info("arm started")

    def shutdown(self) -> None:
        """Rest pose, then disable. A pending hold is released first: the arm must be at rest
        before the motors go off, or it falls. Nothing to do if the arm never connected (a failed
        start): reading its joints would raise and hide the start's error."""
        if not self.driver.connected:
            log.info("arm not connected: nothing to shut down")
            return
        self._held.clear()
        self.driver.resume()
        if (fault := self.driver.fault()) is not None:  # or rest fails and the bus stays open
            log.warning("clearing the arm's fault before the rest pose: %s", fault)
            self.driver.clear_fault()
        self._lift()
        self._go("rest")
        self.driver.disconnect()
        log.info("arm at rest, motors off")

    def home(self) -> None:
        self._go("home")

    def look(self, zone: Zone) -> None:
        self._check_held()
        if zone not in LOOK_POSES:
            raise ValueError(f"no look pose for {zone}")
        if self._at != LOOK_POSES[zone]:
            self._go(LOOK_POSES[zone])

    def aim_camera(
        self,
        T_link5_cam: Pose,
        target: Sequence[float],
        heights_mm: Sequence[float],
        tilts_deg: Sequence[float] = (0.0, 10.0, 20.0, 30.0),
    ) -> float | None:
        """Point the camera (fixed to link5 by `T_link5_cam`, the hand-eye result) at `target`
        (mm) from the first of `heights_mm` the arm reaches safely; the camera leans up to
        `tilts_deg` off vertical. The camera's height above the target, or None (no motion) if
        no pose works."""
        self._check_held()
        q0 = self.driver.joints()
        c = self.cfg
        home = self.poses["home"]

        def reachable(q: np.ndarray) -> bool:  # a pose with a clear path there, else the next
            for a in (q0, home):
                try:
                    self.plan_move(a, q, via_home=False)
                    return True
                except ArmError:
                    continue
            return False

        found = kin.camera_look(
            T_link5_cam,
            target,
            q0,
            heights_mm=heights_mm,
            tilts_deg=tilts_deg,
            z_min_mm=c.z_min_mm,
            keep_out=c.keep_out_mm,
            keep_out_margin_mm=c.keep_out_margin_mm,
            link5_points=c.link5_points_mm,
            accept=reachable,
        )
        if found is None:
            return None
        q, h, _ = found
        try:
            wps = self.plan_move(q0, q)
        except ArmError:  # a straight move would hit something: via home
            self._go("home")
            wps = self.plan_move(self.driver.joints(), q)
        self._run(wps)
        return h

    def plan_pick(
        self,
        target: ArmPoint,
        zone: Zone,
        yaw_rad: float | None = None,
        q0: Sequence[float] | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Paths of a pick from `q0` (default: where the arm is): to above the target, down,
        up. `yaw_rad`: the fingers open along this direction (angle from +x in the arm frame;
        None = as the IK comes). TargetRejected if the target is outside the zone workspace or
        anything fails."""
        z = self.zones[zone]
        if not in_polygon(target.x, target.y, z.workspace_mm):
            raise TargetRejected(
                f"({target.x:.0f}, {target.y:.0f}) is outside the {zone} workspace"
            )
        above = (target.x, target.y, target.z + z.approach_mm)
        grasp = (target.x, target.y, max(target.z - z.grasp_depth_mm, z.z_floor_mm))
        lift = (target.x, target.y, max(z.lift_z_mm, above[2]))
        q0 = np.asarray(self.driver.joints() if q0 is None else q0, dtype=float)
        # the IK's answer depends on its seed: from where the arm is, else elbow up towards the
        # target (the arm's usual shape over the floor)
        canonical = np.array([kin.joint1_toward(target.x, target.y), 1.2, 1.5, 0.0, 0.0, 0.0])
        err: TargetRejected | None = None
        for seed in (q0, canonical):
            try:
                return self._plan_pick_from(q0, seed, above, grasp, lift, yaw_rad)
            except TargetRejected as e:
                err = err or e
        assert err is not None
        raise err

    def _plan_pick_from(self, q0, seed, above, grasp, lift, yaw_rad):
        a = self.cfg.approach
        q_above = kin.solve(above, a, seed, self.cfg.z_min_mm)
        if q_above is None:
            raise TargetRejected(f"no IK solution above ({above[0]:.0f}, {above[1]:.0f})")
        if yaw_rad is not None:
            q_above = kin.with_yaw(q_above, yaw_rad)
            if q_above is None:
                raise TargetRejected(f"joint 6 can't turn the gripper to {yaw_rad:.2f} rad")
        try:  # a joint move, swinging joint 1 first or last if a straight one would hit something
            to_above = self.plan_move(q0, q_above)
        except ArmError as e:
            raise TargetRejected(str(e)) from None
        down = self._plan_to(to_above[-1], grasp, a, linear=True)
        up = self._plan_to(down[-1], lift, a, linear=True)
        return to_above, down, up

    def pick(self, target: ArmPoint, zone: Zone, yaw_rad: float | None = None) -> PickResult:
        self._check_held()
        # a rejection happens with no motion
        to_above, down, up = self.plan_pick(target, zone, yaw_rad)
        self._run(to_above)
        self._gripper(self.cfg.gripper.open)
        self._run(down)
        self._gripper(0.0)
        self._run(up)
        # read once lifted: a sock that slipped out on the way up reads empty too
        opening = self.driver.gripper()
        log.debug("pick at (%.0f, %.0f): gripper %.3f", target.x, target.y, opening)
        return PickResult(opening, likely_empty=opening < self.cfg.gripper.empty_below)

    def _drop(self, pose: str) -> None:
        # via home: a straight joint move from a pick would sweep the gripper through walls
        self._go("home")
        self._go(pose)
        self._check_held()
        self.driver.wait(self.cfg.drop_settle_s)  # the hanging sock stops swinging first
        self._gripper(self.cfg.gripper.open)
        self._go("home")  # and back the same way, clear of the walls

    def drop_to_cargo(self, color: ColorClass) -> None:
        """Over the box high up, then straight down to `cargo_<color>` and back up. A move in at
        the drop height drags the sock hanging from the fingers across the wall, and it stays
        there, half out. Without that path (IK, keep-out): the plain drop."""
        pose = f"cargo_{color.value}"
        self._go("home")
        try:
            if self._drop_plans.get(pose, ()) is None:
                raise TargetRejected("didn't plan before")
            # from `home` itself, as the layout check plans it: the IK from where the arm
            # stopped (a hair off home) may not converge
            if pose not in self._drop_plans:  # the same plan every time: IK takes seconds
                self._drop_plans[pose] = self._plan_drop_from_above(pose, self.poses["home"])
            in_, down, up = self._drop_plans[pose]
        except TargetRejected as e:
            self._drop_plans[pose] = None
            log.warning("no drop into %s from above (%s): the plain drop", pose, e)
            self._drop(pose)
            return
        self._run(in_)
        self._run(down)
        self._check_held()
        self.driver.wait(self.cfg.drop_settle_s)  # the hanging sock stops swinging first
        self._gripper(self.cfg.gripper.open)
        self._run(up)  # out of the cloth, then straight up: the fingers don't drag the sock out
        self._go("home")

    def _plan_drop_from_above(
        self, pose: str, q0: Sequence[float] | None = None
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(`q0` (default: where the arm is) → above, above → drop, drop → back along the tool
        → above it) with the gripper tilted outwards by the first of `cargo_drop_tilts_deg`
        (leaning away from the arm's base, turned by one of `cargo_drop_azimuths_deg`) that
        plans. TargetRejected if none does."""
        q0 = self.driver.joints() if q0 is None else q0
        err: TargetRejected | None = None
        for tilt in self.cfg.cargo_drop_tilts_deg:
            for az in self.cfg.cargo_drop_azimuths_deg:
                try:
                    return self._plan_drop_tilted(pose, q0, tilt, az)
                except TargetRejected as e:
                    err = err or e
        assert err is not None
        raise err

    def _plan_drop_tilted(
        self, pose: str, q0: Sequence[float], tilt_deg: float, azimuth_deg: float = 0.0
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        c = self.cfg
        x, y, z = kin.fk_tcp(self.poses[pose])[:3, 3]
        az = math.atan2(y, x) + math.radians(azimuth_deg)  # 0: straight away from the base
        t = math.radians(tilt_deg)
        approach = np.array((math.cos(az) * math.sin(t), math.sin(az) * math.sin(t), -math.cos(t)))
        z_above = z + c.cargo_drop_above_mm
        in_ = self._plan_to(q0, (x, y, z_above), approach)
        release = np.array((x, y, z - c.cargo_drop_depth_mm))
        down = self._plan_to(in_[-1], release, approach, linear=True)
        back = release - c.cargo_drop_back_mm * approach
        out = self._plan_to(down[-1], back, approach, linear=True)
        # then up: the fingers are out of the cloth, so lower is fine where the arm (moved out
        # by the back-out) doesn't reach that high
        err: TargetRejected | None = None
        for zu in (z_above, z_above - 25.0, z_above - 50.0):
            try:
                up = self._plan_to(out[-1], (*back[:2], zu), approach, linear=True)
                return in_, down, np.vstack([out, up[1:]])
            except TargetRejected as e:
                err = err or e
        assert err is not None
        raise err

    def drop_to_laundry(self, color: ColorClass) -> None:
        self._drop(f"laundry_{color.value}")

    def ee_pose(self) -> Pose:
        """T_base_link5: the link the camera is fixed to (it doesn't turn with joint 6)."""
        return kin.fk_link5(self.driver.joints())

    @property
    def connected(self) -> bool:
        """Not in the ArmController protocol: the motors are on (`start()` done)."""
        return self.driver.connected

    def joints(self) -> tuple[float, ...]:
        return tuple(float(v) for v in self.driver.joints())

    def gripper_opening(self) -> float:
        """Not in the ArmController protocol: for the dashboard's 3D view."""
        return self.driver.gripper()

    # --- speed (the dashboard's speed control), not in the ArmController protocol ---

    @property
    def speed_scale(self) -> float:
        """Of rebot_b601's joint speeds; starts at `arm.speed_scale`."""
        return self._speed

    @property
    def max_speed_scale(self) -> float:
        """`arm.max_speed_scale`, at most `speed_ceiling()`."""
        return self._max_speed

    def set_speed_scale(self, scale: float) -> float:
        """From the next motion on (one under way keeps its speed), clamped to
        [MIN_SPEED_SCALE, max_speed_scale]. Returns the speed set."""
        if not math.isfinite(scale):
            raise ValueError(f"speed_scale must be a number, not {scale}")
        self._speed = min(max(float(scale), MIN_SPEED_SCALE), self._max_speed)
        log.info("arm speed_scale %.2f", self._speed)
        return self._speed

    # --- manual control (the dashboard's setup page), not in the ArmController protocol ---

    @property
    def held(self) -> bool:
        return self._held.is_set()

    @property
    def at(self) -> str | None:
        """The named pose the arm is at, if any."""
        return self._at

    @property
    def fault(self) -> str | None:
        """A latched driver fault (e.g. a blocked joint); every motion fails until clear_fault()."""
        return self.driver.fault()

    def go_to(self, name: str) -> None:
        """Straight joint move to a named pose. ArmError if the path hits the floor, the base or a
        keep-out box."""
        if name not in self.poses:
            raise ValueError(f"unknown pose {name!r}")
        self._go(name)

    def move_joints(self, q: Sequence[float]) -> None:
        """Straight joint move to `q` (rad). ArmError outside the joint limits, below the floor
        or into a keep-out box."""
        self._check_held()
        self._run(self._plan_joints(self.driver.joints(), q))

    def move_tcp(self, xyz_mm: Sequence[float], *, linear: bool = False) -> None:
        """TCP to `xyz_mm` with the gripper pointing down (the calibration page).
        TargetRejected if IK or the path check fails; nothing moves then."""
        self._check_held()
        self._run(self._plan_to(self.driver.joints(), xyz_mm, "down", linear=linear))

    def lift(self) -> None:
        """Straight up to `safe_z_mm` if lower, keeping the tool orientation. Best effort."""
        self._check_held()
        self._lift()

    def set_gripper(self, opening: float) -> float:
        return self._gripper(opening)

    def release(self) -> None:
        """Leave hold without moving (recover() without its lift / open / home)."""
        self._held.clear()
        self.driver.resume()
        self._at = None
        log.info("arm hold released")

    def clear_fault(self) -> None:
        """Accept a driver fault after inspecting the arm: hold where it is now, leave hold."""
        self.driver.clear_fault()
        self.release()

    def set_pose(self, name: str, q: Sequence[float]) -> None:
        """Replace a named pose for this process (a re-taught pose)."""
        if name not in self.poses:
            raise ValueError(f"unknown pose {name!r}")
        self.poses[name] = np.asarray(q, dtype=float)
        self._drop_plans.clear()
        if self._at == name:
            self._at = None

    def hold(self) -> None:
        self._held.set()
        self.driver.stop()
        log.warning("arm held")

    def recover(self) -> None:
        """Leave hold and a latched driver fault: lift, open the gripper over the floor view
        (what it holds drops there), home."""
        self._held.clear()
        self.driver.resume()
        if (fault := self.driver.fault()) is not None:
            log.warning("clearing the arm's fault: %s", fault)
            self.driver.clear_fault()
        self._at = None
        self._lift()
        self._go("look_floor")
        self._gripper(self.cfg.gripper.open)
        self.home()
        log.info("arm recovered")
