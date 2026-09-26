"""ArmController on top of an ArmDriver: named poses, pick / place / drop, hold, recover.

Contract: docs/architecture.md → Arm controller. The same controller runs the real arm and the
simulator; only the driver differs.
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Sequence

import numpy as np

from sorter.arm import kinematics as kin
from sorter.arm.config import POSE_NAMES, ArmConfig, ZoneConfig
from sorter.arm.driver import ArmDriver
from sorter.core.errors import EStopped, TargetRejected
from sorter.core.types import ArmPoint, ColorClass, PickResult, Pose, Zone

log = logging.getLogger(__name__)

_LOOK = {Zone.BOX: "look_box", Zone.BACKGROUND: "look_bg"}


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
        if set(zones) != set(Zone):
            raise ValueError("rig.yaml → zones needs `box` and `background`")
        self.driver = driver
        self.cfg = cfg
        self.poses = {name: np.asarray(poses[name], dtype=float) for name in POSE_NAMES}
        self.zones = zones
        self._held = threading.Event()
        self._at: str | None = None  # the named pose the arm is at, if any

    # --- helpers ---

    def _check_held(self) -> None:
        if self._held.is_set():
            raise EStopped("arm is held; recover() first")

    def _run(self, wps: np.ndarray) -> None:
        self._check_held()
        self._at = None
        self.driver.execute(wps, self.cfg.speed_scale)
        self._check_held()  # a hold that arrived as the motion ended

    def _go(self, name: str) -> None:
        self._check_held()
        wps = kin.plan_joints(self.driver.joints(), self.poses[name], z_min_mm=self.cfg.z_min_mm)
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
            wps = kin.plan_to(
                q,
                (T[0, 3], T[1, 3], self.cfg.safe_z_mm),
                T[:3, 0],
                linear=True,
                z_min_mm=self.cfg.z_min_mm,
            )
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
        before the motors go off, or it falls."""
        self._held.clear()
        self.driver.resume()
        self._lift()
        self._go("rest")
        self.driver.disconnect()
        log.info("arm at rest, motors off")

    def home(self) -> None:
        self._go("home")

    def look(self, zone: Zone) -> None:
        self._check_held()
        if self._at != _LOOK[zone]:
            self._go(_LOOK[zone])

    def plan_pick(
        self, target: ArmPoint, zone: Zone, q0: Sequence[float] | None = None
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Paths of a pick from `q0` (default: where the arm is): to above the target, down,
        up. TargetRejected if the target is outside the zone workspace or anything fails."""
        z = self.zones[zone]
        if not in_polygon(target.x, target.y, z.workspace_mm):
            raise TargetRejected(
                f"({target.x:.0f}, {target.y:.0f}) is outside the {zone} workspace"
            )
        above = (target.x, target.y, target.z + z.approach_mm)
        grasp = (target.x, target.y, max(target.z - z.grasp_depth_mm, z.z_floor_mm))
        lift = (target.x, target.y, max(z.lift_z_mm, above[2]))
        a, zmin = self.cfg.approach, self.cfg.z_min_mm
        q0 = self.driver.joints() if q0 is None else q0
        to_above = kin.plan_to(q0, above, a, z_min_mm=zmin)
        down = kin.plan_to(to_above[-1], grasp, a, linear=True, z_min_mm=zmin)
        up = kin.plan_to(down[-1], lift, a, linear=True, z_min_mm=zmin)
        return to_above, down, up

    def pick(self, target: ArmPoint, zone: Zone) -> PickResult:
        self._check_held()
        to_above, down, up = self.plan_pick(target, zone)  # a rejection happens with no motion
        self._run(to_above)
        self._gripper(self.cfg.gripper.open)
        self._run(down)
        opening = self._gripper(0.0)
        self._run(up)
        log.debug("pick at (%.0f, %.0f): gripper %.2f", target.x, target.y, opening)
        return PickResult(opening, likely_empty=opening < self.cfg.gripper.empty_below)

    def place_on_background(self) -> None:
        self._go("place_bg")
        self._gripper(self.cfg.gripper.open)

    def drop_to_bin(self, color: ColorClass) -> None:
        # via home: a straight joint move from the mat would sweep the gripper through the bin walls
        self._go("home")
        self._go(f"bin_{color.value}")
        self._gripper(self.cfg.gripper.open)
        self._go("home")  # and back the same way, clear of the walls

    def ee_pose(self) -> Pose:
        """T_base_link5: the link the camera is fixed to (it doesn't turn with joint 6)."""
        return kin.fk_link5(self.driver.joints())

    def joints(self) -> tuple[float, ...]:
        return tuple(float(v) for v in self.driver.joints())

    def gripper_opening(self) -> float:
        """Not in the ArmController protocol: for the dashboard's 3D view."""
        return self.driver.gripper()

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
        """Straight joint move to a named pose. ArmError if the path hits the table or base."""
        if name not in self.poses:
            raise ValueError(f"unknown pose {name!r}")
        self._go(name)

    def move_joints(self, q: Sequence[float]) -> None:
        """Straight joint move to `q` (rad). ArmError outside the joint limits or the table."""
        self._check_held()
        self._run(kin.plan_joints(self.driver.joints(), q, z_min_mm=self.cfg.z_min_mm))

    def move_tcp(self, xyz_mm: Sequence[float], *, linear: bool = False) -> None:
        """TCP to `xyz_mm` with the gripper pointing down (the calibration page).
        TargetRejected if IK or the path check fails; nothing moves then."""
        self._check_held()
        q0 = self.driver.joints()
        self._run(kin.plan_to(q0, xyz_mm, "down", linear=linear, z_min_mm=self.cfg.z_min_mm))

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
        if self._at == name:
            self._at = None

    def hold(self) -> None:
        self._held.set()
        self.driver.stop()
        log.warning("arm held")

    def recover(self) -> None:
        """Leave hold: lift, open the gripper above the background, home."""
        self._held.clear()
        self.driver.resume()
        self._at = None
        self._lift()
        self._go("place_bg")
        self._gripper(self.cfg.gripper.open)
        self.home()
        log.info("arm recovered")
