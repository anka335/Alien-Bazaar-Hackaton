"""SimArm: implements ArmController on the SimWorld."""

from __future__ import annotations

import logging
import math
import threading

import numpy as np

from sorter.core.errors import EStopped, TargetRejected
from sorter.core.types import ArmPoint, ColorClass, PickResult, Pose, Zone
from sorter.sim.world import BG_ITEM_HEIGHT_MM, SimItem, SimWorld

log = logging.getLogger(__name__)

_HOME_XYZ = (300.0, 0.0, 450.0)
_JOINTS: dict[Zone | None, tuple[float, ...]] = {  # fake joint angles per pose, rad
    None: (0.0, -0.5, 1.0, 0.0, 0.5, 0.0),
    Zone.BOX: (-0.5, -0.3, 0.9, 0.0, 0.9, 0.0),
    Zone.BACKGROUND: (0.4, -0.3, 0.9, 0.0, 0.9, 0.0),
}


class SimArm:
    def __init__(self, world: SimWorld):
        self.world = world
        self._held = threading.Event()

    # --- motion helpers ---

    def _check_held(self) -> None:
        if self._held.is_set():
            raise EStopped("arm is held; recover() first")

    def _move(self) -> None:
        """One blocking motion. A hold during the motion interrupts it."""
        self._check_held()
        if self.world.cfg.motion_s > 0 and self._held.wait(self.world.cfg.motion_s):
            raise EStopped("arm held during motion")
        self._check_held()

    def _land_on_background(self, items: list[SimItem]) -> None:
        for it in items:
            it.x, it.y = self.world.free_point_on_background()
            it.height_mm = BG_ITEM_HEIGHT_MM
            it.location = "background"

    # --- ArmController ---

    def start(self) -> None:
        log.info("sim arm started")

    def shutdown(self) -> None:
        with self.world.lock:
            self.world.looking_at = None
        log.info("sim arm at rest, motors off")

    def home(self) -> None:
        self._move()
        with self.world.lock:
            self.world.looking_at = None

    def look(self, zone: Zone) -> None:
        self._check_held()
        if self.world.looking_at == zone:
            return
        self._move()
        with self.world.lock:
            self.world.looking_at = zone

    def pick(self, target: ArmPoint, zone: Zone) -> PickResult:
        self._check_held()
        cfg = self.world.cfg
        if not self.world.views[zone].contains(target.x, target.y):
            where = f"({target.x:.0f}, {target.y:.0f})"
            raise TargetRejected(f"{where} is outside the {zone} workspace")
        self._move()
        with self.world.lock:
            self.world.looking_at = None
            candidates = [
                it
                for it in self.world.at(zone.value)
                if math.hypot(it.x - target.x, it.y - target.y) <= 1.2 * cfg.item_radius_mm
            ]
            grabbed: list[SimItem] = []
            if candidates and self.world.rng.random() >= cfg.miss_prob:
                grabbed.append(max(candidates, key=lambda it: it.height_mm))
                if zone is Zone.BOX and self.world.rng.random() < cfg.double_prob:
                    rest = [it for it in self.world.at("box") if it is not grabbed[0]]
                    if rest:
                        grabbed.append(
                            min(rest, key=lambda it: math.hypot(it.x - target.x, it.y - target.y))
                        )
            for it in grabbed:
                it.location = "gripper"
        if grabbed:
            log.info("sim pick from %s: %s", zone, [it.id for it in grabbed])
        return PickResult(gripper_opening=0.3 if grabbed else 0.0, likely_empty=not grabbed)

    def place_on_background(self) -> None:
        self._move()
        with self.world.lock:
            self._land_on_background(self.world.at("gripper"))
            self.world.looking_at = None

    def drop_to_bin(self, color: ColorClass) -> None:
        self._move()
        with self.world.lock:
            for it in self.world.at("gripper"):
                it.location = "bin"
                it.bin = color
            self.world.looking_at = None

    def ee_pose(self) -> Pose:
        """Top-down flange pose; at a look pose it is right above the zone center."""
        with self.world.lock:
            zone = self.world.looking_at
        if zone is None:
            x, y, z = _HOME_XYZ
        else:
            view = self.world.views[zone]
            (x, y), z = view.center_mm, view.cam_z_mm
        T = np.eye(4)
        T[:3, :3] = np.diag([1.0, -1.0, -1.0])  # z axis (camera optical axis) points down
        T[:3, 3] = (x, y, z)
        return T

    def joints(self) -> tuple[float, ...]:
        with self.world.lock:
            return _JOINTS[self.world.looking_at]

    def hold(self) -> None:
        self._held.set()
        log.warning("sim arm held")

    def recover(self) -> None:
        self._held.clear()
        with self.world.lock:
            self._land_on_background(self.world.at("gripper"))  # open the gripper above the bg
        self.home()
        log.info("sim arm recovered")
