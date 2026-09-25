"""SimArm: implements ArmController on the SimWorld."""

from __future__ import annotations

import logging
import math
import threading

import numpy as np

from sorter.core.errors import EStopped, TargetRejected
from sorter.core.types import ArmPoint, ColorClass, PickResult, Pose, Zone
from sorter.sim.world import (
    BG_ITEM_HEIGHT_MM,
    BIN_FLOOR_Z_MM,
    HOME_CAM,
    CamPose,
    SimItem,
    SimWorld,
)

log = logging.getLogger(__name__)

_ABOVE_MM = 260.0  # camera above the target before and after a pick
_AT_GRASP_MM = 110.0  # camera above the target at the grasp (it sits behind the fingers)
_RELEASE_MM = 300.0  # camera above the surface where the gripper opens
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

    def _move(self, *waypoints: CamPose) -> None:
        """One blocking motion through `waypoints`, `sim.motion_s` per segment. A hold during
        the motion interrupts it and freezes the camera where it is."""
        self._check_held()
        seg_s = self.world.cfg.motion_s
        with self.world.lock:
            self.world.camera.move(list(waypoints), seg_s)
        if seg_s > 0 and self._held.wait(seg_s * len(waypoints)):
            with self.world.lock:
                self.world.camera.freeze()
            raise EStopped("arm held during motion")
        self._check_held()

    def _land_on_background(self, items: list[SimItem], first: tuple[float, float] | None = None):
        for i, it in enumerate(items):
            it.x, it.y = first if i == 0 and first else self.world.free_point_on_background()
            it.height_mm = BG_ITEM_HEIGHT_MM
            it.location = "background"

    # --- ArmController ---

    def start(self) -> None:
        with self.world.lock:
            if self.world.items and all(it.location == "bin" for it in self.world.items):
                self.world.fill_box()  # all sorted: tip the bins back into the box for a new run
                log.info("sim: bins emptied back into the box")
        log.info("sim arm started")

    def shutdown(self) -> None:
        with self.world.lock:
            self.world.looking_at = None
            self.world.camera.move([CamPose(*HOME_CAM)], 0)
        log.info("sim arm at rest, motors off")

    def home(self) -> None:
        with self.world.lock:
            self.world.looking_at = None
        self._move(CamPose(*HOME_CAM))

    def look(self, zone: Zone) -> None:
        self._check_held()
        if self.world.looking_at == zone:
            return
        with self.world.lock:
            self.world.looking_at = None
        self._move(self.world.look_pose(zone))
        with self.world.lock:
            self.world.looking_at = zone

    def pick(self, target: ArmPoint, zone: Zone) -> PickResult:
        self._check_held()
        cfg = self.world.cfg
        if not self.world.views[zone].contains(target.x, target.y):
            where = f"({target.x:.0f}, {target.y:.0f})"
            raise TargetRejected(f"{where} is outside the {zone} workspace")
        s = self.world.views[zone].surface_z_mm
        with self.world.lock:
            self.world.looking_at = None
        self._move(
            CamPose(target.x, target.y, target.z + _ABOVE_MM, s),
            CamPose(target.x, target.y, target.z + _AT_GRASP_MM, s),
        )
        with self.world.lock:
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
        self._move(CamPose(target.x, target.y, target.z + _ABOVE_MM, s))
        if grabbed:
            log.info("sim pick from %s: %s", zone, [it.id for it in grabbed])
        return PickResult(gripper_opening=0.3 if grabbed else 0.0, likely_empty=not grabbed)

    def place_on_background(self) -> None:
        with self.world.lock:
            self.world.looking_at = None
            x, y = self.world.free_point_on_background()
        s = self.world.views[Zone.BACKGROUND].surface_z_mm
        self._move(CamPose(x, y, s + _RELEASE_MM, s))
        with self.world.lock:
            self._land_on_background(self.world.at("gripper"), first=(x, y))

    def drop_to_bin(self, color: ColorClass) -> None:
        with self.world.lock:
            self.world.looking_at = None
        x, y = self.world.bin_xy[color]
        self._move(CamPose(x, y, BIN_FLOOR_Z_MM + _RELEASE_MM + 150, BIN_FLOOR_Z_MM))
        with self.world.lock:
            for it in self.world.at("gripper"):
                it.location = "bin"
                it.bin = color

    def ee_pose(self) -> Pose:
        """Top-down flange pose; at a look pose it is right above the zone center."""
        with self.world.lock:
            zone = self.world.looking_at
        if zone is None:
            x, y, z = HOME_CAM
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
