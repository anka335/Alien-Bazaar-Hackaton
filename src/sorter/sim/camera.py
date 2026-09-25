"""SimCamera: renders the zone the arm looks at, colored discs on a flat surface, plus depth."""

from __future__ import annotations

import itertools
import threading
import time

import cv2
import numpy as np

from sorter.core.types import Frame, Zone
from sorter.sim.world import SimWorld

_SURFACE_BGR = {Zone.BOX: (70, 100, 140), Zone.BACKGROUND: (128, 128, 128)}
_NO_ZONE_BGR = (60, 60, 60)  # between look poses: nothing useful in view


class SimCamera:
    def __init__(self, world: SimWorld):
        self.world = world
        self._seq = itertools.count()
        self._lock = threading.Lock()

    def start(self) -> None:
        pass

    def close(self) -> None:
        pass

    def latest(self) -> Frame | None:
        return self._render()

    def fresh(self, timeout_s: float = 2.0) -> Frame:
        return self._render()

    def _render(self) -> Frame:
        cfg = self.world.cfg
        color = np.empty((cfg.height, cfg.width, 3), np.uint8)
        depth = np.zeros((cfg.height, cfg.width), np.uint16)
        with self.world.lock:
            zone = self.world.looking_at
            view = self.world.views[zone or Zone.BACKGROUND]
            if zone is None:
                color[:] = _NO_ZONE_BGR
            else:
                color[:] = _SURFACE_BGR[zone]
                depth[:] = round(view.cam_height_mm)
                r = round(cfg.item_radius_mm / view.mm_per_px)
                for it in sorted(self.world.at(zone.value), key=lambda it: it.height_mm):
                    u, v = view.to_px(it.x, it.y)
                    center = (round(u), round(v))
                    cv2.circle(color, center, r, it.bgr, -1, cv2.LINE_AA)
                    cv2.circle(depth, center, r, round(view.cam_height_mm - it.height_mm), -1)
        with self._lock:
            seq = next(self._seq)
        return Frame(
            color=color,
            depth_mm=depth,
            intrinsics=view.intrinsics(),
            timestamp=time.monotonic(),
            seq=seq,
        )
