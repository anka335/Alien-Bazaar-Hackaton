"""SimWorld: items, zones, bins, and where the arm is looking. Shared by all sim components."""

from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass
from typing import Literal

from sorter.core.types import ColorClass, Intrinsics, Zone
from sorter.sim.config import SimConfig, SimZoneConfig

Location = Literal["box", "background", "gripper", "bin"]

_PALETTE: dict[ColorClass, list[tuple[int, int, int]]] = {  # BGR
    # white, cream, light gray, pale blue
    ColorClass.LIGHT: [(238, 238, 236), (208, 228, 240), (206, 206, 204), (236, 222, 206)],
    # black, navy, charcoal, dark brown
    ColorClass.DARK: [(32, 30, 30), (72, 38, 20), (48, 46, 46), (30, 40, 62)],
    # red, blue, green, yellow, orange, purple, pink
    ColorClass.COLORED: [
        (48, 44, 206),
        (196, 104, 34),
        (70, 160, 48),
        (40, 196, 236),
        (32, 124, 238),
        (150, 62, 118),
        (170, 120, 236),
    ],
}

BG_ITEM_HEIGHT_MM = 25.0  # an item lying on the background
BIN_SIZE_MM = 170.0
BIN_FLOOR_Z_MM = 15.0
HOME_CAM = (300.0, 0.0, 450.0)  # camera x, y, z at the home pose, mm


@dataclass(frozen=True)
class CamPose:
    """Where the wrist camera is, looking straight down. `ref_z_mm` is the surface whose scale
    the image follows (the zone surface at a look pose), so look-pose images match `ZoneView`."""

    x: float
    y: float
    z: float
    ref_z_mm: float = 0.0

    def lerp(self, other: CamPose, t: float) -> CamPose:
        return CamPose(
            self.x + (other.x - self.x) * t,
            self.y + (other.y - self.y) * t,
            self.z + (other.z - self.z) * t,
            self.ref_z_mm + (other.ref_z_mm - self.ref_z_mm) * t,
        )


class CamPath:
    """The camera's motion: straight segments between waypoints, eased, one per `seg_s`."""

    def __init__(self, pose: CamPose):
        self._start = pose
        self._t0 = 0.0
        self._seg_s = 0.0
        self._waypoints: list[CamPose] = []

    def pose(self, now: float | None = None) -> CamPose:
        if not self._waypoints:
            return self._start
        now = time.monotonic() if now is None else now
        if self._seg_s <= 0:
            return self._waypoints[-1]
        k, frac = divmod(max(now - self._t0, 0.0) / self._seg_s, 1.0)
        if k >= len(self._waypoints):
            return self._waypoints[-1]
        a = self._start if k == 0 else self._waypoints[int(k) - 1]
        return a.lerp(self._waypoints[int(k)], frac * frac * (3 - 2 * frac))  # smoothstep

    def move(self, waypoints: list[CamPose], seg_s: float) -> None:
        now = time.monotonic()
        self._start, self._t0, self._seg_s = self.pose(now), now, seg_s
        self._waypoints = list(waypoints)

    def freeze(self) -> None:
        self._start, self._waypoints = self.pose(), []


@dataclass(frozen=True)
class ZoneView:
    """Linear mapping between the image of a zone (from its look pose) and the arm frame."""

    zone: Zone
    center_mm: tuple[float, float]
    surface_z_mm: float
    mm_per_px: float
    width: int
    height: int
    cam_height_mm: float

    @classmethod
    def from_config(cls, zone: Zone, zc: SimZoneConfig, cfg: SimConfig) -> ZoneView:
        return cls(
            zone=zone,
            center_mm=zc.center_mm,
            surface_z_mm=zc.surface_z_mm,
            mm_per_px=zc.width_mm / cfg.width,
            width=cfg.width,
            height=cfg.height,
            cam_height_mm=cfg.cam_height_mm,
        )

    @property
    def cam_z_mm(self) -> float:
        return self.surface_z_mm + self.cam_height_mm

    def to_px(self, x: float, y: float) -> tuple[float, float]:
        return (
            (x - self.center_mm[0]) / self.mm_per_px + self.width / 2,
            (y - self.center_mm[1]) / self.mm_per_px + self.height / 2,
        )

    def to_xy(self, u: float, v: float) -> tuple[float, float]:
        return (
            (u - self.width / 2) * self.mm_per_px + self.center_mm[0],
            (v - self.height / 2) * self.mm_per_px + self.center_mm[1],
        )

    def contains(self, x: float, y: float, margin_mm: float = 0.0) -> bool:
        u, v = self.to_px(x, y)
        m = margin_mm / self.mm_per_px
        return m <= u <= self.width - m and m <= v <= self.height - m

    def intrinsics(self) -> Intrinsics:
        # a pinhole consistent with the linear mapping at the zone surface
        f = self.cam_height_mm / self.mm_per_px
        return Intrinsics(
            fx=f, fy=f, cx=self.width / 2, cy=self.height / 2, width=self.width, height=self.height
        )


@dataclass
class SimItem:
    id: int
    color: ColorClass
    bgr: tuple[int, int, int]
    x: float  # mm, arm frame
    y: float
    height_mm: float  # top surface above the zone surface
    location: Location
    bin: ColorClass | None = None


class SimWorld:
    def __init__(self, cfg: SimConfig):
        self.cfg = cfg
        self.rng = random.Random(cfg.seed)
        self.lock = threading.RLock()
        self.views = {z: ZoneView.from_config(z, zc, cfg) for z, zc in cfg.zones.items()}
        self.looking_at: Zone | None = None  # the zone whose look pose the arm is at
        self.camera = CamPath(CamPose(*HOME_CAM))
        self.bin_xy = self._bin_layout()
        self.items = [
            SimItem(i, color, self.rng.choice(_PALETTE[color]), 0.0, 0.0, 0.0, "box")
            for i, color in enumerate(cfg.items)
        ]
        self.fill_box()

    def fill_box(self) -> None:
        """Put every item in the box, at random places, stacked in id order."""
        with self.lock:
            for i, it in enumerate(self.items):
                it.x, it.y = self.random_point(Zone.BOX, margin_mm=self.cfg.item_radius_mm)
                it.height_mm = 20.0 + 15.0 * i
                it.location, it.bin = "box", None

    def at(self, location: Location) -> list[SimItem]:
        with self.lock:
            return [it for it in self.items if it.location == location]

    def random_point(self, zone: Zone, margin_mm: float) -> tuple[float, float]:
        view = self.views[zone]
        u = self.rng.uniform(0, view.width)
        v = self.rng.uniform(0, view.height)
        x, y = view.to_xy(u, v)
        cx, cy = view.center_mm
        half_w = view.width * view.mm_per_px / 2 - margin_mm
        half_h = view.height * view.mm_per_px / 2 - margin_mm
        return (min(max(x, cx - half_w), cx + half_w), min(max(y, cy - half_h), cy + half_h))

    def free_point_on_background(self) -> tuple[float, float]:
        """A point where an item lands without touching the items already on the background."""
        r = self.cfg.item_radius_mm
        others = self.at("background")
        x, y = self.random_point(Zone.BACKGROUND, margin_mm=r)
        for _ in range(100):
            if all(math.hypot(x - o.x, y - o.y) > 2.5 * r for o in others):
                break
            x, y = self.random_point(Zone.BACKGROUND, margin_mm=r)
        return x, y

    def look_pose(self, zone: Zone) -> CamPose:
        view = self.views[zone]
        return CamPose(*view.center_mm, view.cam_z_mm, view.surface_z_mm)

    def _bin_layout(self) -> dict[ColorClass, tuple[float, float]]:
        """Three bins in a column beyond the far edge of the zones."""
        x0, x1, y0, y1 = self.zones_extent()
        x = x1 + 60 + BIN_SIZE_MM / 2
        yc, step = (y0 + y1) / 2, BIN_SIZE_MM + 30
        return {c: (x, yc + (i - 1) * step) for i, c in enumerate(ColorClass)}

    def zones_extent(self) -> tuple[float, float, float, float]:
        """x0, x1, y0, y1 of everything the zone views cover, mm."""
        xs, ys = [], []
        for v in self.views.values():
            for u, w in ((0, 0), (v.width, v.height)):
                x, y = v.to_xy(u, w)
                xs.append(x)
                ys.append(y)
        return min(xs), max(xs), min(ys), max(ys)

    def bins(self) -> dict[ColorClass, list[SimItem]]:
        with self.lock:
            return {c: [it for it in self.items if it.bin is c] for c in ColorClass}
