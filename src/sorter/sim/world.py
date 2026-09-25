"""SimWorld: items, zones, bins, and where the arm is looking. Shared by all sim components."""

from __future__ import annotations

import math
import random
import threading
from dataclasses import dataclass
from typing import Literal

from sorter.core.types import ColorClass, Intrinsics, Zone
from sorter.sim.config import SimConfig, SimZoneConfig

Location = Literal["box", "background", "gripper", "bin"]

_PALETTE: dict[ColorClass, list[tuple[int, int, int]]] = {  # BGR
    ColorClass.LIGHT: [(235, 235, 235), (215, 225, 230), (225, 230, 220)],
    ColorClass.DARK: [(30, 30, 30), (55, 35, 25), (25, 25, 45)],
    ColorClass.COLORED: [(40, 40, 200), (190, 90, 30), (40, 160, 40), (30, 200, 220)],
}

BG_ITEM_HEIGHT_MM = 25.0  # an item lying on the background


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
        self.items: list[SimItem] = []
        for i, color in enumerate(cfg.items):
            x, y = self.random_point(Zone.BOX, margin_mm=cfg.item_radius_mm)
            self.items.append(
                SimItem(
                    i,
                    color,
                    self.rng.choice(_PALETTE[color]),
                    x,
                    y,
                    height_mm=20.0 + 15.0 * i,
                    location="box",
                )
            )

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

    def bins(self) -> dict[ColorClass, list[SimItem]]:
        with self.lock:
            return {c: [it for it in self.items if it.bin is c] for c in ColorClass}
