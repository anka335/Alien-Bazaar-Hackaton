"""Sim vision: reads the SimWorld directly, returns pixels like the real detectors."""

from __future__ import annotations

import math
import time
from collections.abc import Sequence

from sorter.core.types import (
    BackgroundResult,
    BoxResult,
    BoxStatus,
    Frame,
    GraspPoint,
    ItemResult,
    Marker,
    Overlay,
    PixelPoint,
    Zone,
)
from sorter.sim.world import SimItem, SimWorld, ZoneView


def _grasp(view: ZoneView, frame: Frame, it: SimItem) -> GraspPoint:
    u, v = view.to_px(it.x, it.y)
    px = PixelPoint(min(max(round(u), 0), view.width - 1), min(max(round(v), 0), view.height - 1))
    depth = float(frame.depth_mm[px.v, px.u]) or view.cam_z_mm - view.surface_z_mm - it.height_mm
    return GraspPoint(px, depth)


class SimBoxDetector:
    def __init__(self, world: SimWorld, avoid_radius_px: int):
        self.world = world
        self.avoid_radius_px = avoid_radius_px

    def detect(self, frame: Frame, avoid: Sequence[PixelPoint] = ()) -> BoxResult:
        time.sleep(self.world.cfg.vision_s)
        view = self.world.views[Zone.BOX]
        items = sorted(self.world.at("box"), key=lambda it: it.height_mm, reverse=True)
        overlay = Overlay(markers=[Marker(a, "avoid", "avoid") for a in avoid])
        r_px = self.world.cfg.item_radius_mm / view.mm_per_px
        coverage = min(1.0, len(items) * math.pi * r_px**2 / (view.width * view.height))
        if not items:
            return BoxResult(BoxStatus.EMPTY, None, coverage, overlay)
        for it in items:  # highest first
            g = _grasp(view, frame, it)
            if any(math.dist((g.px.u, g.px.v), (a.u, a.v)) < self.avoid_radius_px for a in avoid):
                continue
            overlay.markers.append(Marker(g.px, f"grasp {g.depth_mm:.0f} mm", "grasp"))
            return BoxResult(BoxStatus.GRASP, g, coverage, overlay)
        return BoxResult(BoxStatus.NO_GRASP, None, coverage, overlay)


class SimColorClassifier:
    def __init__(self, world: SimWorld):
        self.world = world

    def classify(self, frame: Frame) -> BackgroundResult:
        time.sleep(self.world.cfg.vision_s)
        view = self.world.views[Zone.BACKGROUND]
        r_px = self.world.cfg.item_radius_mm / view.mm_per_px
        overlay = Overlay()
        items = []
        for it in self.world.at("background"):
            g = _grasp(view, frame, it)
            touches = not view.contains(it.x, it.y, margin_mm=self.world.cfg.item_radius_mm)
            items.append(
                ItemResult(
                    color=it.color,
                    confidence=1.0,
                    grasp=g,
                    area_px=round(math.pi * r_px**2),
                    touches_roi_edge=touches,
                    stats={},
                )
            )
            overlay.markers.append(Marker(g.px, it.color.value, "grasp"))
        items.sort(key=lambda r: r.area_px, reverse=True)
        return BackgroundResult(items, overlay)
