"""Stage 0's floor detector: block 4's color classifier (segmentation + color stats) over the
floor view, its items taken as socks. A baseline for stage A to replace (A2)."""

from __future__ import annotations

import math
from collections.abc import Sequence

from sorter.core.protocols import ColorClassifier
from sorter.core.types import FloorResult, Frame, PixelPoint, Sock


class ClassifierFloorDetector:
    def __init__(self, classifier: ColorClassifier, avoid_radius_px: float = 30.0):
        self.classifier = classifier
        self.avoid_radius_px = avoid_radius_px

    def detect(self, frame: Frame, avoid: Sequence[PixelPoint] = ()) -> FloorResult:
        bg = self.classifier.classify(frame)
        socks = [
            Sock(
                color=it.color,
                confidence=it.confidence,
                grasp=it.grasp,
                grasp_angle_rad=None,
                area_px=it.area_px,
                touches_roi_edge=it.touches_roi_edge,
                mask=None,
                stats=it.stats,
            )
            for it in bg.items
            if all(
                math.hypot(it.grasp.px.u - a.u, it.grasp.px.v - a.v) > self.avoid_radius_px
                for a in avoid
            )
        ]
        return FloorResult(socks, bg.overlay)
