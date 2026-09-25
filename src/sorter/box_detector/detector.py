"""Box detector (D-015): cloth masks from the SAM3 service, grasp at the middle of the cloth.

No depth (D-014), so "the top of the pile" is not observable. The grasp point is the cloth pixel
farthest from any cloth edge: the fingers close on cloth there, whatever the pile looks like.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence

import cv2
import numpy as np

from sorter.box_detector.config import BoxDetectorConfig
from sorter.color_classifier.classifier import roi_mask
from sorter.color_classifier.segmenter import Instance
from sorter.core.types import (
    BoxResult,
    BoxStatus,
    Frame,
    GraspPoint,
    Marker,
    Overlay,
    PixelPoint,
)

Segment = Callable[[np.ndarray], list[Instance]]


class SamBoxDetector:
    """`BoxDetector` on top of a segmentation function (the SAM3 client in production)."""

    def __init__(self, cfg: BoxDetectorConfig, segment: Segment, roi: Sequence[tuple[int, int]]):
        self.cfg = cfg
        self.segment = segment
        self.roi = list(roi)

    def detect(self, frame: Frame, avoid: Sequence[PixelPoint] = ()) -> BoxResult:
        shape = frame.color.shape[:2]
        roi = roi_mask(shape, self.roi)
        cloth = np.zeros(shape, dtype=bool)
        for inst in self.segment(frame.color):
            m = inst.mask & roi
            if m.sum() >= self.cfg.min_area_px:
                cloth |= m
        coverage = float(cloth.sum()) / max(int(roi.sum()), 1)
        overlay = Overlay(mask=cloth)
        for p in avoid:
            overlay.markers.append(Marker(p, "avoid", "avoid"))
        overlay.text.append(f"cloth {coverage:.0%} of the box")
        if coverage < self.cfg.empty_coverage:
            return BoxResult(BoxStatus.EMPTY, None, coverage, overlay)

        # distance to the nearest non-cloth pixel, and to the ROI edge (walls)
        dist = cv2.distanceTransform(cloth.astype(np.uint8), cv2.DIST_L2, 5)
        border = np.pad(roi, 1, constant_values=False).astype(np.uint8)
        wall = cv2.distanceTransform(border, cv2.DIST_L2, 5)[1:-1, 1:-1]
        allowed = (wall >= self.cfg.wall_margin_px) & (dist >= self.cfg.min_inset_px)
        vv, uu = np.mgrid[: shape[0], : shape[1]]
        r2 = self.cfg.avoid_radius_px**2
        for p in avoid:
            allowed &= (uu - p.u) ** 2 + (vv - p.v) ** 2 > r2
        if not allowed.any():
            overlay.text.append("no grasp candidate left")
            return BoxResult(BoxStatus.NO_GRASP, None, coverage, overlay)
        score = np.where(allowed, dist, -1.0)
        # the ridge of equally thick cloth: its pixel nearest to the ridge's centroid
        top_v, top_u = np.nonzero(score >= score.max() - 1.0)
        i = int(np.argmin((top_u - top_u.mean()) ** 2 + (top_v - top_v.mean()) ** 2))
        v, u = int(top_v[i]), int(top_u[i])
        grasp = GraspPoint(PixelPoint(int(u), int(v)), None)
        overlay.markers.append(Marker(grasp.px, f"grasp {dist[v, u]:.0f}px in", "grasp"))
        return BoxResult(BoxStatus.GRASP, grasp, coverage, overlay)
