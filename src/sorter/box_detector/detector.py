"""DepthBoxDetector: grasp point in a box from the depth image (classic CV + depth).

From a fixed look pose the box floor is at a near-constant depth, so cloth is whatever stands
higher than the floor (walls and dividers too: give it one compartment as the ROI). The grasp
point is the top of the pile, inside a cloth region, at least `wall_margin_mm` inside the ROI,
and away from the failed grasps in `avoid`.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import cv2
import numpy as np

from sorter.box_detector.config import BoxDetectorConfig
from sorter.core.types import BoxResult, BoxStatus, Frame, GraspPoint, Marker, Overlay, PixelPoint


def roi_mask(shape: tuple[int, int], roi: Sequence[tuple[int, int]]) -> np.ndarray:
    """bool mask of the ROI polygon; the whole frame if there is no ROI."""
    if len(roi) < 3:
        return np.ones(shape, bool)
    m = np.zeros(shape, np.uint8)
    cv2.fillPoly(m, [np.array(roi, np.int32)], 1)
    return m.astype(bool)


def _erode(mask: np.ndarray, px: int) -> np.ndarray:
    if px <= 0:
        return mask
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * px + 1, 2 * px + 1))
    return cv2.erode(mask.astype(np.uint8), k, borderType=cv2.BORDER_CONSTANT, borderValue=0) > 0


def robust_depth(depth_mm: np.ndarray, u: int, v: int, half: int) -> float | None:
    win = depth_mm[max(v - half, 0) : v + half + 1, max(u - half, 0) : u + half + 1]
    win = win[win > 0]
    return float(np.median(win)) if win.size else None


class DepthBoxDetector:
    def __init__(self, cfg: BoxDetectorConfig, roi: Sequence[tuple[int, int]] = ()):
        self.cfg = cfg
        self.roi = [tuple(p) for p in roi]

    def detect(self, frame: Frame, avoid: Sequence[PixelPoint] = ()) -> BoxResult:
        cfg = self.cfg
        depth = frame.depth_mm.astype(np.float32)
        h, w = depth.shape
        roi = roi_mask((h, w), self.roi)
        valid = roi & (depth > 0)
        overlay = Overlay(markers=[Marker(a, "avoid", "avoid") for a in avoid])
        if self.roi:
            overlay.polygons.append(([PixelPoint(int(u), int(v)) for u, v in self.roi], "box"))
        if not valid.any():
            overlay.text.append("no depth in the box ROI")
            return BoxResult(BoxStatus.EMPTY, None, 0.0, overlay)

        floor = cfg.floor_depth_mm or float(np.percentile(depth[valid], cfg.floor_percentile))
        height = np.where(valid, floor - depth, 0.0)
        cloth = valid & (height > cfg.cloth_height_mm)
        # drop specks: keep regions of a reasonable size
        n, labels, stats, _ = cv2.connectedComponentsWithStats(cloth.astype(np.uint8))
        keep = np.zeros(n, bool)
        keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= cfg.min_cloth_px // 5
        cloth = keep[labels]
        coverage = float(cloth.sum() / max(roi.sum(), 1))
        overlay.mask = cloth
        overlay.text.append(f"cloth {coverage:.0%} of the box, floor at {floor:.0f} mm")
        if cloth.sum() < cfg.min_cloth_px:
            return BoxResult(BoxStatus.EMPTY, None, coverage, overlay)

        smooth = cv2.GaussianBlur(height.astype(np.float32), (0, 0), cfg.smooth_px)
        margin_px = round(cfg.wall_margin_mm * frame.intrinsics.fx / floor)
        allowed = _erode(roi, margin_px) & _erode(cloth, cfg.inset_px) & valid
        if avoid:
            yy, xx = np.mgrid[0:h, 0:w]
            for a in avoid:
                allowed &= (xx - a.u) ** 2 + (yy - a.v) ** 2 >= cfg.avoid_radius_px**2
        if not allowed.any():
            overlay.text.append("cloth, but no grasp point left")
            return BoxResult(BoxStatus.NO_GRASP, None, coverage, overlay)

        score = np.where(allowed, smooth, -np.inf)
        for _ in range(3):  # a few runners-up, for the overlay
            v, u = np.unravel_index(int(np.argmax(score)), score.shape)
            if not math.isfinite(score[v, u]):
                break
            overlay.markers.append(
                Marker(PixelPoint(int(u), int(v)), f"{smooth[v, u]:.0f} mm", "candidate")
            )
            cv2.circle(score, (int(u), int(v)), 40, -np.inf, -1)  # type: ignore[arg-type]
        best = next(m for m in overlay.markers if m.kind == "candidate")
        d = robust_depth(frame.depth_mm, best.px.u, best.px.v, cfg.depth_window_px)
        if d is None:
            return BoxResult(BoxStatus.NO_GRASP, None, coverage, overlay)
        grasp = GraspPoint(best.px, d)
        overlay.markers.append(Marker(best.px, f"grasp {d:.0f} mm", "grasp"))
        return BoxResult(BoxStatus.GRASP, grasp, coverage, overlay)
