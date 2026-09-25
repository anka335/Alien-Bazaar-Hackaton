"""Color classifier: SAM3 instance masks → items on the background, color class, re-grasp point."""

from __future__ import annotations

import logging
from collections.abc import Callable, Sequence

import cv2
import numpy as np

from sorter.color_classifier.config import ColorClassifierConfig
from sorter.color_classifier.segmenter import Instance
from sorter.core.types import (
    BackgroundResult,
    ColorClass,
    Frame,
    GraspPoint,
    ItemResult,
    Marker,
    Overlay,
    PixelPoint,
)

log = logging.getLogger(__name__)

Segment = Callable[[np.ndarray], list[Instance]]


def roi_mask(shape: tuple[int, int], roi: Sequence[tuple[int, int]]) -> np.ndarray:
    """HxW bool of the ROI polygon; the whole frame when `roi` is empty."""
    if not roi:
        return np.ones(shape, dtype=bool)
    m = np.zeros(shape, np.uint8)
    cv2.fillPoly(m, [np.asarray(roi, np.int32)], 1)
    return m.astype(bool)


def color_stats(color: np.ndarray, mask: np.ndarray, erode_px: int) -> dict[str, float]:
    """Median L* (0..100), a*, b*, chroma over the eroded mask (all of it if erosion empties it)."""
    core = mask
    if erode_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * erode_px + 1, 2 * erode_px + 1))
        eroded = cv2.erode(mask.astype(np.uint8), k).astype(bool)
        if eroded.any():
            core = eroded
    lab = cv2.cvtColor(color, cv2.COLOR_BGR2LAB)[core].astype(np.float32)
    L = float(np.median(lab[:, 0])) * 100 / 255
    a = float(np.median(lab[:, 1])) - 128
    b = float(np.median(lab[:, 2])) - 128
    return {"L": L, "a": a, "b": b, "chroma": float(np.hypot(a, b)), "px": float(core.sum())}


def decide(stats: dict[str, float], cfg: ColorClassifierConfig) -> tuple[ColorClass, float]:
    """Very dark → dark; high chroma → colored; otherwise L* splits light / dark.

    Confidence grows with the distance from the threshold that decided (0.5 on it, 1 a margin away).
    """
    margin = cfg.confidence_margin
    d_dark = cfg.lightness_dark - stats["L"]
    d_chroma = stats["chroma"] - cfg.chroma_colored
    if d_dark > 0:
        return ColorClass.DARK, _conf(d_dark, margin)
    if d_chroma >= 0:
        return ColorClass.COLORED, min(_conf(d_chroma, margin), _conf(-d_dark, margin))
    d_light = stats["L"] - cfg.lightness_light
    conf = min(_conf(-d_chroma, margin), _conf(abs(d_light), margin))
    return (ColorClass.LIGHT if d_light >= 0 else ColorClass.DARK), conf


def _conf(distance: float, margin: float) -> float:
    return float(np.clip(0.5 + 0.5 * distance / margin, 0.5, 1.0))


def regrasp_point(
    mask: np.ndarray,
    depth_mm: np.ndarray | None,
    inset_px: int,
    window_px: int,
    tol_mm: float = 5.0,
) -> GraspPoint | None:
    """The highest cloth point (smallest depth) at least `inset_px` inside the blob.

    Points within `tol_mm` of the highest are equally good; of those, the one deepest inside the
    blob wins, so a flat cloth is grasped in its middle, not at the inset line. Without depth
    (RGB-only camera) it is the point deepest inside the blob, with `depth_mm` None.
    """
    dist = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    if depth_mm is None:
        v, u = np.unravel_index(int(np.argmax(dist)), dist.shape)
        return GraspPoint(PixelPoint(int(u), int(v)), None)
    inner = dist >= min(inset_px, 0.5 * float(dist.max()))
    k = max(1, window_px) | 1
    # a median over the window rejects single-pixel depth spikes; 0 = no data stays out
    valid_depth = np.where(depth_mm > 0, depth_mm, np.iinfo(np.uint16).max).astype(np.uint16)
    smooth = cv2.medianBlur(valid_depth, k) if k <= 5 else valid_depth
    cand = inner & (depth_mm > 0) & (smooth < np.iinfo(np.uint16).max)
    if cand.any():
        top = cand & (smooth <= int(smooth[cand].min()) + tol_mm)
        # the middle of the top region, and away from the blob edge
        dist_top = cv2.distanceTransform(top.astype(np.uint8), cv2.DIST_L2, 5)
        v, u = np.unravel_index(int(np.argmax(np.minimum(dist, dist_top))), dist.shape)
        v, u = int(v), int(u)
    else:  # no depth inside the blob: the deepest-inside pixel, depth from around it
        v, u = np.unravel_index(int(np.argmax(dist)), dist.shape)
        v, u = int(v), int(u)
    depth = _robust_depth(depth_mm, u, v, max(window_px // 2, 1))
    if depth is None:
        return None
    return GraspPoint(PixelPoint(u, v), depth)


def _robust_depth(depth_mm: np.ndarray, u: int, v: int, half: int) -> float | None:
    """Median of the non-zero depths in a window around (u, v), growing it until some exist."""
    h, w = depth_mm.shape
    for r in (half, 2 * half, 4 * half, 8 * half):
        win = depth_mm[max(v - r, 0) : v + r + 1, max(u - r, 0) : u + r + 1]
        vals = win[win > 0]
        if vals.size:
            return float(np.median(vals))
    return None


class Sam3ColorClassifier:
    """`ColorClassifier` on top of a segmentation function (the SAM3 client in production)."""

    def __init__(
        self,
        cfg: ColorClassifierConfig,
        segment: Segment,
        roi: Sequence[tuple[int, int]] = (),
    ):
        self.cfg = cfg
        self.segment = segment
        self.roi = list(roi)

    def blobs(self, frame: Frame) -> list[tuple[np.ndarray, float]]:
        """Instance masks inside the ROI (with depth data, if any), de-duplicated, largest first."""
        shape = frame.color.shape[:2]
        roi = roi_mask(shape, self.roi)
        max_area = self.cfg.max_area_frac * roi.sum()
        seen = np.zeros(shape, dtype=bool)
        out = []
        for inst in sorted(self.segment(frame.color), key=lambda i: i.score, reverse=True):
            m = inst.mask & roi
            if frame.depth_mm is not None:
                # pixels without depth are the gripper fingers (too close) or glare: not cloth
                m &= frame.depth_mm > 0
            area = int(m.sum())
            if area < self.cfg.min_area_px or area > max_area:
                continue
            if (m & seen).sum() > self.cfg.overlap_max * area:
                continue  # a part of an item already taken (a sleeve of the shirt)
            m &= ~seen
            seen |= m
            out.append((m, inst.score))
        out.sort(key=lambda t: int(t[0].sum()), reverse=True)
        return out

    def classify(self, frame: Frame) -> BackgroundResult:
        shape = frame.color.shape[:2]
        roi = roi_mask(shape, self.roi)
        outside = ~roi
        outside[[0, -1], :] = True
        outside[:, [0, -1]] = True
        near_outside = cv2.dilate(outside.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)

        overlay = Overlay(mask=np.zeros(shape, dtype=bool))
        items = []
        for mask, score in self.blobs(frame):
            grasp = regrasp_point(
                mask,
                frame.depth_mm,
                self.cfg.grasp_inset_px,
                self.cfg.depth_window_px,
                self.cfg.grasp_depth_tol_mm,
            )
            if grasp is None:
                log.warning("item of %d px without any depth around it, skipped", mask.sum())
                continue
            stats = color_stats(frame.color, mask, self.cfg.erode_px)
            stats["score"] = score
            color, conf = decide(stats, self.cfg)
            items.append(
                ItemResult(
                    color=color,
                    confidence=conf,
                    grasp=grasp,
                    area_px=int(mask.sum()),
                    touches_roi_edge=bool((mask & near_outside).any()),
                    stats=stats,
                )
            )
            overlay.mask |= mask
            overlay.polygons.append((_outline(mask), color.value))
            overlay.markers.append(Marker(grasp.px, f"{color} {conf:.2f}", "grasp"))
        overlay.text.append(f"{len(items)} item(s) on the background")
        return BackgroundResult(items, overlay)


def _outline(mask: np.ndarray) -> list[PixelPoint]:
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return []
    c = cv2.approxPolyDP(max(contours, key=cv2.contourArea), 2.0, True)
    return [PixelPoint(int(p[0][0]), int(p[0][1])) for p in c]
