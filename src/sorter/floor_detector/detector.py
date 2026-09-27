"""The floor detector (stage A): socks in a frame of the wrist camera looking at the floor.

Segmentation (SAM3 on the rig, the render's segmentation in the simulator) gives the masks; each
mask with depth becomes a sock when its size in mm (pixels scaled by depth) is sock-like. Per
sock: the color from the mask pixels (the color classifier's statistics), the grasp point (the
point deepest inside the mask: the widest part, where the fingers gather the most cloth), the
gripper yaw across the sock there (PCA of the mask around the grasp point, image frame: a sock
bent at the heel has its own direction in the leg and in the foot), and whether the mask is cut
off by the frame or by pixels without depth.

Stateless: frame in, socks out. Best view first: the socks nearest the image center.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import cv2
import numpy as np

from sorter.color_classifier.classifier import (
    Sam3ColorClassifier,
    _outline,
    _robust_depth,
    color_stats,
    decide,
)
from sorter.core.types import FloorResult, Frame, GraspPoint, Marker, Overlay, PixelPoint, Sock
from sorter.floor_detector.config import FloorDetectorConfig


def mask_axes(mask: np.ndarray) -> tuple[float, float, float]:
    """(angle of the long axis in the image, rad; long, short spread in px): PCA of the mask
    pixels. The spreads are 2 standard deviations (a flat strip of length L gives ~0.58 L)."""
    v, u = np.nonzero(mask)
    if len(u) < 3:
        return 0.0, 0.0, 0.0
    pts = np.c_[u, v].astype(np.float64)
    pts -= pts.mean(axis=0)
    cov = pts.T @ pts / max(len(pts) - 1, 1)
    w, vec = np.linalg.eigh(cov)  # ascending
    long = vec[:, 1]
    return float(math.atan2(long[1], long[0])), 2 * math.sqrt(w[1]), 2 * math.sqrt(max(w[0], 0))


class SockDetector:
    def __init__(self, cfg: FloorDetectorConfig, classifier: Sam3ColorClassifier):
        self.cfg = cfg
        self.classifier = classifier  # segmentation + color thresholds (color_classifier)

    def detect(self, frame: Frame, avoid: Sequence[PixelPoint] = ()) -> FloorResult:
        cfg, ccfg = self.cfg, self.classifier.cfg
        h, w = frame.depth_mm.shape
        f = frame.intrinsics.fx
        # where the view is cut off: the frame's edge and pixels without depth (the gripper in
        # front of the lens, too close for the depth camera)
        blind = frame.depth_mm == 0
        blind = cv2.morphologyEx(blind.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        e = cfg.edge_px
        cut = cv2.dilate(blind, np.ones((2 * e + 1, 2 * e + 1), np.uint8)).astype(bool)
        cut[:e, :] = cut[-e:, :] = True
        cut[:, :e] = cut[:, -e:] = True

        overlay = Overlay(mask=np.zeros((h, w), dtype=bool))
        socks: list[Sock] = []
        rejected = 0
        for mask, score in self.classifier.blobs(frame):
            z = float(np.median(frame.depth_mm[mask]))
            mm_per_px = z / f
            area_mm2 = float(mask.sum()) * mm_per_px**2
            angle, long_px, short_px = mask_axes(mask)
            length_mm = long_px * mm_per_px / 0.58
            if not (cfg.min_area_mm2 <= area_mm2 <= cfg.max_area_mm2) or (
                length_mm > cfg.max_length_mm
            ):
                rejected += 1
                overlay.polygons.append((_outline(mask), "not a sock"))
                continue
            grasp = grasp_point(mask, frame.depth_mm, ccfg.depth_window_px)
            if grasp is None:
                continue
            # the sock's direction around the grasp: within its length's reach of the grasp
            r_px = cfg.local_axis_mm / mm_per_px
            yy, xx = np.ogrid[:h, :w]
            near = mask & ((xx - grasp.px.u) ** 2 + (yy - grasp.px.v) ** 2 <= r_px**2)
            local, _, _ = mask_axes(near) if near.sum() > 20 else (angle, 0, 0)
            if any(
                math.hypot(grasp.px.u - a.u, grasp.px.v - a.v) <= cfg.avoid_radius_px for a in avoid
            ):
                overlay.markers.append(Marker(grasp.px, "avoid", "avoid"))
                continue
            stats = color_stats(frame.color, mask, ccfg.erode_px)
            color, conf = decide(stats, ccfg)
            stats.update(
                score=score,
                area_mm2=area_mm2,
                length_mm=length_mm,
                width_mm=short_px * mm_per_px / 0.58,
                depth_mm=z,
            )
            socks.append(
                Sock(
                    color=color,
                    confidence=conf,
                    grasp=grasp,
                    grasp_angle_rad=local + math.pi / 2,  # the fingers close across the sock
                    area_px=int(mask.sum()),
                    touches_roi_edge=bool((mask & cut).any()),
                    mask=mask,
                    stats=stats,
                )
            )
        cx, cy = w / 2, h / 2
        socks.sort(key=lambda s: math.hypot(s.grasp.px.u - cx, s.grasp.px.v - cy))
        for s in socks:
            overlay.mask |= s.mask
            overlay.polygons.append((_outline(s.mask), s.color.value))
            overlay.markers.append(Marker(s.grasp.px, f"{s.color} {s.confidence:.2f}", "grasp"))
            a = s.grasp_angle_rad or 0.0
            d = PixelPoint(int(25 * math.cos(a)), int(25 * math.sin(a)))
            p0, p1 = s.grasp.px, s.grasp.px
            overlay.polygons.append(
                ([PixelPoint(p0.u - d.u, p0.v - d.v), PixelPoint(p1.u + d.u, p1.v + d.v)], "yaw")
            )
        rej = f", {rejected} rejected" if rejected else ""
        overlay.text.append(f"{len(socks)} sock(s){rej}")
        return FloorResult(socks, overlay)


def grasp_point(mask: np.ndarray, depth_mm: np.ndarray, window_px: int) -> GraspPoint | None:
    """The mask pixel deepest inside it (the distance transform's maximum, smoothed so a single
    pixel doesn't decide), with the cloth surface's robust depth around it."""
    dist = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    dist = cv2.GaussianBlur(dist, (0, 0), 3) * mask
    dist[depth_mm == 0] = 0
    if dist.max() <= 0:
        return None
    v, u = np.unravel_index(int(np.argmax(dist)), dist.shape)
    depth = _robust_depth(depth_mm, int(u), int(v), max(window_px // 2, 1))
    return None if depth is None else GraspPoint(PixelPoint(int(u), int(v)), depth)
