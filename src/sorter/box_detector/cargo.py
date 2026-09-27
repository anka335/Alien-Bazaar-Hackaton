"""The next sock to take out of the cargo box: which one, its color, where and how to grasp it.

From the cargo look pose every pixel is turned into an arm-frame point (depth + the camera
pose), so "in the box" and "cloth" are geometry, not image heuristics: cloth is what stands more
than `cloth_mm` above the box floor inside its walls. Socks are the segmentation's instances
(SAM3 on the rig, the render's segmentation in the sim) inside the box; without instances the
cloth's connected regions stand in. The sock taken is the one on top of the pile (the highest
cloth), its color from its own pixels (block 4's statistics), the grasp at its highest point
inside the pick workspace, away from failed grasps, with the fingers straddling the sock.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

import cv2
import numpy as np

from sorter.box_detector.geometry import in_polygon, points, polygon_mask
from sorter.color_classifier.classifier import color_stats, decide
from sorter.color_classifier.config import ColorClassifierConfig
from sorter.color_classifier.segmenter import Instance
from sorter.core.types import (
    ArmPoint,
    ColorClass,
    GraspPoint,
    Marker,
    Observation,
    Overlay,
    PixelPoint,
)
from sorter.sim.config import RectConfig

Segment = Callable[[np.ndarray], list[Instance]]

CLOTH_MM = 5.0  # above the box floor: cloth
MIN_SOCK_PX = 250  # a visible part of a sock smaller than this is not worth a grasp
INSET_PX = 6  # the grasp stays this far inside the sock's visible region
TOP_TOL_MM = 4.0  # points this close to the highest count as the top
AVOID_MM = 18.0  # a failed grasp rules out its surroundings
EMPTY_PX = 150  # less cloth than this in the box: empty
FINGER_PAD_MM = 12.0  # an open finger's tip covers this radius


@dataclass
class SockTarget:
    color: ColorClass
    confidence: float
    grasp: GraspPoint  # pixel + depth, for the calibration
    point: ArmPoint  # the same, arm frame (pinhole estimate)
    yaws: list[float]  # finger directions to try, best first (rad, from +x)
    height_mm: float  # the grasp above the box floor
    area_px: int
    stats: dict[str, float] = field(default_factory=dict)


@dataclass
class SockSeen:
    """A sock visible in the box: its color (lit from above: the reliable view of it)."""

    color: ColorClass
    confidence: float
    lab: tuple[float, float, float]  # median L*, a*, b*
    xy: tuple[float, float]  # centroid of its visible part, mm
    area_px: int
    mask: np.ndarray | None = field(default=None, repr=False, compare=False)  # its pixels


@dataclass
class CargoView:
    target: SockTarget | None  # None: nothing graspable
    cloth_px: int  # cloth in the box
    seen: list[SockSeen]  # every visible sock
    overlay: Overlay

    @property
    def socks(self) -> int:
        return len(self.seen)

    @property
    def empty(self) -> bool:
        return self.cloth_px < EMPTY_PX


def _yaws(xy: np.ndarray) -> list[float]:
    """Finger directions: across the sock's long axis first (the fingers straddle it), then
    along it, the diagonals, and along the box's walls: near a wall only some fit."""
    if len(xy) >= 5:
        c = np.cov((xy - xy.mean(axis=0)).T)
        w, v = np.linalg.eigh(c)
        long = math.atan2(v[1, 1], v[0, 1])
    else:
        long = 0.0
    # and square to the box: along a wall is how a sock against it can be grasped
    out = [long + math.pi / 2, long, long + math.pi / 4, long - math.pi / 4, 0.0, math.pi / 2]
    return [math.remainder(a, math.pi) for a in out]


def _clear_yaws(
    yaws: list[float], at: tuple[float, float], finger_mm: float, others: np.ndarray
) -> list[float]:
    """`yaws` with those whose open fingers come down on other socks last: pressed into a
    neighbor, the fingers drag it out of the box. `others`: Nx2 points of the other socks."""
    if len(others) == 0:
        return yaws

    def on_others(yaw: float) -> int:
        d = np.array([math.cos(yaw), math.sin(yaw)]) * finger_mm
        tips = (np.asarray(at) + d, np.asarray(at) - d)
        return sum(int((np.hypot(*(others - t).T) < FINGER_PAD_MM).sum()) for t in tips)

    return sorted(yaws, key=on_others)  # stable: the preference stays among the clear ones


def find_sock(
    obs: Observation,
    box: RectConfig,
    floor_z_mm: float,
    rim_z_mm: float,
    workspace: Sequence[tuple[float, float]],
    classifier: ColorClassifierConfig,
    segment: Segment | None = None,
    avoid: Sequence[ArmPoint] = (),
    finger_mm: float = 50.0,
) -> CargoView:
    p = points(obs)
    x, y, z = p[..., 0], p[..., 1], p[..., 2]
    x0, x1, y0, y1 = box.bounds(-2.0)
    with np.errstate(invalid="ignore"):
        inside = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1) & (z < rim_z_mm + 80)
        height = np.where(inside, z - floor_z_mm, np.nan)
        cloth = inside & (height > CLOTH_MM)
    overlay = Overlay(mask=cloth)
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    overlay.polygons.append((_outline(polygon_mask(obs, corners, floor_z_mm)), "box"))
    n_cloth = int(cloth.sum())
    if n_cloth < EMPTY_PX:
        overlay.text.append(f"box empty ({n_cloth} px of cloth)")
        return CargoView(None, n_cloth, [], overlay)

    if segment is not None:
        masks = [i.mask & cloth for i in segment(obs.frame.color)]
    else:  # the cloth's connected regions
        n, labels = cv2.connectedComponents(cloth.astype(np.uint8))
        masks = [labels == k for k in range(1, n)]
    masks = [m for m in masks if m.sum() >= MIN_SOCK_PX]
    overlay.text.append(f"{len(masks)} sock(s), {n_cloth} px of cloth")
    seen = []
    for m in masks:
        st = color_stats(obs.frame.color, m, classifier.erode_px)
        c, conf = decide(st, classifier)
        xy = (float(np.nanmean(x[m])), float(np.nanmean(y[m])))
        seen.append(SockSeen(c, conf, (st["L"], st["a"], st["b"]), xy, int(m.sum()), m))

    # inside the workspace, a few pixels in: the calibration's grasp point is a bit off this one
    reach = _erode(in_polygon(x, y, workspace), 4)
    with np.errstate(invalid="ignore"):
        for a in avoid:
            reach &= np.hypot(x - a.x, y - a.y) > AVOID_MM
            px = _pixel_of(obs, a)
            if px is not None:
                overlay.markers.append(Marker(px, "avoid", "avoid"))
    # the top of the pile first: a sock's highest visible cloth
    order = sorted(masks, key=lambda m: -float(np.nanpercentile(height[m], 90)))
    for m in order:
        others = cloth & ~m  # the other socks' cloth: the fingers had better not land on it
        core = _erode(m, INSET_PX)
        allowed = (core if core.any() else m) & reach
        if not allowed.any():
            continue
        h = cv2.GaussianBlur(np.nan_to_num(height, nan=0.0).astype(np.float32), (0, 0), 2.0)
        top = allowed & (h >= h[allowed].max() - TOP_TOL_MM)
        dist = cv2.distanceTransform(m.astype(np.uint8), cv2.DIST_L2, 5)
        v, u = np.unravel_index(int(np.argmax(np.where(top, dist, -1))), dist.shape)
        stats = color_stats(obs.frame.color, m, classifier.erode_px)
        color, conf = decide(stats, classifier)
        depth = float(np.median(_window(obs.frame.depth_mm, int(u), int(v), 3)))
        pt = p[v, u]
        target = SockTarget(
            color=color,
            confidence=conf,
            grasp=GraspPoint(PixelPoint(int(u), int(v)), depth),
            point=ArmPoint(float(pt[0]), float(pt[1]), float(pt[2])),
            yaws=_clear_yaws(
                _yaws(np.stack([x[m], y[m]], axis=1)),
                (float(pt[0]), float(pt[1])),
                finger_mm,
                np.stack([x[others], y[others]], axis=1),
            ),
            height_mm=float(height[v, u]),
            area_px=int(m.sum()),
            stats=stats,
        )
        overlay.polygons.append((_outline(m), color.value))
        overlay.markers.append(Marker(target.grasp.px, f"{color} {conf:.2f}", "grasp"))
        return CargoView(target, n_cloth, seen, overlay)
    overlay.text.append("cloth, but no grasp point left")
    return CargoView(None, n_cloth, seen, overlay)


SAME_DE = 12.0  # a sock in two views of the box: this close in color (ΔE, Lab)
SAME_MM = 60.0  # and in place (its visible part's centroid), when there are no masks
SAME_OVERLAP = 0.3  # or covering this much of where it was (the same look pose)


def _same(b: SockSeen, a: SockSeen) -> float | None:
    """How unlike sock `b` of an earlier view sock `a` of a later one is (lower: more alike);
    None: not the same sock. Taking the sock above another shows more of that one, and moves
    its centroid: where masks are there, the overlap with where it was decides."""
    de = math.dist(b.lab, a.lab)
    if de >= SAME_DE:
        return None
    if b.mask is not None and a.mask is not None and b.mask.shape == a.mask.shape:
        kept = float((b.mask & a.mask).sum()) / max(int(b.mask.sum()), 1)
        return de + 50.0 * (1.0 - kept) if kept >= SAME_OVERLAP else None
    d = math.dist(b.xy, a.xy)
    return de + 0.2 * d if d < SAME_MM else None


def taken(before: CargoView, after: CargoView) -> list[SockSeen]:
    """The socks seen in `before` that are not in `after`: what a pick took out of the box.
    Each sock in `after` is the one of `before` most like it, if alike enough."""
    pairs = []
    for i, b in enumerate(before.seen):
        for j, a in enumerate(after.seen):
            cost = _same(b, a)
            if cost is not None:
                pairs.append((cost, i, j))
    kept: set[int] = set()
    used: set[int] = set()
    for _, i, j in sorted(pairs):
        if i not in kept and j not in used:
            kept.add(i)
            used.add(j)
    return [b for i, b in enumerate(before.seen) if i not in kept]


def _window(depth: np.ndarray, u: int, v: int, half: int) -> np.ndarray:
    win = depth[max(v - half, 0) : v + half + 1, max(u - half, 0) : u + half + 1]
    win = win[win > 0]
    return win if win.size else np.array([0.0])


def _erode(mask: np.ndarray, px: int) -> np.ndarray:
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * px + 1, 2 * px + 1))
    return cv2.erode(mask.astype(np.uint8), k).astype(bool)


def _pixel_of(obs: Observation, a: ArmPoint) -> PixelPoint | None:
    from sorter.box_detector.geometry import project

    uv = project(obs, np.array([[a.x, a.y, a.z]]))[0]
    if np.isnan(uv).any():
        return None
    return PixelPoint(int(round(uv[0])), int(round(uv[1])))


def _outline(mask: np.ndarray) -> list[PixelPoint]:
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return []
    c = cv2.approxPolyDP(max(contours, key=cv2.contourArea), 2.0, True)
    return [PixelPoint(int(q[0][0]), int(q[0][1])) for q in c]
