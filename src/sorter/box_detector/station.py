"""Where a laundry bin of the unload station really stands, from one look at it.

The rover parks within a tolerance and the bins are put down by hand, so the layout's bin
places are only a first guess. From a look pose over that guess every pixel becomes an
arm-frame point. Seen from above, every point of a bin's walls (their tops and the upper part of
their faces) lies on a square ring of the bin's size, so the wall points (between 40 % of the
bin's height and a bit over its rim above the floor) are drawn into a top-down grid and the
ring is matched against it at every turn: the best match is the bin's center and turn. A
neighbor bin in view is another ring, farther than the search reaches. The share of the ring
the camera saw at all tells whether the bin was all in view: if not, the caller looks again,
centered on the estimate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from sorter.box_detector.geometry import points, project
from sorter.core.types import Marker, Observation, Overlay, PixelPoint

MM_PX = 2.0  # the top-down grid
SEARCH_MM = 90.0  # how far from the guess the center may be
TURNS_DEG = np.arange(-20.0, 20.01, 1.0)  # the turns tried (a square repeats every 90°)
MIN_SEEN = 0.9  # share of the ring in view to call the fit complete
MIN_SCORE = 0.35  # share of the ring covered by wall points to call it a bin


@dataclass
class BinFit:
    center: tuple[float, float] | None  # mm, arm frame; None: no bin found
    yaw: float  # rad, the turn of its sides from the x / y axes
    score: float  # share of the ring covered by wall points
    seen: float  # share of the ring in view
    complete: bool  # a bin, all of it in view
    overlay: Overlay


def _ring(size_mm: float, wall_mm: float, turn_deg: float) -> np.ndarray:
    """A square ring (float 0/1) of outer side `size_mm`, turned, on the top-down grid."""
    n = int(math.ceil(size_mm * 1.5 / MM_PX)) | 1
    t = np.zeros((n, n), np.float32)
    c = (n - 1) / 2
    mid = (size_mm - wall_mm) / 2 / MM_PX  # the wall's middle line
    half_w = max(wall_mm / 2 / MM_PX + 1.5, 2.0)
    yy, xx = np.mgrid[0:n, 0:n] - c
    a = math.radians(turn_deg)
    u = np.cos(a) * xx + np.sin(a) * yy
    v = -np.sin(a) * xx + np.cos(a) * yy
    d = np.abs(np.maximum(np.abs(u), np.abs(v)) - mid)
    t[d <= half_w] = 1.0
    return t


def find_bin(
    obs: Observation,
    guess: tuple[float, float],
    floor_z_mm: float,
    size_mm: float,
    height_mm: float,
    wall_mm: float = 5.0,
) -> BinFit:
    p = points(obs)
    h = p[..., 2] - floor_z_mm
    with np.errstate(invalid="ignore"):
        wall = (h > 0.4 * height_mm) & (h < height_mm + 10)
    seen_px = ~np.isnan(h)
    overlay = Overlay(mask=wall)
    # the top-down grids around the guess: wall points, and where the camera saw anything
    half = SEARCH_MM + size_mm * 0.75
    n = int(math.ceil(2 * half / MM_PX))
    x0, y0 = guess[0] - half, guess[1] - half

    def grid(mask: np.ndarray) -> np.ndarray:
        xy = p[mask][:, :2]
        i = ((xy[:, 1] - y0) / MM_PX).astype(int)  # rows along y
        j = ((xy[:, 0] - x0) / MM_PX).astype(int)  # columns along x
        ok = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        g = np.zeros((n, n), np.float32)
        g[i[ok], j[ok]] = 1.0
        return g

    walls = cv2.dilate(grid(wall), np.ones((3, 3), np.uint8))
    # the floor right behind a wall is hidden by it: a narrow strip, closed here
    seen = cv2.morphologyEx(grid(seen_px), cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    best = (-1.0, 0.0, 0, 0, None)
    for turn in TURNS_DEG:
        t = _ring(size_mm, wall_mm, float(turn))
        r = cv2.matchTemplate(walls, t, cv2.TM_CCORR) / t.sum()
        _, score, _, (j, i) = cv2.minMaxLoc(r)
        if score > best[0]:
            best = (score, float(turn), i, j, t)
    score, turn, i, j, t = best
    k = t.shape[0]
    cx = x0 + (j + (k - 1) / 2) * MM_PX + MM_PX / 2
    cy = y0 + (i + (k - 1) / 2) * MM_PX + MM_PX / 2
    in_view = float((seen[i : i + k, j : j + k] * t).sum() / t.sum())
    found = score >= MIN_SCORE * in_view and score > 0.2
    yaw = math.radians(turn)
    fit = BinFit(
        (float(cx), float(cy)) if found else None,
        yaw,
        float(score),
        in_view,
        found and in_view >= MIN_SEEN,
        overlay,
    )
    if not found:
        overlay.text.append(f"no bin near ({guess[0]:.0f}, {guess[1]:.0f}) ({score:.2f})")
        return fit
    s = size_mm / 2
    c, sn = math.cos(yaw), math.sin(yaw)
    square = ((-s, -s), (s, -s), (s, s), (-s, s))
    corners = [(cx + c * u - sn * v, cy + sn * u + c * v) for u, v in square]
    uv = project(obs, np.array([(x, y, floor_z_mm + height_mm) for x, y in corners]))
    if not np.isnan(uv).any():
        overlay.polygons.append(([PixelPoint(int(a), int(b)) for a, b in uv], "bin"))
    m = project(obs, np.array([[cx, cy, floor_z_mm + height_mm]]))[0]
    if not np.isnan(m).any():
        overlay.markers.append(Marker(PixelPoint(int(m[0]), int(m[1])), "bin", "info"))
    overlay.text.append(
        f"bin at ({cx:.0f}, {cy:.0f}), {turn:+.0f}°, "
        f"ring {score:.0%} covered, {in_view:.0%} in view"
    )
    return fit
