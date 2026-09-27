"""Pixels ↔ the arm frame for a whole frame at once (pinhole; the calibration module does the
exact per-point conversion of a grasp)."""

from __future__ import annotations

from collections.abc import Sequence

import cv2
import numpy as np

from sorter.core.types import Observation


def points(obs: Observation) -> np.ndarray:
    """HxWx3 arm-frame points (mm) of every pixel with depth; NaN where there is none."""
    assert obs.T_base_cam is not None
    k = obs.frame.intrinsics
    z = obs.frame.depth_mm.astype(np.float64)
    v, u = np.mgrid[0 : k.height, 0 : k.width]
    cam = np.stack([(u - k.cx) / k.fx * z, (v - k.cy) / k.fy * z, z], axis=-1)
    T = obs.T_base_cam
    p = cam @ T[:3, :3].T + T[:3, 3]
    p[z == 0] = np.nan
    return p


def project(obs: Observation, xyz: np.ndarray) -> np.ndarray:
    """Nx2 pixel coordinates (float) of Nx3 arm-frame points (mm); NaN behind the camera."""
    assert obs.T_base_cam is not None
    k = obs.frame.intrinsics
    T = np.linalg.inv(obs.T_base_cam)
    c = np.asarray(xyz, dtype=np.float64) @ T[:3, :3].T + T[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = np.stack([k.fx * c[:, 0] / c[:, 2] + k.cx, k.fy * c[:, 1] / c[:, 2] + k.cy], axis=1)
    uv[c[:, 2] <= 0] = np.nan
    return uv


def polygon_mask(
    obs: Observation, poly_xy: Sequence[tuple[float, float]], z_mm: float
) -> np.ndarray:
    """HxW bool: the pixels that see the arm-frame polygon `poly_xy` at height `z_mm`."""
    k = obs.frame.intrinsics
    xyz = np.array([(x, y, z_mm) for x, y in poly_xy])
    uv = project(obs, xyz)
    m = np.zeros((k.height, k.width), np.uint8)
    if not np.isnan(uv).any():
        cv2.fillPoly(m, [np.round(uv).astype(np.int32)], 1)
    return m.astype(bool)


def in_polygon(x: np.ndarray, y: np.ndarray, poly: Sequence[tuple[float, float]]) -> np.ndarray:
    """Elementwise: (x, y) inside the polygon (even-odd rule); NaN is outside."""
    inside = np.zeros(np.shape(x), bool)
    n = len(poly)
    with np.errstate(invalid="ignore", divide="ignore"):
        for i in range(n):
            (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
            if y0 == y1:
                continue
            cross = ((y0 > y) != (y1 > y)) & (x < x0 + (y - y0) * (x1 - x0) / (y1 - y0))
            inside ^= cross
    return inside
