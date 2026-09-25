"""Plane calibration (D-014): the camera has no depth, so pixel → arm goes through one homography
per zone, valid at that zone's look pose, onto the zone's plane."""

from __future__ import annotations

from collections.abc import Sequence

import cv2
import numpy as np

from sorter.calibration.config import CalibrationConfig, ZoneCalibration
from sorter.core.errors import CalibrationError
from sorter.core.types import ArmPoint, GraspPoint, Observation, PixelPoint, Pose


def fit_homography(
    pixels: Sequence[tuple[float, float]], xy_mm: Sequence[tuple[float, float]]
) -> tuple[np.ndarray, float]:
    """Least-squares homography pixel → arm XY and its RMSE in mm. Needs ≥ 4 points."""
    src = np.asarray(pixels, dtype=np.float64)
    dst = np.asarray(xy_mm, dtype=np.float64)
    if len(src) < 4:
        raise CalibrationError(f"need at least 4 points, got {len(src)}")
    H, _ = cv2.findHomography(src, dst, 0)
    if H is None:
        raise CalibrationError("degenerate points (all on a line?)")
    err = apply(H, src) - dst
    return H, float(np.sqrt((err**2).sum(axis=1).mean()))


def apply(H: np.ndarray, pts: np.ndarray) -> np.ndarray:
    p = np.c_[pts, np.ones(len(pts))] @ H.T
    return p[:, :2] / p[:, 2:3]


class PlaneCalibration:
    """`Calibration` from per-zone homographies. `obs` must come from the zone's look pose."""

    def __init__(self, cfg: CalibrationConfig):
        self.cfg = cfg

    def _zone(self, obs: Observation) -> ZoneCalibration:
        z = self.cfg.zones.get(obs.zone)
        if z is None:
            raise CalibrationError(
                f"{obs.zone} not calibrated; run `python -m sorter.calibration.setup {obs.zone}`"
            )
        return z

    def cam_pose(self, ee_pose: Pose) -> Pose:
        # Informational only (run logs): pixel ↔ arm doesn't use it. The camera sits at the wrist.
        return np.array(ee_pose, dtype=np.float64)

    def to_arm(self, obs: Observation, point: GraspPoint) -> ArmPoint:
        z = self._zone(obs)
        ((x, y),) = apply(np.asarray(z.H), np.array([[point.px.u, point.px.v]], dtype=float))
        return ArmPoint(float(x), float(y), z.plane_z_mm + z.surface_offset_mm)

    def to_pixel(self, obs: Observation, p: ArmPoint) -> PixelPoint | None:
        z = self._zone(obs)
        ((u, v),) = apply(np.linalg.inv(np.asarray(z.H)), np.array([[p.x, p.y]]))
        k = obs.frame.intrinsics
        if not (0 <= u < k.width and 0 <= v < k.height):
            return None
        return PixelPoint(int(round(u)), int(round(v)))
