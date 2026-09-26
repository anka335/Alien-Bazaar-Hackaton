"""HandEyeCalibration: the `Calibration` contract with a pinhole camera and the hand-eye result.

T_base_cam = T_base_flange · T_flange_cam
pixel (u, v) + depth Z  →  camera point (x, y, Z)  →  arm point
"""

from __future__ import annotations

import cv2
import numpy as np

from sorter.core.errors import CalibrationError
from sorter.core.types import ArmPoint, GraspPoint, Intrinsics, Observation, PixelPoint, Pose


def _undistort(k: Intrinsics, u: float, v: float) -> tuple[float, float]:
    """Normalized image coordinates (x/Z, y/Z) of pixel (u, v)."""
    if not k.coeffs:
        return (u - k.cx) / k.fx, (v - k.cy) / k.fy
    K = np.array([[k.fx, 0, k.cx], [0, k.fy, k.cy], [0, 0, 1]])
    p = cv2.undistortPoints(np.array([[[u, v]]], np.float64), K, np.array(k.coeffs))
    return float(p[0, 0, 0]), float(p[0, 0, 1])


class HandEyeCalibration:
    def __init__(self, T_flange_cam: Pose):
        self.T_flange_cam = np.asarray(T_flange_cam, dtype=np.float64)

    def cam_pose(self, ee_pose: Pose) -> Pose:
        return np.asarray(ee_pose, dtype=np.float64) @ self.T_flange_cam

    def to_arm(self, obs: Observation, point: GraspPoint) -> ArmPoint:
        if obs.T_base_cam is None:
            raise CalibrationError("observation has no camera pose")
        if point.depth_mm <= 0:
            raise CalibrationError(f"no depth at {point.px}")
        x, y = _undistort(obs.frame.intrinsics, point.px.u, point.px.v)
        p = obs.T_base_cam @ np.array([x * point.depth_mm, y * point.depth_mm, point.depth_mm, 1.0])
        return ArmPoint(float(p[0]), float(p[1]), float(p[2]))

    def to_pixel(self, obs: Observation, p: ArmPoint) -> PixelPoint | None:
        if obs.T_base_cam is None:
            raise CalibrationError("observation has no camera pose")
        c = np.linalg.inv(obs.T_base_cam) @ np.array([p.x, p.y, p.z, 1.0])
        if c[2] <= 0:
            return None
        k = obs.frame.intrinsics
        K = np.array([[k.fx, 0, k.cx], [0, k.fy, k.cy], [0, 0, 1]])
        uv, _ = cv2.projectPoints(
            c[:3].reshape(1, 3), np.zeros(3), np.zeros(3), K, np.array(k.coeffs or [0.0] * 5)
        )
        u, v = uv.ravel()
        if not (0 <= u < k.width and 0 <= v < k.height):
            return None
        return PixelPoint(int(round(u)), int(round(v)))
