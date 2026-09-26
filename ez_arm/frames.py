"""Coordinate frames: base (x forward, y left, z up, metres), the wrist camera, pixel ↔ base.

The RealSense sits on link4 (the joint5 motor cap). Its mount is given like in the ROS track
(ros2_ws/src/rebot_b601_moveit_config/config/camera_mount.yaml): the colour lens relative to
gripper_end with every joint at zero; converted here to link4 once, then posed by FK.
"""

from __future__ import annotations

import os

import numpy as np
import yaml
from rebot_b601 import kinematics as K

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MOUNT_FILE = os.path.join(REPO, "ros2_ws/src/rebot_b601_moveit_config/config/camera_mount.yaml")


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _T(xyz, R) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, xyz
    return T


# REP-103 optical frame (z forward, x right, y down) in camera_link (x forward, y left, z up)
_T_LINK_OPT = _T((0, 0, 0), rpy_matrix(-np.pi / 2, 0, -np.pi / 2))


def load_mount(path: str = MOUNT_FILE) -> tuple[str, np.ndarray]:
    """(parent link, parent → optical frame 4x4)."""
    with open(path) as f:
        m = yaml.safe_load(f)
    parent = m.get("parent", "gripper_end")
    zero = K.link_frames(np.zeros(6))
    T_ge_cam = _T(m["xyz"], rpy_matrix(*m["rpy"]))
    T_parent_cam = np.linalg.inv(zero[parent]) @ zero["gripper_end"] @ T_ge_cam
    return parent, T_parent_cam @ _T_LINK_OPT


class CameraFrame:
    """Where the camera is for a given joint vector q [rad]."""

    def __init__(self, path: str = MOUNT_FILE):
        self.parent, self.T_parent_opt = load_mount(path)

    def base_T_opt(self, q) -> np.ndarray:
        return K.link_frames(np.asarray(q, float))[self.parent] @ self.T_parent_opt

    def to_base(self, q, pts_opt: np.ndarray) -> np.ndarray:
        """Nx3 points in the optical frame → Nx3 in base."""
        T = self.base_T_opt(q)
        return pts_opt @ T[:3, :3].T + T[:3, 3]


def deproject(u, v, z_m, K_int) -> np.ndarray:
    """Pixels (arrays ok) + depth along the optical axis → Nx3 optical-frame points."""
    fx, fy, cx, cy = K_int
    u, v, z = np.asarray(u, float), np.asarray(v, float), np.asarray(z_m, float)
    return np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], axis=-1)


def project(p_opt: np.ndarray, K_int) -> np.ndarray:
    fx, fy, cx, cy = K_int
    p = np.atleast_2d(p_opt)
    return np.stack([fx * p[:, 0] / p[:, 2] + cx, fy * p[:, 1] / p[:, 2] + cy], axis=-1)
