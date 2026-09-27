"""Where the simulated wrist camera sits, and the cloth colors of the sim scenes."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from sorter.arm import kinematics as kin
from sorter.core.types import ColorClass, Pose
from sorter.sim.config import SimConfig

PALETTE: dict[ColorClass, list[tuple[int, int, int]]] = {  # BGR
    # white, cream, light gray, pale blue
    ColorClass.LIGHT: [(238, 238, 236), (208, 228, 240), (206, 206, 204), (236, 222, 206)],
    # black, navy, charcoal, dark brown
    ColorClass.DARK: [(32, 30, 30), (72, 38, 20), (48, 46, 46), (30, 40, 62)],
    # red, blue, green, yellow, orange, purple, pink
    ColorClass.COLORED: [
        (48, 44, 206),
        (196, 104, 34),
        (70, 160, 48),
        (40, 196, 236),
        (32, 124, 238),
        (150, 62, 118),
        (170, 120, 236),
    ],
}


def camera_mount(cfg: SimConfig) -> Pose:
    """T_link5_cam: the camera is fixed to link5, so joint 6 doesn't turn it (D-027). Where it
    is with joint 6 at 0: `camera_mount_mm` off the TCP, optical axis along the approach."""
    return kin.T_LINK5_TCP0 @ _camera_on_tcp(cfg)


def _camera_on_tcp(cfg: SimConfig) -> Pose:
    """T_tcp_cam with joint 6 at 0."""
    T = np.eye(4)
    T[:3, :3] = [
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0],
    ]  # columns: x_cam = y_tcp, y_cam = z_tcp, z_cam = x_tcp: the image's long side across the
    # arm, as on the rig (~110° from radial there)
    T[:3, 3] = cfg.camera_mount_mm
    return T


def camera_pose(cfg: SimConfig, q: Sequence[float]) -> Pose:
    """T_base_cam for joints `q`."""
    return kin.fk_link5(q) @ camera_mount(cfg)
