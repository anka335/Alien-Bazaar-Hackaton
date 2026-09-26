"""Real backend of block 1: the RealSense D435i on the wrist."""

from __future__ import annotations

from typing import TYPE_CHECKING

from sorter.camera.realsense import RealSenseCamera

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config) -> RealSenseCamera:
    return RealSenseCamera(cfg.camera)
