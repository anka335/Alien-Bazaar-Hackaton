"""Real backend of block 1: the SO-101 wrist camera (UVC, OpenCV)."""

from __future__ import annotations

from typing import TYPE_CHECKING

from sorter.camera.uvc import UvcCamera

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config) -> UvcCamera:
    return UvcCamera(cfg.camera)
