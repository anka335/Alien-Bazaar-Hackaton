"""Real backend of block 3: the depth-based box detector."""

from __future__ import annotations

from typing import TYPE_CHECKING

from sorter.box_detector.detector import DepthBoxDetector
from sorter.core.types import Zone

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config) -> DepthBoxDetector:
    view = cfg.views.get(Zone.BOX)
    return DepthBoxDetector(cfg.box_detector, view.roi if view else ())
