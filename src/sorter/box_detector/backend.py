"""Real backend of block 3: SAM3 cloth masks (D-015), the same service as block 4."""

from __future__ import annotations

from typing import TYPE_CHECKING

from sorter.box_detector.detector import SamBoxDetector
from sorter.color_classifier.segmenter import SamSegmenter
from sorter.core.types import Zone

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config) -> SamBoxDetector:
    view = cfg.views.get(Zone.BOX)
    segmenter = SamSegmenter(cfg.color_classifier.sam)
    return SamBoxDetector(cfg.box_detector, segmenter.segment, view.roi if view else ())
