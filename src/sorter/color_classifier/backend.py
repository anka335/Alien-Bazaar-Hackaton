"""Real backend of block 4: SAM3 segmentation service + color statistics."""

from __future__ import annotations

from typing import TYPE_CHECKING

from sorter.color_classifier.classifier import Sam3ColorClassifier
from sorter.color_classifier.segmenter import SamSegmenter
from sorter.core.types import Zone

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config) -> Sam3ColorClassifier:
    view = cfg.views.get(Zone.BACKGROUND)
    segmenter = SamSegmenter(cfg.color_classifier.sam)
    return Sam3ColorClassifier(cfg.color_classifier, segmenter.segment, view.roi if view else ())
