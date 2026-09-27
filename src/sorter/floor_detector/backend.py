"""Real backend of the floor detector (stage A): SAM3 segmentation service + color statistics."""

from __future__ import annotations

from typing import TYPE_CHECKING

from sorter.color_classifier.classifier import Sam3ColorClassifier
from sorter.color_classifier.segmenter import SamSegmenter
from sorter.core.types import Zone
from sorter.floor_detector.detector import ClassifierFloorDetector

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config, segment=None) -> ClassifierFloorDetector:
    """`segment`: the segmentation function (default: the SAM3 service; the simulator passes its
    rendered segmentation)."""
    view = cfg.views.get(Zone.FLOOR)
    if segment is None:
        segment = SamSegmenter(cfg.color_classifier.sam).segment
    return ClassifierFloorDetector(
        Sam3ColorClassifier(cfg.color_classifier, segment, view.roi if view else ())
    )
