"""Real backend of the floor detector (stage A): SAM3 segmentation service + color statistics."""

from __future__ import annotations

from typing import TYPE_CHECKING

from sorter.color_classifier.classifier import Sam3ColorClassifier
from sorter.color_classifier.segmenter import SamSegmenter
from sorter.floor_detector.detector import SockDetector

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config, segment=None) -> SockDetector:
    """`segment`: the segmentation function (default: the SAM3 service; the simulator passes its
    rendered segmentation). The whole frame is searched: the scan poses look all around."""
    if segment is None:
        segment = SamSegmenter(cfg.color_classifier.sam).segment
    return SockDetector(cfg.floor_detector, Sam3ColorClassifier(cfg.color_classifier, segment))
