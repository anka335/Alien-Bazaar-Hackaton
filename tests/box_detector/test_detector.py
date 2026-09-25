import numpy as np
import pytest

from sorter.box_detector.config import BoxDetectorConfig
from sorter.box_detector.detector import SamBoxDetector
from sorter.color_classifier.segmenter import Instance
from sorter.core.types import BoxStatus, Frame, Intrinsics, PixelPoint

H, W = 240, 320
ROI = [(20, 20), (300, 20), (300, 220), (20, 220)]


def _frame():
    return Frame(
        color=np.zeros((H, W, 3), np.uint8),
        depth_mm=None,
        intrinsics=Intrinsics(300, 300, W / 2, H / 2, W, H),
        timestamp=0.0,
        seq=0,
    )


def _rect(u0, v0, u1, v1):
    m = np.zeros((H, W), dtype=bool)
    m[v0:v1, u0:u1] = True
    return m


def _detector(masks, **cfg):
    return SamBoxDetector(
        BoxDetectorConfig(min_area_px=50, **cfg),
        lambda _img: [Instance(m, 0.9) for m in masks],
        ROI,
    )


def test_empty_box():
    r = _detector([]).detect(_frame())
    assert r.status is BoxStatus.EMPTY and r.grasp is None and r.coverage == 0


def test_grasp_in_the_middle_of_the_cloth_away_from_walls():
    r = _detector([_rect(100, 80, 220, 180)]).detect(_frame())
    assert r.status is BoxStatus.GRASP
    assert r.grasp.depth_mm is None
    assert (r.grasp.px.u, r.grasp.px.v) == (pytest.approx(160, abs=3), pytest.approx(130, abs=3))
    assert 0.1 < r.coverage < 0.3


def test_cloth_at_the_wall_is_grasped_at_the_margin():
    r = _detector([_rect(0, 0, 160, 240)], wall_margin_px=40).detect(_frame())
    assert r.status is BoxStatus.GRASP
    assert r.grasp.px.u >= 60 and r.grasp.px.v >= 60 and r.grasp.px.v <= 180


def test_avoid_moves_the_grasp_and_can_leave_no_candidate():
    det = _detector([_rect(100, 80, 220, 180)], avoid_radius_px=30)
    first = det.detect(_frame()).grasp.px
    second = det.detect(_frame(), [first]).grasp.px
    assert np.hypot(second.u - first.u, second.v - first.v) > 30
    everywhere = [PixelPoint(u, v) for u in range(100, 221, 20) for v in range(80, 181, 20)]
    r = det.detect(_frame(), everywhere)
    assert r.status is BoxStatus.NO_GRASP
