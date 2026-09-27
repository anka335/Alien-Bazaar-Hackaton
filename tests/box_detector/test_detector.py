import numpy as np
import pytest

from sorter.box_detector.config import BoxDetectorConfig
from sorter.box_detector.detector import DepthBoxDetector
from sorter.core.types import BoxStatus, Frame, Intrinsics, PixelPoint

W, H, FLOOR = 640, 480, 260.0
ROI = [(0, 0), (W - 1, 0), (W - 1, H - 1), (0, H - 1)]


def frame(bumps=()) -> Frame:
    """Depth of the box floor with cloth bumps: (u, v, radius px, height mm)."""
    yy, xx = np.mgrid[0:H, 0:W]
    depth = np.full((H, W), FLOOR)
    for u, v, r, h in bumps:
        d = np.hypot(xx - u, yy - v)
        depth = np.minimum(depth, np.where(d < r, FLOOR - h * (1 - (d / r) ** 2), FLOOR))
    return Frame(
        color=np.zeros((H, W, 3), np.uint8),
        depth_mm=np.rint(depth).astype(np.uint16),
        intrinsics=Intrinsics(615, 615, W / 2, H / 2, W, H),
        timestamp=0.0,
        seq=0,
    )


@pytest.fixture
def detector():
    return DepthBoxDetector(BoxDetectorConfig(), ROI)


def test_grasps_the_top_of_the_pile(detector):
    r = detector.detect(frame([(380, 240, 70, 40), (230, 250, 60, 20)]))
    assert r.status is BoxStatus.GRASP
    assert abs(r.grasp.px.u - 380) <= 3 and abs(r.grasp.px.v - 240) <= 3
    assert r.grasp.depth_mm == pytest.approx(FLOOR - 40, abs=2)
    assert 0 < r.coverage < 1


def test_avoid_skips_the_failed_point(detector):
    a = PixelPoint(380, 240)
    r = detector.detect(frame([(380, 240, 70, 40)]), [a])
    assert r.status is BoxStatus.GRASP
    d = np.hypot(r.grasp.px.u - a.u, r.grasp.px.v - a.v)
    assert detector.cfg.avoid_radius_px <= d < 70  # the same pile, off the failed point


def test_empty_box(detector):
    r = detector.detect(frame())
    assert r.status is BoxStatus.EMPTY and r.grasp is None


def test_cloth_only_at_the_wall_has_no_grasp(detector):
    # 45 mm on a floor 260 mm away is ~106 px: cloth closer to the edge can't be grasped
    r = detector.detect(frame([(610, 240, 50, 30)]))
    assert r.status is BoxStatus.NO_GRASP
