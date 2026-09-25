import numpy as np
import pytest

from sorter.calibration.config import CalibrationConfig, ZoneCalibration
from sorter.calibration.plane import PlaneCalibration, apply, fit_homography
from sorter.calibration.setup import shrink
from sorter.core.errors import CalibrationError
from sorter.core.types import (
    ArmPoint,
    Frame,
    GraspPoint,
    Intrinsics,
    Observation,
    PixelPoint,
    Zone,
)

# a camera looking down, 2 px per mm: image v → arm -x, image u → arm -y, center → (200, 0)
TRUE_H = np.array([[0.0, -0.5, 200 + 0.5 * 240], [-0.5, 0.0, 0.5 * 320], [0.0, 0.0, 1.0]])


def _obs(zone=Zone.BACKGROUND):
    f = Frame(
        np.zeros((480, 640, 3), np.uint8), None, Intrinsics(500, 500, 320, 240, 640, 480), 0, 0
    )
    return Observation(f, zone, None, None)


def test_fit_homography_recovers_a_known_mapping():
    px = np.array([(100, 100), (540, 90), (560, 400), (80, 420), (320, 240)], dtype=float)
    xy = apply(TRUE_H, px)
    H, rmse = fit_homography(px, xy)
    assert rmse < 1e-6
    assert apply(H, np.array([[200.0, 300.0]])) == pytest.approx(apply(TRUE_H, [[200, 300]]))


def test_fit_homography_needs_four_points():
    with pytest.raises(CalibrationError):
        fit_homography([(0, 0), (1, 0), (0, 1)], [(0, 0), (1, 0), (0, 1)])


def test_to_arm_and_back():
    cal = PlaneCalibration(
        CalibrationConfig(
            zones={Zone.BACKGROUND: ZoneCalibration(H=TRUE_H.tolist(), plane_z_mm=12.0)}
        )
    )
    obs = _obs()
    p = cal.to_arm(obs, GraspPoint(PixelPoint(320, 240), None))
    assert (p.x, p.y, p.z) == pytest.approx((200, 0, 12))
    assert cal.to_pixel(obs, ArmPoint(p.x, p.y, p.z)) == PixelPoint(320, 240)
    assert cal.to_pixel(obs, ArmPoint(2000, 0, 0)) is None
    with pytest.raises(CalibrationError, match="not calibrated"):
        cal.to_arm(_obs(Zone.BOX), GraspPoint(PixelPoint(1, 1), None))


@pytest.mark.parametrize("reverse", [False, True])
def test_shrink_square(reverse):
    square = [(0, 0), (100, 0), (100, 100), (0, 100)]
    out = shrink(square[::-1] if reverse else square, 10)
    assert sorted(out) == sorted([(10, 10), (90, 10), (90, 90), (10, 90)])
