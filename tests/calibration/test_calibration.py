import math

import numpy as np
import pytest

from sorter.calibration.calibration import HandEyeCalibration
from sorter.calibration.hand_eye import solve
from sorter.core.errors import SorterError
from sorter.core.types import ArmPoint, Frame, GraspPoint, Intrinsics, Observation, Zone


def pose(rx=0.0, ry=0.0, rz=0.0, t=(0, 0, 0)) -> np.ndarray:
    cx, sx, cy, sy, cz, sz = (f(a) for a in (rx, ry, rz) for f in (math.cos, math.sin))
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = t
    return T


X = pose(0.1, -0.05, 1.5, (-140, 5, 55))  # flange → camera


def obs(ee: np.ndarray) -> Observation:
    k = Intrinsics(615, 615, 320, 240, 640, 480)
    f = Frame(np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640), np.uint16), k, 0.0, 0)
    return Observation(f, Zone.BOX, ee @ X, None)


def test_pixel_to_arm_and_back():
    cal = HandEyeCalibration(X)
    o = obs(pose(math.pi, 0, 0.3, (250, -100, 300)))
    c = np.array([12.0, -20.0, 250.0, 1.0])  # in front of the camera
    x, y, z = (o.T_base_cam @ c)[:3]
    p = ArmPoint(x, y, z)
    px = cal.to_pixel(o, p)
    assert px is not None
    assert (px.u, px.v) == (round(320 + 615 * 12 / 250), round(240 - 615 * 20 / 250))
    depth = 250.0
    q = cal.to_arm(o, GraspPoint(px, float(depth)))
    assert np.allclose([q.x, q.y, q.z], [p.x, p.y, p.z], atol=0.5)  # pixel rounding


def test_hand_eye_recovers_the_mount():
    rng = np.random.default_rng(0)
    board = pose(0, 0, 0.2, (270, -60, 0))  # in the arm base frame
    pairs = []
    for _ in range(10):
        ee = pose(
            math.pi + rng.uniform(-0.3, 0.3),
            rng.uniform(-0.3, 0.3),
            rng.uniform(-1, 1),
            (250 + rng.uniform(-40, 40), rng.uniform(-40, 40), 300 + rng.uniform(-30, 30)),
        )
        pairs.append((ee, np.linalg.inv(ee @ X) @ board))
    T, rmse = solve(pairs)
    assert np.allclose(T, X, atol=1e-3) and rmse < 1e-3


def test_hand_eye_needs_views():
    with pytest.raises(SorterError):
        solve([(np.eye(4), np.eye(4))] * 3)
