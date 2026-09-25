import numpy as np

from sorter.core.io import load_observation, save_observation
from sorter.core.types import Frame, Intrinsics, Observation, Zone


def _obs(T, joints):
    rng = np.random.default_rng(0)
    k = Intrinsics(fx=600, fy=601, cx=320, cy=240, width=8, height=6, coeffs=(0.1, -0.2))
    frame = Frame(
        color=rng.integers(0, 255, (6, 8, 3), dtype=np.uint8),
        depth_mm=rng.integers(0, 2000, (6, 8), dtype=np.uint16),
        intrinsics=k,
        timestamp=12.5,
        seq=7,
    )
    return Observation(frame, Zone.BOX, T, joints)


def test_roundtrip(tmp_path):
    obs = _obs(np.eye(4) * 2.0, (0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    path = save_observation(tmp_path / "sub" / "0001_look_box", obs)
    assert path.suffix == ".npz" and path.with_suffix(".png").is_file()
    back = load_observation(path)
    assert np.array_equal(back.frame.color, obs.frame.color)
    assert np.array_equal(back.frame.depth_mm, obs.frame.depth_mm)
    assert back.frame.depth_mm.dtype == np.uint16
    assert back.frame.intrinsics == obs.frame.intrinsics
    assert (back.frame.timestamp, back.frame.seq, back.zone) == (12.5, 7, Zone.BOX)
    assert np.array_equal(back.T_base_cam, obs.T_base_cam)
    assert back.joints == obs.joints


def test_roundtrip_without_arm(tmp_path):
    back = load_observation(save_observation(tmp_path / "x", _obs(None, None)))
    assert back.T_base_cam is None and back.joints is None


def test_roundtrip_without_depth(tmp_path):
    obs = _obs(None, None)
    rgb = Observation(
        Frame(obs.frame.color, None, obs.frame.intrinsics, 1.0, 1), Zone.BACKGROUND, None, None
    )
    back = load_observation(save_observation(tmp_path / "rgb", rgb))
    assert back.frame.depth_mm is None
    assert np.array_equal(back.frame.color, rgb.frame.color)
