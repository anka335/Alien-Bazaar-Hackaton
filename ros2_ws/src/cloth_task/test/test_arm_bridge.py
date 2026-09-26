import numpy as np
import pytest

from cloth_task.arm_bridge import TimedTrajectory, retime

VMAX = np.full(6, 1.0)  # rad/s


def test_retime_leaves_a_slow_trajectory_alone():
    times = np.array([0.0, 1.0, 2.0])
    points = np.array([np.zeros(6), np.full(6, 0.5), np.full(6, 1.0)])
    out, factor = retime(times, points, VMAX)
    assert factor == 1.0
    np.testing.assert_allclose(out, times)


def test_retime_stretches_the_fastest_segment_to_the_limit():
    times = np.array([0.0, 0.5, 1.0])
    points = np.array([np.zeros(6), np.full(6, 1.0), np.full(6, 1.2)])  # 2 rad/s, then 0.4
    out, factor = retime(times, points, VMAX)
    assert factor == pytest.approx(2.0)
    np.testing.assert_allclose(out, times * 2.0)


def test_retime_rejects_a_jump_at_the_same_time():
    with pytest.raises(ValueError):
        retime(np.array([0.0, 0.0]), np.array([np.zeros(6), np.ones(6)]), VMAX)


def test_timed_trajectory_interpolates_and_clamps():
    tr = TimedTrajectory(np.array([0.0, 2.0]), np.array([np.zeros(6), np.full(6, 1.0)]), t0=0.0)
    assert tr.duration == 2.0
    np.testing.assert_allclose(tr.sample(1.0), np.full(6, 0.5))
    np.testing.assert_allclose(tr.sample(5.0), np.full(6, 1.0))  # holds the end


def _rebot_kinematics():
    import os
    import sys

    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    sys.path.insert(0, os.path.join(repo, "rebot_b601"))
    from rebot_b601 import kinematics

    return kinematics


def test_unsafety_matches_pose_is_safe_and_grows_deeper():
    from cloth_task.arm_bridge import unsafety

    K = _rebot_kinematics()
    for deg in ([0, 0, 0, 0, 0, 0], [0, 20, 15, 0, 0, 0], [-50.8, 49.4, 60.5, -72.0, -5.6, 2.0]):
        q = np.radians(deg)
        assert K.pose_is_safe(q, z_min=0.005)[0] and unsafety(K, q, 0.005) == 0.0
    into_table = np.radians([0, 130, 40, -40, 0, 0])
    deeper = np.radians([0, 135, 40, -40, 0, 0])
    assert not K.pose_is_safe(into_table, z_min=0.005)[0]
    assert 0.0 < unsafety(K, into_table, 0.005) < unsafety(K, deeper, 0.005)
