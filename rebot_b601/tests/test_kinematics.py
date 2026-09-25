import numpy as np
import pytest

from rebot_b601 import kinematics as K
from rebot_b601.config import JOINT_LIMITS_RAD as LIM


def rand_q(rng):
    return LIM[:, 0] + (LIM[:, 1] - LIM[:, 0]) * rng.random(6)


def test_zero_pose():
    p, R = K.fk(np.zeros(6))
    assert p == pytest.approx([0.3017, 0.0, 0.2177], abs=1e-3)
    assert R[:, 0] == pytest.approx([1, 0, 0], abs=1e-3)       # approach points forward at rest


def test_planar_pitch_relation():
    # approach elevation = q3 + q4 - q2 (measured from the model, see README)
    for q2, q3, q4 in [(30, 0, 0), (0, 30, 0), (0, 0, 30), (100, 85, -75)]:
        _, R = K.fk(np.radians([0, q2, q3, q4, 0, 0]))
        assert np.degrees(np.arcsin(R[2, 0])) == pytest.approx(q3 + q4 - q2, abs=0.2)


def test_jacobian_matches_numeric():
    rng = np.random.default_rng(0)
    for _ in range(5):
        q = rand_q(rng)
        J = K.jacobian(q)
        eps = 1e-6
        Jn = np.zeros((3, 6))
        for i in range(6):
            d = np.zeros(6)
            d[i] = eps
            Jn[:, i] = (K.fk(q + d)[0] - K.fk(q - d)[0]) / (2 * eps)
        assert np.allclose(J[:3], Jn, atol=1e-6)


def test_ik_roundtrip_position_only():
    rng = np.random.default_rng(3)
    solved = 0
    n = 40
    for _ in range(n):
        q = rand_q(rng)
        p, _ = K.fk(q)
        seed = np.clip(q + rng.normal(0, 0.3, 6), LIM[:, 0], LIM[:, 1])
        r = K.solve_ik(p, None, seed, LIM, z_min=None)
        if r.success:
            solved += 1
            assert np.linalg.norm(K.fk(r.q)[0] - p) < 1.1e-3
            assert np.all(r.q >= LIM[:, 0] - 1e-9) and np.all(r.q <= LIM[:, 1] + 1e-9)
    assert solved >= 0.9 * n


def test_ik_with_approach():
    seed = np.radians([0, 60, 90, 0, 0, 0])
    r = K.solve_ik([0.30, 0.0, 0.10], "down", seed, LIM)
    assert r.success
    p, R = K.fk(r.q)
    assert np.linalg.norm(p - [0.30, 0, 0.10]) < 1e-3
    assert R[:, 0] @ [0, 0, -1] > np.cos(np.radians(3))


def test_ik_rejects_unreachable_and_table():
    seed = np.zeros(6)
    assert not K.solve_ik([1.0, 0.0, 0.3], None, seed, LIM).success       # too far
    assert not K.solve_ik([0.3, 0.0, -0.1], None, seed, LIM).success      # below the table
    assert not K.solve_ik([0.3, 0.0, 0.3], "down", seed, LIM).success     # 'down' not possible this high


def test_parse_approach():
    assert K.parse_approach("free") is None
    assert K.parse_approach(None) is None
    assert K.parse_approach("down") == pytest.approx([0, 0, -1])
    assert K.parse_approach([0, 0, 2]) == pytest.approx([0, 0, 1])
    with pytest.raises(ValueError):
        K.parse_approach("sideways")
    with pytest.raises(ValueError):
        K.parse_approach([0, 0, 0])


def test_home_pose_is_safe():
    ok, why = K.pose_is_safe(np.zeros(6), z_min=0.03)
    assert ok, why
