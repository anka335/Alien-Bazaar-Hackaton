import numpy as np
import pytest

from sorter.arm import kinematics as kin


def test_zero_pose_points_forward():
    T = kin.fk(np.zeros(kin.N_JOINTS))
    assert T[:3, 2] == pytest.approx([1, 0, 0], abs=1e-3)
    assert T[0, 3] > 300  # stretched out along +X


@pytest.mark.parametrize("target", [(200, 0, 20), (180, 80, 40), (150, -100, 0)])
def test_ik_down_reaches_the_table_top_down(target):
    r = kin.ik_down(target, [0.0, 0.0, 0.5, 1.0, 0.0])
    assert r.pos_err_mm < 1.0
    assert r.tilt_deg < 3.0
    assert kin.fk(r.q)[:3, 3] == pytest.approx(target, abs=1.0)


def test_ik_down_tilts_instead_of_failing_at_the_edge():
    r = kin.ik_down((315, 0, 30), [0.0, 0.0, 0.5, 1.0, 0.0])
    assert r.pos_err_mm < 1.0
    assert r.tilt_deg > 5.0


def test_jacobian_matches_finite_differences():
    q = np.array([0.3, -0.2, 0.4, 0.8, 0.1])
    T, J = kin.jacobian(q)
    eps = 1e-6
    for i in range(kin.N_JOINTS):
        dq = np.zeros(kin.N_JOINTS)
        dq[i] = eps
        dp = (kin.fk(q + dq)[:3, 3] - T[:3, 3]) / eps
        assert J[:3, i] == pytest.approx(dp, abs=1e-3)
