"""Arm logic against the simulated backend (no hardware, no CAN)."""

import threading
import time

import numpy as np
import pytest

from rebot_b601.arm import Arm, ArmError, path_duration


@pytest.fixture
def arm():
    a = Arm(max_speed_scale=1.0)
    a.connect(simulate=True)
    yield a
    if a.connected:
        a.backend.blocked.clear()
        a._stop_thread.set()
        a.backend.close()
        a.backend = None


def test_status_after_connect(arm):
    s = arm.status()
    assert s["connected"] and s["simulated"] and s["torque_enabled"]
    assert s["tcp_xyz_m"] == pytest.approx([0.3017, 0.0, 0.2177], abs=1e-3)
    assert s["fault"] is None


def test_move_to_xyz_reaches_target(arm):
    r = arm.move_to_xyz(0.30, 0.0, 0.10, approach="down", speed_scale=1.0)
    assert r["ok"] and r["error_mm"] < 3.0
    assert r["approach_axis"][2] < -0.99


def test_linear_move_from_home_blends_orientation(arm):
    # regression: the start pose points forward, the goal points down; the constraint must be blended in
    plan = arm.plan_xyz(0.30, 0.05, 0.08, approach="down", linear=True)
    assert plan["reachable"] and plan["waypoints"] > 10


def test_linear_relative_move(arm):
    arm.move_to_xyz(0.30, 0.0, 0.10, approach="down", speed_scale=1.0)
    r = arm.move_relative(dy=0.05, approach="down", linear=True, speed_scale=1.0)
    assert r["error_mm"] < 3.0
    assert r["reached_xyz_m"][1] == pytest.approx(0.05, abs=3e-3)


def test_plan_does_not_move(arm):
    before = arm.status()["joints_deg"]
    plan = arm.plan_xyz(0.30, 0.0, 0.10, approach="down")
    assert plan["reachable"] and plan["error_mm"] < 1.0
    time.sleep(0.2)
    assert arm.status()["joints_deg"] == before


def test_rejections(arm):
    with pytest.raises(ArmError, match="workspace"):
        arm.move_to_xyz(2.0, 0.0, 0.2)
    with pytest.raises(ArmError, match="not reachable"):
        arm.move_to_xyz(0.5, 0.0, 0.5, approach="down")
    with pytest.raises(ArmError, match="limit"):
        arm.move_joints([0, 200, 0, 0, 0, 0])
    with pytest.raises(ArmError, match="6 finite"):
        arm.move_joints([0, 0, 0])
    with pytest.raises(ArmError):
        arm.move_joints([0, float("nan"), 0, 0, 0, 0])
    with pytest.raises(ArmError, match="speed_scale"):
        arm.move_joints([0, 10, 10, 0, 0, 0], speed_scale=-1)


def test_speed_is_capped(arm):
    arm.max_speed_scale = 0.5
    assert arm._speed(5.0) == 0.5
    q = np.radians([0, 40, 40, 0, 0, 0])
    wps = np.vstack([np.zeros(6), q])
    assert path_duration(wps, 0.5) > path_duration(wps, 1.0)


def test_joint_move_and_home(arm):
    arm.move_joints([20, 30, 40, 0, 0, 0], speed_scale=1.0)
    assert arm.status()["joints_deg"][:3] == pytest.approx([20, 30, 40], abs=1.0)
    arm.home(speed_scale=1.0)
    assert np.allclose(arm.status()["joints_deg"], 0, atol=1.0)


def test_stop_cancels_move(arm):
    err = {}

    def run():
        try:
            arm.move_joints([60, 60, 60, 0, 0, 0], speed_scale=0.3)
        except ArmError as e:
            err["e"] = str(e)

    t = threading.Thread(target=run)
    t.start()
    time.sleep(0.8)
    arm.stop()
    t.join(timeout=5)
    assert "stopped" in err.get("e", "")
    q_after = arm.status()["joints_deg"]
    time.sleep(0.3)
    assert arm.status()["joints_deg"] == pytest.approx(q_after, abs=0.6)   # holding, not moving on
    assert 0 < q_after[0] < 60
    arm.move_joints([0, 0, 0, 0, 0, 0], speed_scale=1.0)                   # still usable after stop


def test_busy_is_refused(arm):
    def first_move():
        with pytest.raises(ArmError, match="stopped"):
            arm.move_joints([40, 40, 40, 0, 0, 0], speed_scale=0.3)

    t = threading.Thread(target=first_move)
    t.start()
    time.sleep(0.4)
    with pytest.raises(ArmError, match="busy"):
        arm.move_joints([0, 0, 0, 0, 0, 0])
    arm.stop()
    t.join(timeout=5)


def test_blocked_joint_triggers_fault(arm):
    arm.backend.blocked = {0}                       # joint1 refuses to move (collision)
    with pytest.raises(ArmError, match="joint1"):
        arm.move_joints([60, 0, 0, 0, 0, 0], speed_scale=1.0)
    assert "joint1" in arm.status()["fault"]
    with pytest.raises(ArmError, match="fault"):
        arm.move_joints([0, 0, 0, 0, 0, 0])


def test_clear_fault_keeps_torque_and_allows_motion(arm):
    arm.backend.blocked = {0}
    with pytest.raises(ArmError, match=r"commanded [\d.]+, measured [\d.]+ deg"):
        arm.move_joints([60, 0, 0, 0, 0, 0], speed_scale=1.0)
    arm.backend.blocked = set()
    arm.clear_fault()
    s = arm.status()
    assert s["fault"] is None and s["torque_enabled"]
    arm.move_joints([10, 0, 0, 0, 0, 0], speed_scale=1.0)
    assert abs(arm.status()["joints_deg"][0] - 10) < 2


def test_clear_fault_needs_torque(arm):
    arm.emergency_disable()
    with pytest.raises(ArmError, match="torque is off"):
        arm.clear_fault()


def test_gripper(arm):
    r = arm.set_gripper(1.0)
    assert r["gripper_opening"] > 0.9
    r = arm.set_gripper(0.0)
    assert r["gripper_opening"] < 0.1
    with pytest.raises(ArmError):
        arm.set_gripper(1.5)


def test_emergency_disable(arm):
    arm.emergency_disable()
    assert not arm.status()["torque_enabled"]
    with pytest.raises(ArmError):
        arm.move_joints([0, 10, 10, 0, 0, 0])


def test_disconnect_goes_home():
    a = Arm(max_speed_scale=1.0)
    a.connect(simulate=True, sim_start_deg=[10, 20, 30, 0, 0, 0])
    backend = a.backend
    out = a.disconnect(go_home=True, speed_scale=1.0)
    assert out["ok"] and not a.connected
    assert np.allclose(np.degrees(backend.q), 0, atol=1.0)


def test_read_only_connect_refuses_motion():
    a = Arm()
    a.connect(enable=False, simulate=True)
    with pytest.raises(ArmError, match="not enabled"):
        a.move_joints([0, 10, 10, 0, 0, 0])
    a.disconnect(go_home=False)


def test_bad_start_pose_is_refused():
    a = Arm()
    with pytest.raises(ArmError, match="zero calibration"):
        a.connect(simulate=True, sim_start_deg=[0, -40, 0, 0, 0, 0])
