import threading

import numpy as np
import pytest

from sorter.arm import kinematics as kin
from sorter.arm.bus import MockBus
from sorter.arm.config import ArmConfig, ZoneConfig
from sorter.arm.controller import So101Arm, inside
from sorter.core.errors import EStopped, TargetRejected
from sorter.core.types import ArmPoint, ColorClass, Zone

RANGES = {
    1: (745, 3363),
    2: (791, 3328),
    3: (868, 3084),
    4: (666, 3164),
    5: (124, 3970),
    6: (140, 1595),
}
HOME = [0.0, -0.3, 0.6, 1.2, 0.0]
POSES = {
    "rest": [0.0, -1.5, 1.5, 1.0, 0.0],
    "home": HOME,
    "look_box": [0.5, -0.2, 0.3, 1.3, 0.0],
    "look_bg": [-0.5, -0.2, 0.3, 1.3, 0.0],
    "place_bg": [-0.5, 0.0, 0.5, 1.0, 0.0],
    **{f"bin_{c.value}": [1.0 + 0.2 * i, 0.0, 0.5, 1.0, 0.0] for i, c in enumerate(ColorClass)},
}
ZONES = {
    Zone.BACKGROUND: ZoneConfig(
        workspace_mm=[(120, -150), (280, -150), (280, 150), (120, 150)],
        z_floor_mm=5,
        grasp_depth_mm=20,
        approach_mm=60,
    )
}


def _arm(bus=None, **cfg):
    bus = bus or MockBus(RANGES)
    arm = So101Arm(ArmConfig(settle_s=0.05, **cfg), POSES, ZONES, lambda: bus, sleep=lambda s: None)
    return arm, bus


def test_joint_mapping_roundtrip_and_midrange_zero():
    arm, _ = _arm()
    arm.connect()
    mids = [(lo + hi) // 2 for lo, hi in list(RANGES.values())[:5]]
    assert arm.q_from_raw(mids) == pytest.approx(np.zeros(5), abs=2e-3)
    q = np.array([0.2, -0.4, 0.5, 0.9, -0.3])
    assert arm.q_from_raw(arm.raw_from_q(q)) == pytest.approx(q, abs=2e-3)


def test_start_holds_the_current_position():
    bus = MockBus(RANGES, positions={2: 900})
    arm, _ = _arm(bus)
    arm.start()
    assert all(bus.torque.values())
    assert bus.goal[2] == 900  # goal = present before torque on: nothing jumps


def test_goto_moves_to_the_pose_and_look_is_idempotent():
    arm, bus = _arm()
    arm.start()
    arm.look(Zone.BOX)
    assert arm.read_q() == pytest.approx(POSES["look_box"], abs=3e-3)
    n = len(bus.goal_log)
    arm.look(Zone.BOX)
    assert len(bus.goal_log) == n


def test_pick_descends_top_down_closes_and_lifts():
    arm, bus = _arm()
    arm.start()
    arm.goto("home")
    bus.blocked[6] = 400  # cloth between the fingers
    r = arm.pick(ArmPoint(200, 0, 10), Zone.BACKGROUND)
    assert not r.likely_empty and 0.1 < r.gripper_opening < 0.3
    lowest = min(arm.tcp(arm.q_from_raw([g[i] for i in range(1, 6)]))[2] for g in bus.goal_log)
    assert lowest == pytest.approx(5, abs=2)  # clamped to z_floor (10 - 20 < 5)
    assert arm.tcp(arm.read_q())[2] == pytest.approx(70, abs=2)  # back at the approach height


def test_pick_empty_gripper_is_reported():
    arm, _ = _arm()
    arm.start()
    arm.goto("home")
    assert arm.pick(ArmPoint(200, 0, 10), Zone.BACKGROUND).likely_empty


@pytest.mark.parametrize(
    "target, zone",
    [(ArmPoint(400, 0, 10), Zone.BACKGROUND), (ArmPoint(200, 0, 10), Zone.BOX)],
)
def test_pick_outside_the_workspace_is_rejected_without_motion(target, zone):
    arm, bus = _arm()
    arm.start()
    n = len(bus.goal_log)
    with pytest.raises(TargetRejected):
        arm.pick(target, zone)
    assert len(bus.goal_log) == n


def test_hold_stops_motions_until_recover():
    arm, bus = _arm()
    arm.start()
    arm.hold()
    with pytest.raises(EStopped):
        arm.home()
    arm.recover()
    assert arm.read_q() == pytest.approx(HOME, abs=3e-3)
    assert arm.read_gripper() == pytest.approx(0.7, abs=0.01)  # opened above the background


def test_hold_from_another_thread_interrupts_a_motion():
    arm, bus = _arm()
    arm.start()
    sent = threading.Event()
    orig = bus.write_goals

    def write_goals(goals):
        orig(goals)
        if len(bus.goal_log) == 10:
            threading.Thread(target=arm.hold).start()
            sent.set()

    bus.write_goals = write_goals
    arm._sleep = lambda s: sent.wait(0.01)
    with pytest.raises(EStopped):
        arm.goto("bin_dark")


def test_shutdown_goes_to_rest_then_disables():
    arm, bus = _arm()
    arm.start()
    arm.shutdown()
    assert not any(bus.torque.values()) and bus.closed
    assert arm.bus is None


def test_shutdown_keeps_torque_if_rest_is_unreachable():
    bus = MockBus(RANGES)
    arm = So101Arm(ArmConfig(settle_s=0.05), {}, ZONES, lambda: bus, sleep=lambda s: None)
    arm.start()
    arm.shutdown()
    assert all(bus.torque.values())


def test_blocked_joint_raises_on_a_strict_move():
    arm, bus = _arm()
    arm.start()
    bus.blocked[2] = bus.pos[2] + 50
    with pytest.raises(Exception, match="blocked"):
        arm.move_joints([0.0, 1.0, 0.0, 0.0, 0.0])


def test_inside():
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    assert inside(square, 5, 5) and not inside(square, 15, 5)


def test_fk_of_tcp_extension():
    arm, _ = _arm(tcp_extend_mm=10)
    q = np.array(HOME)
    T = kin.fk(q)
    assert arm.tcp(q) == pytest.approx(T[:3, 3] + 10 * T[:3, 2])


def test_sag_is_commanded_away():
    arm, bus = _arm()
    arm.start()
    bus.sag[2] = 60  # ≈ 5° short under load
    arm.goto("home")
    assert arm.read_q() == pytest.approx(HOME, abs=np.radians(1.0))


def test_drop_to_bin_goes_through_home():
    arm, bus = _arm()
    arm.start()
    arm.look(Zone.BACKGROUND)
    visited = []
    orig = arm.goto
    arm.goto = lambda name, speed_scale=1.0: (visited.append(name), orig(name, speed_scale))
    arm.drop_to_bin(ColorClass.DARK)
    assert visited == ["home", "bin_dark", "home"]
    assert arm.read_gripper() == pytest.approx(0.7, abs=0.01)
