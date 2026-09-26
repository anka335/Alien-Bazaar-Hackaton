import math
import os
import sys

import numpy as np
import pytest

from cloth_task.spectacles_session import Session, quat_to_R, rotvec


def _rebot():
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    sys.path.insert(0, os.path.join(repo, "rebot_b601"))
    from rebot_b601 import config, kinematics

    return kinematics, config


K, C = _rebot()
Q0 = np.radians([10.0, 50.0, 70.0, -20.0, 10.0, 5.0])
IDENTITY = [0.0, 0.0, 0.0, 1.0]


def quat_about(axis, deg):
    a = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return [*(a * math.sin(h)), math.cos(h)]


def teleop(
    seq, *, arm=False, position=(0.0, 0.0, 0.0), orientation=IDENTITY, gripper=0.5, base=False
):
    return {
        "v": 1,
        "type": "teleop",
        "seq": seq,
        "timestamp": 1000.0 + seq,
        "base": {"engaged": base, "vx": 0.3 if base else 0.0, "wz": 0.2 if base else 0.0},
        "arm": {
            "engaged": arm,
            "position": list(position),
            "orientation": list(orientation),
            "gripper": gripper,
        },
    }


class Motor:
    def __init__(self):
        self.joints: list[np.ndarray] = []
        self.gripper: list[float] = []


class Clock:
    def __init__(self):
        self.t = 50.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def rig(clock):
    motor = Motor()
    s = Session(K, C.JOINT_LIMITS_RAD, motor.joints.append, motor.gripper.append, clock)
    s.on_joint_state(Q0)
    s.on_teleop_mode(True)
    assert s.on_connect()
    return s, motor


def assert_pose(q, p, R):
    p_q, R_q = K.fk(q)
    assert np.linalg.norm(p_q - p) <= 1e-3
    assert math.degrees(np.linalg.norm(rotvec(R @ R_q.T))) <= 3.0


def test_rising_right_clutch_latches_the_measured_pose(rig):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True, gripper=0.7))

    assert len(motor.joints) == 1
    assert_pose(motor.joints[0], *K.fk(Q0))
    assert motor.gripper == [0.7]
    assert s.status() == {
        "v": 1, "type": "status", "echoSeq": 0, "base": "idle", "arm": "tracking", "fault": None
    }


def test_later_sample_solves_the_full_pose_from_the_latch(rig):
    s, motor = rig
    p0, R0 = K.fk(Q0)
    s.on_teleop(teleop(0, arm=True))
    s.on_joint_state(Q0 + np.radians([2.0, -1.0, 1.0, 0.0, 3.0, 0.0]))  # arm moved: latch stays
    delta = np.array([0.03, -0.02, 0.02])
    twist = quat_about([0.2, 0.3, 1.0], 15.0)
    s.on_teleop(teleop(1, arm=True, position=delta, orientation=twist, gripper=0.2))

    assert len(motor.joints) == 2
    assert_pose(motor.joints[1], p0 + delta, quat_to_R(np.array(twist)) @ R0)
    assert motor.gripper[-1] == 0.2
    assert np.all(motor.joints[1] >= C.JOINT_LIMITS_RAD[:, 0])
    assert np.all(motor.joints[1] <= C.JOINT_LIMITS_RAD[:, 1])


def test_a_large_step_publishes_the_full_solution_not_an_intermediate(rig):
    s, motor = rig
    p0, R0 = K.fk(Q0)
    s.on_teleop(teleop(0, arm=True))
    s.on_teleop(teleop(1, arm=True, position=[-0.08, 0.10, 0.06]))

    assert_pose(motor.joints[-1], p0 + np.array([-0.08, 0.10, 0.06]), R0)


@pytest.mark.parametrize("gripper, sent", [(-0.4, 0.0), (1.7, 1.0), (0.0, 0.0), (1.0, 1.0)])
def test_gripper_is_clamped_to_closed_through_open(rig, gripper, sent):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True, gripper=gripper))
    assert motor.gripper == [sent]


def test_release_publishes_nothing_more_and_holds(rig):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    s.on_teleop(teleop(1, arm=True, position=[0.01, 0.0, 0.0]))
    sent = (len(motor.joints), len(motor.gripper))
    s.on_teleop(teleop(2, arm=False, position=[0.05, 0.0, 0.0], gripper=0.0))
    s.on_teleop(teleop(3, arm=False))

    assert (len(motor.joints), len(motor.gripper)) == sent
    assert s.status()["arm"] == "holding"
    assert s.status()["echoSeq"] == 3


def test_next_clutch_relatches_where_the_arm_is(rig):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    s.on_teleop(teleop(1, arm=True, position=[0.04, 0.0, 0.0]))
    s.on_teleop(teleop(2))
    q_now = Q0 + np.radians([5.0, 3.0, -4.0, 2.0, 0.0, 0.0])
    s.on_joint_state(q_now)
    s.on_teleop(teleop(3, arm=True))

    assert_pose(motor.joints[-1], *K.fk(q_now))


def test_unreachable_sample_publishes_nothing_and_stays_tracking(rig):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    sent = (len(motor.joints), len(motor.gripper))
    s.on_teleop(teleop(1, arm=True, position=[1.5, 0.0, 0.0], gripper=0.9))

    assert (len(motor.joints), len(motor.gripper)) == sent
    assert s.status()["arm"] == "tracking"
    assert s.status()["echoSeq"] == 1


def test_left_clutch_stays_base_idle_and_moves_nothing(rig):
    s, motor = rig
    s.on_teleop(teleop(0, base=True))

    assert s.status()["base"] == "idle"
    assert s.status()["arm"] == "holding"
    assert motor.joints == [] and motor.gripper == []


def test_left_clutch_does_not_stop_the_arm(rig):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True, base=True))
    s.on_teleop(teleop(1, arm=True, base=True, position=[0.0, 0.02, 0.0]))

    assert len(motor.joints) == 2
    assert s.status()["base"] == "idle"
    assert s.status()["arm"] == "tracking"


def test_status_before_any_teleop():
    s = Session(K, C.JOINT_LIMITS_RAD, lambda q: None, lambda g: None)
    assert s.status() == {
        "v": 1, "type": "status", "echoSeq": None, "base": "idle", "arm": "holding", "fault": None
    }


TIMED_OUT = {"base": "fault", "arm": "fault", "fault": "timeout"}


def link(s):
    st = s.status()
    return {k: st[k] for k in ("base", "arm", "fault")}


def test_silence_after_accept_times_out(rig, clock):
    s, _ = rig
    clock.t += 0.19
    s.tick()
    assert s.status()["fault"] is None
    clock.t += 0.02
    s.tick()
    assert link(s) == TIMED_OUT


def test_timeout_while_tracking_stops_publishing(rig, clock):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    clock.t += 0.25
    s.tick()
    sent = len(motor.joints)
    s.tick()

    assert link(s) == TIMED_OUT
    assert len(motor.joints) == sent


def test_open_clutches_refresh_the_timer_and_hold(rig, clock):
    s, motor = rig
    for seq in range(5):
        clock.t += 0.15
        s.on_teleop(teleop(seq))
    s.tick()

    assert link(s) == {"base": "idle", "arm": "holding", "fault": None}
    assert motor.joints == []


def test_malformed_teleop_does_not_refresh_the_timer_or_echo_seq(rig, clock):
    s, _ = rig
    s.on_teleop(teleop(4))
    clock.t += 0.15
    bad = teleop(5)
    bad["arm"]["position"] = [0.0, 0.0]
    s.on_teleop(bad)
    assert s.status()["echoSeq"] == 4
    clock.t += 0.1
    s.tick()
    assert link(s) == TIMED_OUT
    assert s.status()["echoSeq"] == 4


def test_the_timer_ignores_the_lens_timestamp_and_skipped_seqs(rig, clock):
    s, motor = rig
    for seq in (0, 7, 30):
        clock.t += 0.15
        s.on_joint_state(Q0)
        msg = teleop(seq, arm=True)
        msg["timestamp"] = 1.0  # stale on the lens clock
        s.on_teleop(msg)
    assert s.status()["fault"] is None
    assert s.status()["echoSeq"] == 30
    assert len(motor.joints) == 3


def test_timeout_clearing_frame_with_clutch_held_does_not_track(rig, clock):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    sent = len(motor.joints)
    clock.t += 0.3
    s.on_teleop(teleop(1, arm=True))

    assert link(s) == {"base": "idle", "arm": "holding", "fault": None}
    assert s.status()["echoSeq"] == 1
    assert len(motor.joints) == sent


def test_after_timeout_both_hands_must_release(rig, clock):
    s, motor = rig
    clock.t += 0.3
    s.tick()
    s.on_joint_state(Q0)
    s.on_teleop(teleop(0, arm=True, base=True))  # clears the fault, both still held
    s.on_teleop(teleop(1, arm=False, base=True))  # right released, left still held
    s.on_teleop(teleop(2, arm=True, base=True))
    assert motor.joints == []
    s.on_teleop(teleop(3, arm=True, base=False))  # right was re-grabbed while blocked
    assert motor.joints == []
    s.on_teleop(teleop(4))
    s.on_teleop(teleop(5, arm=True))

    assert len(motor.joints) == 1
    assert s.status()["arm"] == "tracking"


def test_first_socket_accepts_the_first_right_clutch(clock):
    motor = Motor()
    s = Session(K, C.JOINT_LIMITS_RAD, motor.joints.append, motor.gripper.append, clock)
    s.on_joint_state(Q0)
    s.on_teleop_mode(True)
    s.status()
    s.on_connect()
    s.on_teleop(teleop(0, arm=True))
    assert len(motor.joints) == 1
    assert s.status()["arm"] == "tracking"


def test_replacement_socket_stops_the_arm_and_needs_each_hand_open(rig, clock):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    clock.t += 0.3
    s.tick()
    s.on_joint_state(Q0)
    s.on_connect()
    assert link(s) == {"base": "idle", "arm": "holding", "fault": None}
    assert s.status()["echoSeq"] is None

    sent = len(motor.joints)
    s.on_teleop(teleop(0, arm=True))
    s.on_teleop(teleop(1, arm=True, position=[0.01, 0.0, 0.0]))
    assert len(motor.joints) == sent
    assert s.status()["arm"] == "holding"
    s.on_teleop(teleop(2))
    s.on_teleop(teleop(3, arm=True))
    assert len(motor.joints) == sent + 1


def test_replacement_socket_restarts_the_timer(rig, clock):
    s, _ = rig
    clock.t += 0.15
    s.on_connect()
    clock.t += 0.15
    s.tick()
    assert s.status()["fault"] is None
    clock.t += 0.1
    s.tick()
    assert link(s) == TIMED_OUT


HOLDING = {"base": "idle", "arm": "holding", "fault": None}


def test_driver_fault_stops_publishing_and_reports_holding(rig, clock):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    s.on_driver_fault(True)
    assert link(s) == HOLDING
    sent = len(motor.joints)
    for seq in range(1, 4):
        clock.t += 0.03
        s.on_joint_state(Q0)
        s.on_teleop(teleop(seq, arm=True, position=[0.01 * seq, 0.0, 0.0]))

    assert len(motor.joints) == sent
    assert link(s) == HOLDING


def test_new_clutch_does_not_track_while_the_driver_is_faulted(rig, clock):
    s, motor = rig
    s.on_driver_fault(True)
    s.on_teleop(teleop(0))
    s.on_teleop(teleop(1, arm=True))

    assert motor.joints == []
    assert link(s) == HOLDING


def test_after_the_driver_recovers_a_fresh_clutch_tracks(rig, clock):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    s.on_driver_fault(True)
    s.on_teleop(teleop(1, arm=True))
    s.on_driver_fault(False)
    sent = len(motor.joints)
    s.on_teleop(teleop(2, arm=True))
    assert len(motor.joints) == sent
    s.on_teleop(teleop(3))
    s.on_teleop(teleop(4, arm=True))

    assert len(motor.joints) == sent + 1
    assert s.status()["arm"] == "tracking"


def test_stale_measurement_stops_publishing_and_reports_holding(rig, clock):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    clock.t += 0.09
    s.on_teleop(teleop(1, arm=True))
    assert len(motor.joints) == 2
    clock.t += 0.02
    s.on_teleop(teleop(2, arm=True, position=[0.01, 0.0, 0.0]))

    assert len(motor.joints) == 2
    assert link(s) == HOLDING


def test_stale_measurement_is_noticed_by_tick_alone(rig, clock):
    s, _ = rig
    s.on_teleop(teleop(0, arm=True))
    clock.t += 0.11
    s.tick()
    assert link(s) == HOLDING


def fresh_session(clock):
    motor = Motor()
    return Session(K, C.JOINT_LIMITS_RAD, motor.joints.append, motor.gripper.append, clock)


def test_socket_needs_teleop_mode_and_a_measurement(clock):
    s = fresh_session(clock)
    assert not s.on_connect()
    s.on_teleop_mode(True)
    assert not s.on_connect()
    s.on_joint_state(Q0)
    assert s.on_connect()


def test_socket_needs_teleop_mode_even_with_measurements(clock):
    s = fresh_session(clock)
    s.on_joint_state(Q0)
    assert not s.on_connect()


def test_refused_teleop_mode_never_accepts_a_socket(clock):
    s = fresh_session(clock)
    s.on_joint_state(Q0)
    s.on_driver_fault(True)
    s.on_teleop_mode(False)
    assert not s.on_connect()
    s.on_teleop(teleop(0, arm=True))
    assert s.status()["echoSeq"] is None
