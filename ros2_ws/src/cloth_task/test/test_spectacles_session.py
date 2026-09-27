import math
import os
import sys

import numpy as np
import pytest

from cloth_task.spectacles_session import (
    BASE_STALE_S,
    MEAS_TIMEOUT_S,
    ROVER_ABSENT_S,
    TIMEOUT_S,
    Session,
    check_base_limits,
    quat_to_R,
    rotvec,
)


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
    clock.t += TIMEOUT_S - 0.01
    s.tick()
    assert s.status()["fault"] is None
    clock.t += 0.02
    s.tick()
    assert link(s) == TIMED_OUT


def test_timeout_while_tracking_stops_publishing(rig, clock):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    clock.t += TIMEOUT_S + 0.05
    s.tick()
    sent = len(motor.joints)
    s.tick()

    assert link(s) == TIMED_OUT
    assert len(motor.joints) == sent


def test_open_clutches_refresh_the_timer_and_hold(rig, clock):
    s, motor = rig
    for seq in range(5):
        clock.t += 0.75 * TIMEOUT_S
        s.on_teleop(teleop(seq))
    s.tick()

    assert link(s) == {"base": "idle", "arm": "holding", "fault": None}
    assert motor.joints == []


def test_malformed_teleop_does_not_refresh_the_timer_or_echo_seq(rig, clock):
    s, _ = rig
    s.on_teleop(teleop(4))
    clock.t += 0.75 * TIMEOUT_S
    bad = teleop(5)
    bad["arm"]["position"] = [0.0, 0.0]
    s.on_teleop(bad)
    assert s.status()["echoSeq"] == 4
    clock.t += 0.3 * TIMEOUT_S
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
    clock.t += TIMEOUT_S + 0.1
    s.on_teleop(teleop(1, arm=True))

    assert link(s) == {"base": "idle", "arm": "holding", "fault": None}
    assert s.status()["echoSeq"] == 1
    assert len(motor.joints) == sent


def test_after_timeout_both_hands_must_release(rig, clock):
    s, motor = rig
    clock.t += TIMEOUT_S + 0.1
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
    clock.t += TIMEOUT_S + 0.1
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
    clock.t += 0.75 * TIMEOUT_S
    s.on_connect()
    clock.t += 0.75 * TIMEOUT_S
    s.tick()
    assert s.status()["fault"] is None
    clock.t += 0.3 * TIMEOUT_S
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
    clock.t += MEAS_TIMEOUT_S - 0.01
    s.on_teleop(teleop(1, arm=True))
    assert len(motor.joints) == 2
    clock.t += 0.02
    s.on_teleop(teleop(2, arm=True, position=[0.01, 0.0, 0.0]))

    assert len(motor.joints) == 2
    assert link(s) == HOLDING


def test_stale_measurement_is_noticed_by_tick_alone(rig, clock):
    s, _ = rig
    s.on_teleop(teleop(0, arm=True))
    clock.t += MEAS_TIMEOUT_S + 0.01
    s.tick()
    assert link(s) == HOLDING


def test_a_half_second_network_stall_keeps_tracking(rig, clock):
    # ngrok round trips measured up to 1.2 s; 500 ms stalls were common
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    clock.t += 0.5
    s.on_joint_state(Q0)
    s.on_teleop(teleop(1, arm=True, position=[0.01, 0.0, 0.0]))

    assert len(motor.joints) == 2
    assert s.status()["arm"] == "tracking"


def test_joint_states_150_ms_apart_keep_tracking(rig, clock):
    # arm_bridge on a busy MultiThreadedExecutor published /joint_states up to 150 ms apart
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    clock.t += 0.15
    s.tick()
    s.on_teleop(teleop(1, arm=True, position=[0.01, 0.0, 0.0]))

    assert len(motor.joints) == 2
    assert s.status()["arm"] == "tracking"


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


# Mobile base (left clutch): ADRs 0013 (base stale) and 0014 (rover presence) in the lens repo

ZERO = (0.0, 0.0)
CMD = (0.15, -0.3)  # inside the first-run limits


def drive(seq, vx=CMD[0], wz=CMD[1], *, engaged=True, arm=False, position=(0.0, 0.0, 0.0)):
    msg = teleop(seq, arm=arm, position=position)
    msg["base"] = {"engaged": engaged, "vx": vx, "wz": wz}
    return msg


class Base:
    """A fake send_base: every (vx, wz) sent."""

    def __init__(self):
        self.sent: list[tuple[float, float]] = []

    def __call__(self, vx, wz):
        self.sent.append((vx, wz))


def rover_session(clock, *, rover=True, odometry=True):
    motor, base = Motor(), Base()
    s = Session(
        K,
        C.JOINT_LIMITS_RAD,
        motor.joints.append,
        motor.gripper.append,
        clock,
        send_base=base,
        rover=rover,
        max_vx=0.20,
        max_reverse=0.10,
        max_wz=0.6,
    )
    s.on_joint_state(Q0)
    s.on_teleop_mode(True)
    if odometry:
        s.on_odometry()
    assert s.on_connect()
    return s, motor, base


@pytest.fixture
def rover(clock):
    return rover_session(clock)


def step(s, clock, dt=0.02, *, odometry=True):
    """Time passes with the arm and (unless odometry=False) the rover reporting."""
    clock.t += dt
    s.on_joint_state(Q0)
    if odometry:
        s.on_odometry()


def driving(rover, clock):
    s, motor, base = rover
    s.on_teleop(drive(0))
    step(s, clock)
    s.tick()
    assert base.sent == [CMD]
    assert link(s) == {"base": "driving", "arm": "holding", "fault": None}
    base.sent.clear()
    return s, base


def assert_one_zero_then_silence(s, base, clock):
    assert base.sent == [ZERO]
    for _ in range(5):
        step(s, clock)
        s.tick()
    assert base.sent == [ZERO]
    assert s.status()["base"] in ("idle", "fault")


def test_driving_sends_the_latest_command_on_each_tick(rover, clock):
    s, _, base = rover
    s.on_teleop(drive(0))
    assert base.sent == []  # the tick publishes, not the teleop
    step(s, clock)
    s.tick()
    s.on_teleop(drive(1, 0.05, 0.1))
    step(s, clock)
    s.tick()
    step(s, clock)
    s.tick()

    assert base.sent == [CMD, (0.05, 0.1), (0.05, 0.1)]
    assert s.status()["base"] == "driving"


@pytest.mark.parametrize(
    "vx, wz, sent",
    [
        (0.35, 0.8, (0.20, 0.6)),  # protocol v1 full scale, clamped to the first-run limits
        (-0.15, -0.8, (-0.10, -0.6)),
        (0.0, 0.0, (0.0, 0.0)),  # a pinch inside the dead zone drives at zero
        (0.12, -0.5, (0.12, -0.5)),
    ],
)
def test_the_command_is_clamped_to_the_limits(rover, clock, vx, wz, sent):
    s, _, base = rover
    s.on_teleop(drive(0, vx, wz))
    s.tick()
    assert base.sent == [sent]
    assert s.status()["base"] == "driving"


def test_a_disabled_rover_never_drives(clock):
    s, _, base = rover_session(clock, rover=False)
    for seq in range(5):
        s.on_teleop(drive(seq))
        step(s, clock)
        s.tick()

    assert base.sent == []
    assert link(s) == {"base": "idle", "arm": "holding", "fault": None}


def test_the_default_session_has_the_rover_disabled(clock):
    s = Session(K, C.JOINT_LIMITS_RAD, lambda q: None, lambda g: None, clock)
    s.on_joint_state(Q0)
    s.on_teleop_mode(True)
    s.on_odometry()
    s.on_connect()
    s.on_teleop(drive(0))
    s.tick()
    assert s.status()["base"] == "idle"


def test_a_rover_absent_before_any_odometry_never_drives(clock):
    s, _, base = rover_session(clock, odometry=False)
    for seq in range(5):
        s.on_teleop(drive(seq))
        step(s, clock, odometry=False)
        s.tick()

    assert base.sent == []
    assert s.status()["base"] == "idle"


def test_a_pinch_held_when_the_rover_first_appears_waits_for_a_left_release(clock):
    s, _, base = rover_session(clock, odometry=False)
    s.on_teleop(drive(0))
    step(s, clock)  # the first odometry
    s.on_teleop(drive(1))
    s.tick()
    assert base.sent == []
    assert s.status()["base"] == "idle"

    s.on_teleop(drive(2, engaged=False))
    s.on_teleop(drive(3))
    s.tick()
    assert base.sent == [CMD]


def test_release_sends_one_zero(rover, clock):
    s, base = driving(rover, clock)
    s.on_teleop(drive(1, engaged=False))
    assert_one_zero_then_silence(s, base, clock)
    assert link(s) == {"base": "idle", "arm": "holding", "fault": None}


def test_base_stale_is_seen_by_tick_and_sends_one_zero(rover, clock):
    s, base = driving(rover, clock)
    step(s, clock, BASE_STALE_S - 0.03)  # 0.02 s already passed since the teleop
    s.tick()
    assert base.sent == [CMD]
    base.sent.clear()
    step(s, clock, 0.02)
    s.tick()
    assert_one_zero_then_silence(s, base, clock)
    assert link(s) == {"base": "idle", "arm": "holding", "fault": None}


def test_base_stale_resumes_on_the_next_engaged_teleop_without_release(rover, clock):
    s, base = driving(rover, clock)
    step(s, clock, BASE_STALE_S + 0.1)
    s.tick()
    assert base.sent == [ZERO]
    s.on_teleop(drive(1, 0.1, 0.0))
    s.tick()

    assert base.sent == [ZERO, (0.1, 0.0)]
    assert link(s) == {"base": "driving", "arm": "holding", "fault": None}


def test_malformed_teleop_does_not_keep_the_base_driving(rover, clock):
    s, base = driving(rover, clock)
    bad = drive(1)
    bad["base"]["vx"] = "fast"
    for _ in range(4):
        step(s, clock, 0.1)
        s.on_teleop(bad)
        s.tick()
    assert ZERO in base.sent
    assert s.status()["base"] == "idle"


def test_disconnect_sends_one_zero(rover, clock):
    s, base = driving(rover, clock)
    s.on_disconnect()
    assert_one_zero_then_silence(s, base, clock)


def test_no_teleop_drives_after_the_socket_closed(rover, clock):
    s, _, base = rover
    s.on_disconnect()
    s.on_teleop(drive(0))
    s.tick()
    assert base.sent == []
    assert s.status()["base"] == "idle"


def test_a_replacement_socket_sends_one_zero_and_needs_both_hands_open(rover, clock):
    s, base = driving(rover, clock)
    assert s.on_connect()
    assert_one_zero_then_silence(s, base, clock)

    s.on_teleop(drive(0))
    s.tick()
    assert base.sent == [ZERO]
    s.on_teleop(drive(1, engaged=False))
    s.on_teleop(drive(2))
    s.tick()
    assert base.sent == [ZERO, CMD]


def test_link_timeout_sends_one_zero_and_reports_base_fault(rover, clock):
    s, base = driving(rover, clock)
    step(s, clock, TIMEOUT_S + 0.1)
    s.tick()
    assert_one_zero_then_silence(s, base, clock)
    assert link(s) == TIMED_OUT


def test_after_a_link_timeout_both_hands_must_release_before_the_base_drives(rover, clock):
    s, base = driving(rover, clock)
    step(s, clock, TIMEOUT_S + 0.1)
    s.tick()
    s.on_teleop(drive(1))  # clears the fault, left still held
    s.on_teleop(drive(2, engaged=False, arm=True))  # left open, right held
    s.on_teleop(drive(3))
    s.tick()
    assert base.sent == [ZERO]
    assert link(s) == {"base": "idle", "arm": "holding", "fault": None}

    s.on_teleop(drive(4, engaged=False))
    s.on_teleop(drive(5))
    s.tick()
    assert base.sent == [ZERO, CMD]
    assert s.status()["base"] == "driving"


def lapse_odometry(s, clock, seq0=1):
    """Engaged teleops keep arriving every 0.1 s while the rover's odometry stops."""
    seq = seq0
    for _ in range(round(ROVER_ABSENT_S / 0.1) + 1):
        step(s, clock, 0.1, odometry=False)
        s.on_teleop(drive(seq))
        s.tick()
        seq += 1
    return seq


def test_rover_absent_sends_one_zero_and_reports_idle(rover, clock):
    s, base = driving(rover, clock)
    lapse_odometry(s, clock)
    assert base.sent.count(ZERO) == 1
    assert base.sent[-1] == ZERO
    assert link(s) == {"base": "idle", "arm": "holding", "fault": None}


def test_rover_absent_just_under_the_limit_keeps_driving(rover, clock):
    s, base = driving(rover, clock)
    for seq in range(1, 5):
        step(s, clock, 0.1, odometry=False)
        s.on_teleop(drive(seq))
        s.tick()
    step(s, clock, ROVER_ABSENT_S - 0.43, odometry=False)  # 0.47 s since the last odometry
    s.on_teleop(drive(5))
    s.tick()
    assert base.sent == [CMD] * 5
    assert s.status()["base"] == "driving"


def test_after_the_rover_returns_the_left_hand_must_be_seen_open(rover, clock):
    s, base = driving(rover, clock)
    seq = lapse_odometry(s, clock)
    assert base.sent[-1] == ZERO
    base.sent.clear()
    s.on_teleop(drive(seq, engaged=False))  # open while still absent: does not count
    step(s, clock)  # the rover is back
    s.on_teleop(drive(seq + 1))
    s.tick()
    assert base.sent == []
    assert s.status()["base"] == "idle"

    s.on_teleop(drive(seq + 2, engaged=False))
    s.on_teleop(drive(seq + 3))
    s.tick()
    assert base.sent == [CMD]
    assert s.status()["base"] == "driving"


def test_on_shutdown_sends_one_zero_while_driving(rover, clock):
    s, base = driving(rover, clock)
    s.on_shutdown()
    assert_one_zero_then_silence(s, base, clock)


def test_on_shutdown_sends_one_zero_while_idle(clock):
    s, _, base = rover_session(clock, rover=False)
    s.on_shutdown()
    assert base.sent == [ZERO]


def test_the_arm_tracks_on_while_the_rover_is_absent(rover, clock):
    s, motor, _ = rover
    s.on_teleop(drive(0, arm=True))
    for seq in range(1, 10):
        step(s, clock, 0.1, odometry=False)
        s.on_teleop(drive(seq, arm=True, position=[0.001 * seq, 0.0, 0.0]))
        s.tick()

    assert len(motor.joints) == 10
    assert link(s) == {"base": "idle", "arm": "tracking", "fault": None}


def test_base_stale_does_not_stop_the_arm(rover, clock):
    s, motor, base = rover
    s.on_teleop(drive(0, arm=True))
    step(s, clock, BASE_STALE_S + 0.2)
    s.tick()
    assert base.sent == [ZERO]
    assert link(s) == {"base": "idle", "arm": "tracking", "fault": None}
    s.on_teleop(drive(1, arm=True, engaged=False, position=[0.01, 0.0, 0.0]))

    assert len(motor.joints) == 2
    assert s.status()["arm"] == "tracking"


def test_releasing_the_left_clutch_leaves_the_arm_tracking(rover, clock):
    s, motor, base = rover
    s.on_teleop(drive(0, arm=True))
    s.tick()
    s.on_teleop(drive(1, arm=True, engaged=False, position=[0.01, 0.0, 0.0]))

    assert base.sent == [CMD, ZERO]
    assert len(motor.joints) == 2
    assert link(s) == {"base": "idle", "arm": "tracking", "fault": None}


def test_disconnect_keeps_the_arm_and_the_link_timeout_as_before(rover, clock):
    s, motor, _ = rover
    s.on_teleop(drive(0, arm=True))
    s.on_disconnect()
    assert s.status()["arm"] == "tracking"
    step(s, clock, TIMEOUT_S + 0.1)
    s.tick()
    assert link(s) == TIMED_OUT

    s.on_connect()  # a replacement: the right hand must be seen open again
    s.on_teleop(drive(0, arm=True, engaged=False))
    assert len(motor.joints) == 1


# --- base limits (the bridge's launch settings) ---


@pytest.mark.parametrize("limits", [(0.20, 0.10, 0.6), (0.35, 0.15, 0.8), (0.01, 0.01, 0.01)])
def test_base_limits_up_to_protocol_v1_are_accepted(limits):
    check_base_limits(*limits)


@pytest.mark.parametrize(
    ("limits", "name"),
    [
        ((0.36, 0.10, 0.6), "base_max_vx"),
        ((0.20, 0.16, 0.6), "base_max_reverse"),
        ((0.20, 0.10, 0.81), "base_max_wz"),
        ((0.0, 0.10, 0.6), "base_max_vx"),
        ((0.20, -0.10, 0.6), "base_max_reverse"),
        ((0.20, 0.10, math.nan), "base_max_wz"),
        ((0.20, 0.10, math.inf), "base_max_wz"),
    ],
)
def test_base_limits_not_positive_or_above_protocol_v1_are_refused(limits, name):
    with pytest.raises(ValueError, match=name):
        check_base_limits(*limits)
