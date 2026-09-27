"""The base-only Spectacles session: protocol v1's left hand and its stops."""

import pytest

from sorter.spectacles.session import (
    BASE_STALE_S,
    LINK_TIMEOUT_S,
    ROVER_ABSENT_S,
    BaseSession,
    Limits,
    parse_teleop,
)


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def teleop(seq=0, engaged=False, vx=0.0, wz=0.0, arm=False, **extra):
    return {
        "v": 1,
        "type": "teleop",
        "seq": seq,
        "timestamp": 0,
        "base": {"engaged": engaged, "vx": vx, "wz": wz},
        "arm": {"engaged": arm, "position": [0, 0, 0], "orientation": [0, 0, 0, 1], "gripper": 0},
        **extra,
    }


@pytest.fixture
def rig():
    clock = Clock()
    s = BaseSession(Limits(), clock)
    s.on_odom()
    s.on_connect()
    return s, clock


def advance(s, clock, dt, odom=True):
    clock.t += dt
    if odom:
        s.on_odom()


def test_parse_rejects_malformed():
    assert parse_teleop(teleop()) is not None
    assert parse_teleop({**teleop(), "v": 2}) is None
    assert parse_teleop({**teleop(), "seq": True}) is None
    assert (
        parse_teleop({**teleop(), "base": {"engaged": True, "vx": float("nan"), "wz": 0}}) is None
    )
    bad_q = teleop()
    bad_q["arm"]["orientation"] = [0, 0, 0, 0]
    assert parse_teleop(bad_q) is None
    assert parse_teleop({"v": 1, "type": "status"}) is None


def test_first_socket_drives_at_once_and_clamps(rig):
    s, _ = rig
    s.on_message(teleop(0, True, 0.35, -0.8))
    assert s.command() == (True, 0.20, -0.6)
    s.on_message(teleop(1, True, -0.15, 0.1))
    assert s.command() == (True, -0.10, 0.1)
    st = s.status()
    assert (st["base"], st["arm"], st["fault"], st["echoSeq"]) == ("driving", "holding", None, 1)


def test_open_clutch_sends_zero(rig):
    s, _ = rig
    s.on_message(teleop(0, False, 0.3, 0.3))
    assert s.command() == (False, 0.0, 0.0)
    assert s.status()["base"] == "idle"


def test_base_stale_stops_and_resumes_without_release(rig):
    s, clock = rig
    s.on_message(teleop(0, True, 0.1))
    advance(s, clock, BASE_STALE_S + 0.01)
    assert s.command()[0] is False
    assert s.status()["fault"] is None
    s.on_message(teleop(1, True, 0.1))
    assert s.command() == (True, 0.1, 0.0)


def test_link_timeout_faults_and_needs_a_left_release(rig):
    s, clock = rig
    s.on_message(teleop(0, True, 0.1))
    advance(s, clock, LINK_TIMEOUT_S)
    st = s.status()
    assert (st["base"], st["arm"], st["fault"]) == ("fault", "fault", "timeout")
    s.on_message(teleop(1, True, 0.1))  # clears the fault, but the held pinch does not drive
    assert s.status()["fault"] is None
    assert s.command()[0] is False
    s.on_message(teleop(2, False))
    s.on_message(teleop(3, True, 0.1))
    assert s.command() == (True, 0.1, 0.0)


def test_a_silent_client_faults_two_seconds_after_accept(rig):
    s, clock = rig
    advance(s, clock, LINK_TIMEOUT_S)
    assert s.status()["fault"] == "timeout"


def test_socket_close_stops_at_once(rig):
    s, _ = rig
    s.on_message(teleop(0, True, 0.1))
    s.on_disconnect()
    assert s.command() == (False, 0.0, 0.0)


def test_a_later_socket_needs_a_release(rig):
    s, _ = rig
    s.on_message(teleop(0, True, 0.1))
    s.on_connect()
    s.on_message(teleop(0, True, 0.1))
    assert s.command()[0] is False
    s.on_message(teleop(1, False))
    s.on_message(teleop(2, True, 0.1))
    assert s.command()[0] is True


def test_rover_absent_refuses_the_clutch_and_needs_a_release_after(rig):
    s, clock = rig
    s.on_message(teleop(0, True, 0.1))
    advance(s, clock, ROVER_ABSENT_S + 0.01, odom=False)
    s.on_message(teleop(1, True, 0.1))
    assert s.command()[0] is False
    assert s.status()["base"] == "idle"
    s.on_message(teleop(2, False))  # open while absent does not count
    s.on_odom()
    s.on_message(teleop(3, True, 0.1))
    assert s.command()[0] is False
    s.on_message(teleop(4, False))
    s.on_message(teleop(5, True, 0.1))
    assert s.command()[0] is True


def test_no_odometry_ever_never_drives():
    clock = Clock()
    s = BaseSession(Limits(), clock)
    s.on_connect()
    s.on_message(teleop(0, True, 0.1))
    assert s.command()[0] is False


def test_the_right_hand_and_reset_do_not_move_the_base(rig):
    s, _ = rig
    s.on_message(teleop(0, False, arm=True, reset=True))
    assert s.command() == (False, 0.0, 0.0)
    assert s.status()["arm"] == "holding"


def test_limits_above_the_protocol_are_refused():
    with pytest.raises(ValueError):
        Limits(max_vx=0.4)
    with pytest.raises(ValueError):
        Limits(max_wz=0)
