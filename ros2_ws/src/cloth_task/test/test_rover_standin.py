import csv
import math

import pytest

from cloth_task.rover_standin import CMD_TIMEOUT_S, CmdLog, DiffDrive

DT = 0.01  # the stand-in's 100 Hz odometry


def run(drive, t0, t1, *, every=None, cmd=None):
    """Steps drive at 100 Hz from t0 to t1, re-sending cmd every `every` s if given."""
    n = round((t1 - t0) / DT)
    next_cmd = t0 + every if every else math.inf
    for i in range(1, n + 1):
        t = t0 + i * DT
        if t >= next_cmd - 1e-9:
            drive.command(*cmd, t)
            next_cmd += every
        drive.step(t)


def test_it_stands_still_before_any_twist():
    d = DiffDrive(t0=0.0)
    run(d, 0.0, 1.0)
    assert (d.x, d.y, d.yaw, d.vx, d.wz) == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_a_twist_drives_it_straight_ahead():
    d = DiffDrive(t0=0.0)
    d.command(0.2, 0.0, 0.0)
    run(d, 0.0, 0.4)
    assert d.x == pytest.approx(0.08)
    assert (d.y, d.yaw) == (pytest.approx(0.0), pytest.approx(0.0))
    assert (d.vx, d.wz) == (0.2, 0.0)


def test_a_negative_vx_drives_it_backwards():
    d = DiffDrive(t0=0.0)
    d.command(-0.1, 0.0, 0.0)
    run(d, 0.0, 0.4)
    assert d.x == pytest.approx(-0.04)


def test_wz_turns_it_counter_clockwise_on_the_spot():
    d = DiffDrive(t0=0.0)
    d.command(0.0, 0.6, 0.0)
    run(d, 0.0, 0.4)
    assert d.yaw == pytest.approx(0.24)
    assert (d.x, d.y) == (pytest.approx(0.0), pytest.approx(0.0))


def test_vx_and_wz_together_follow_an_arc_to_the_left():
    v, w = 0.2, 0.5
    d = DiffDrive(t0=0.0)
    d.command(v, w, 0.0)
    run(d, 0.0, 0.4)
    assert d.x == pytest.approx(v / w * math.sin(w * 0.4), abs=1e-6)
    assert d.y == pytest.approx(v / w * (1 - math.cos(w * 0.4)), abs=1e-6)
    assert d.y > 0


def test_it_stops_half_a_second_after_the_last_twist():
    assert CMD_TIMEOUT_S == 0.5
    d = DiffDrive(t0=0.0)
    d.command(0.2, 0.6, 0.0)
    run(d, 0.0, 0.49)
    assert (d.vx, d.wz) == (0.2, 0.6)
    run(d, 0.49, 2.0)
    assert (d.vx, d.wz) == (0.0, 0.0)
    assert d.yaw == pytest.approx(0.6 * CMD_TIMEOUT_S)


def test_repeated_twists_keep_it_moving():
    d = DiffDrive(t0=0.0)
    d.command(0.2, 0.0, 0.0)
    run(d, 0.0, 1.5, every=0.1, cmd=(0.2, 0.0))
    assert d.x == pytest.approx(0.3)
    assert d.vx == 0.2


def test_a_zero_twist_stops_it_at_once():
    d = DiffDrive(t0=0.0)
    d.command(0.2, 0.0, 0.0)
    run(d, 0.0, 0.2)
    d.command(0.0, 0.0, 0.2)
    run(d, 0.2, 0.6)
    assert d.x == pytest.approx(0.04)
    assert d.vx == 0.0


def test_the_log_appends_receive_time_vx_and_wz(tmp_path):
    path = tmp_path / "cmd_vel.csv"
    log = CmdLog(str(path))
    log.write(12.5, 0.2, -0.6)
    log.write(12.52, 0.0, 0.0)
    log.close()
    log = CmdLog(str(path))  # a second run appends
    log.write(20.0, -0.1, 0.3)
    log.close()

    with open(path, newline="") as f:
        rows = [[float(v) for v in row] for row in csv.reader(f)]
    assert rows == [[12.5, 0.2, -0.6], [12.52, 0.0, 0.0], [20.0, -0.1, 0.3]]
