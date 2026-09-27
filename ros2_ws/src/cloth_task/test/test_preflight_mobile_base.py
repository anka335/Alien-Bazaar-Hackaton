"""The pre-flight's checks (#58), fed the files a run writes.

A green run is simulated with the real Session on a fake clock, at the bridge's rates (tick
50 Hz, status push 10 Hz, /joint_states 50 Hz, odometry 100 Hz), playing the pre-flight's own
SCENARIO. The simulation writes the lens stand-in log, the rover stand-in CSV, the monitor CSV
and the events file in their real formats, and the checks read them back with load_run(). A
broken bridge (no zero Twists, a long base stale, limits above the launch set, a link timeout
during the arm hold) must turn the matching checks red.
"""

import json
import os
import sys

import numpy as np

from cloth_task import spectacles_session
from cloth_task.preflight_mobile_base import (
    SCENARIO,
    evaluate,
    load_run,
    scenario_phases,
)
from cloth_task.spectacles_session import Session
from cloth_task.spectacles_standin_lens import load_scenario, teleop_frame


def _rebot():
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    sys.path.insert(0, os.path.join(repo, "rebot_b601"))
    from rebot_b601 import config, kinematics

    return kinematics, config


K, C = _rebot()
Q0 = np.radians([10.0, 50.0, 70.0, -20.0, 10.0, 5.0])
T0 = 1000.0
ROVER_START_S = 1.5  # a restarted rover stand-in's odometry is back after this


class Clock:
    def __init__(self):
        self.t = T0

    def __call__(self):
        return self.t


def simulate(
    out,
    *,
    drop_zeros=False,
    max_vx=0.20,
    max_reverse=0.10,
    max_wz=0.6,
    js_gap_at=None,
    stall_in_hold_s=0.0,
    rover_start_s=ROVER_START_S,
):
    """Plays SCENARIO against a Session; writes the run's files into `out`, returns the paths.

    drop_zeros: the bridge never publishes a zero Twist. js_gap_at: (t after the start of
    c9-hold, gap s) with no /joint_states. stall_in_hold_s: the lens goes silent for that long
    in the middle of c9-hold (a link stall). rover_start_s: the restarted rover stand-in's
    odometry reaches the bridge after that long."""
    clock = Clock()
    lens, rover_rows, mon_rows, events = [], [], [], {}
    state = {"rover": True, "rover_back": None, "sock": -1, "seq": 0, "open": False}

    def log(event, **kw):
        lens.append({"t": clock.t, "event": event, **kw})

    def on_base(vx, wz):
        if drop_zeros and vx == 0.0 and wz == 0.0:
            return
        mon_rows.append(("cmd_vel", clock.t, vx, wz))
        if state["rover"]:
            rover_rows.append((clock.t, vx, wz))

    s = Session(
        K,
        C.JOINT_LIMITS_RAD,
        lambda q: None,
        lambda g: None,
        clock,
        send_base=on_base,
        rover=True,
        max_vx=max_vx,
        max_reverse=max_reverse,
        max_wz=max_wz,
    )
    s.on_teleop_mode(True)
    s.on_joint_state(Q0)

    phases = load_scenario(scenario_phases())
    # frame times and phase starts on a 1 ms grid
    plan = []  # (ms, kind, payload)
    ms = 0
    hold_start = None
    for p in phases:
        plan.append((ms, "phase", p))
        if p.name == "c9-hold":
            hold_start = ms
        dur = round(p.duration * 1000)
        if p.kind == "send":
            k = 0
            while round(k * 1000 / 30) < dur:
                plan.append((ms + round(k * 1000 / 30), "frame", p))
                k += 1
        ms += dur
    total = ms + 500
    stall = set()
    if stall_in_hold_s:
        mid = hold_start + 5000
        stall = set(range(mid, mid + round(stall_in_hold_s * 1000)))
    js_gap = set()
    if js_gap_at is not None:
        at = hold_start + round(js_gap_at[0] * 1000)
        js_gap = set(range(at, at + round(js_gap_at[1] * 1000)))

    by_ms = {}
    for item in plan:
        by_ms.setdefault(item[0], []).append(item)

    def connect():
        log("connect", url="ws://127.0.0.1:9110")
        assert s.on_connect()
        state.update(sock=state["sock"] + 1, seq=0, open=True)

    for m in range(total):
        clock.t = T0 + m / 1000.0
        if state["rover_back"] is not None and clock.t >= state["rover_back"]:
            state["rover"], state["rover_back"] = True, None
            events["bridge_rover_back"] = clock.t  # the session gets odometry this ms
        if state["rover"] and m % 10 == 0:
            s.on_odometry()
            mon_rows.append(("odom", clock.t))
        if m % 20 == 0 and m not in js_gap:
            s.on_joint_state(Q0)
            mon_rows.append(("joint_states", clock.t, clock.t))
        for _, kind, p in by_ms.get(m, []):
            if kind == "phase":
                log("phase", index=phases.index(p), name=p.name, kind=p.kind)
                if p.name == "c7-kill":
                    events["rover_kill"] = clock.t
                    state["rover"] = False
                if p.name == "c7-restart":
                    events["rover_restart"] = clock.t
                    state["rover_back"] = clock.t + rover_start_s
                if p.kind == "disconnect" and state["open"]:
                    log("close", code=1000, reason="disconnect phase", by="client")
                    s.on_disconnect()
                    state["open"] = False
                elif p.kind != "disconnect" and not state["open"]:
                    connect()
            elif m not in stall:
                frame = teleop_frame(state["seq"], p, 0)
                log("sent", frame=frame, t_send=clock.t)
                state["seq"] += 1
                s.on_teleop(frame)
                log("status", status=s.status())
        if m % 20 == 0:
            s.tick()
        if m % 100 == 0 and state["open"]:
            log("status", status=s.status())
    log("close", code=1000, reason="scenario over", by="client")
    s.on_disconnect()
    log("done", ok=True)
    clock.t += 1.0
    events["stack_stop"] = clock.t
    clock.t += 0.01
    s.on_shutdown()

    paths = {
        "lens_log": out / "lens.jsonl",
        "rover_csv": out / "rover.csv",
        "monitor_csv": out / "monitor.csv",
        "events": out / "events.json",
    }
    paths["lens_log"].write_text("".join(json.dumps(e) + "\n" for e in lens))
    paths["rover_csv"].write_text("".join(f"{t!r},{vx!r},{wz!r}\n" for t, vx, wz in rover_rows))
    paths["monitor_csv"].write_text(
        "".join(",".join([r[0], *(repr(float(v)) for v in r[1:])]) + "\n" for r in mon_rows)
    )
    paths["events"].write_text(json.dumps(events))
    return paths


def results(paths):
    return {r.key: r for r in evaluate(load_run(**paths))}


CHECKS = [str(n) for n in range(1, 10)] + ["shutdown"]


def test_the_scenario_is_a_valid_lens_standin_scenario():
    phases = load_scenario(scenario_phases())
    assert [p.name for p in phases] == [p["name"] for p in SCENARIO]
    assert len({p.name for p in phases}) == len(phases)


def test_the_scenario_sends_protocol_v1_full_scale_inputs():
    by = {p["name"]: p for p in SCENARIO}
    assert by["c3-forward"]["base"]["vx"] == 0.35
    assert by["c3-reverse"]["base"]["vx"] == -0.15
    assert by["c3-left"]["base"]["wz"] == 0.8
    assert by["c9-hold"]["duration"] == 10.0


def test_a_green_run_passes_every_check(tmp_path):
    r = results(simulate(tmp_path))
    assert set(r) == set(CHECKS)
    failed = {k: v.value for k, v in r.items() if not v.ok}
    assert failed == {}


def test_check_lines_carry_pass_or_fail_and_a_value(tmp_path):
    for res in evaluate(load_run(**simulate(tmp_path))):
        line = res.line()
        assert ": PASS (" in line or ": FAIL (" in line
        assert line.startswith("check ")


def test_a_bridge_without_zero_twists_fails_the_stop_checks(tmp_path):
    r = results(simulate(tmp_path, drop_zeros=True))
    # the dead-zone pinch's zero commands are dropped too
    for k in ("2", "4", "5", "6", "7", "8", "shutdown"):
        assert not r[k].ok, k
    for k in ("1", "3", "9"):
        assert r[k].ok, (k, r[k].value)


def test_limits_above_the_launch_set_fail_the_clamp_check(tmp_path):
    r = results(simulate(tmp_path, max_vx=0.35, max_reverse=0.15, max_wz=0.8))
    assert not r["3"].ok
    assert r["2"].ok


def test_a_long_base_stale_fails_the_silence_check(tmp_path, monkeypatch):
    monkeypatch.setattr(spectacles_session, "BASE_STALE_S", 0.45)
    r = results(simulate(tmp_path))
    assert not r["5"].ok


def test_a_link_stall_during_the_arm_hold_fails_the_arm_regression(tmp_path):
    r = results(simulate(tmp_path, stall_in_hold_s=2.2))
    assert not r["9"].ok
    assert not r["9"].value.startswith("0 stops")


def test_a_joint_states_gap_over_60_ms_fails_the_arm_regression(tmp_path):
    r = results(simulate(tmp_path, js_gap_at=(3.0, 0.07)))
    assert not r["9"].ok
    assert r["1"].ok


def test_a_joint_states_gap_of_40_ms_passes(tmp_path):
    # one message of the 50 Hz stream missing
    r = results(simulate(tmp_path, js_gap_at=(3.0, 0.015)))
    assert r["9"].ok, r["9"].value


def test_missing_files_fail_every_check_instead_of_raising(tmp_path):
    paths = {
        "lens_log": tmp_path / "none.jsonl",
        "rover_csv": tmp_path / "none.csv",
        "monitor_csv": tmp_path / "none2.csv",
        "events": tmp_path / "none.json",
    }
    r = results(paths)
    assert set(r) == set(CHECKS)
    assert not any(v.ok for v in r.values())


def test_a_failed_lens_run_fails_the_checks(tmp_path):
    paths = simulate(tmp_path)
    lines = paths["lens_log"].read_text().splitlines()
    lines[-1] = json.dumps({"t": 0.0, "event": "done", "ok": False})
    paths["lens_log"].write_text("\n".join(lines) + "\n")
    r = results(paths)
    assert not any(v.ok for k, v in r.items() if k != "shutdown")


def test_the_clamp_check_reports_the_measured_limits(tmp_path):
    r = results(simulate(tmp_path))
    assert "0.200" in r["3"].value and "0.600" in r["3"].value and "-0.100" in r["3"].value


def _add_twist_after(paths, phase, dt, vx, wz):
    """A Twist in the rover stand-in's CSV `dt` after the first frame of `phase`."""
    run = load_run(**paths)
    t = run.frames_of(phase)[0].t + dt
    rows = paths["rover_csv"].read_text().splitlines()
    rows.append(f"{t!r},{vx!r},{wz!r}")
    rows.sort(key=lambda r: float(r.split(",")[0]))
    paths["rover_csv"].write_text("\n".join(rows) + "\n")


def test_a_tick_that_crosses_the_release_frame_still_passes(tmp_path):
    """On the robot, a tick can publish the last command after the release frame was sent and
    before it arrived: the zero follows it (seen in a pre-flight run, 1.4 ms then 1.9 ms)."""
    paths = simulate(tmp_path)
    for phase in ("c4-release", "c8-left-release"):
        t = load_run(**paths).frames_of(phase)[0].t
        rows = [r.split(",") for r in paths["rover_csv"].read_text().splitlines()]
        i = next(k for k, r in enumerate(rows) if float(r[0]) >= t and float(r[1]) == 0.0)
        rows[i][0] = repr(t + 0.0019)  # the zero, delayed
        rows.insert(i, [repr(t + 0.0014), "0.2", "0.0"])  # the tick before it
        paths["rover_csv"].write_text("".join(",".join(r) + "\n" for r in rows))
    r = results(paths)
    assert r["4"].ok and r["8"].ok, (r["4"].value, r["8"].value)


def test_a_non_zero_twist_after_the_release_zero_fails(tmp_path):
    paths = simulate(tmp_path)
    _add_twist_after(paths, "c8-left-release", 0.2, 0.2, 0.0)
    _add_twist_after(paths, "c4-release", 0.2, 0.2, 0.0)
    r = results(paths)
    assert not r["4"].ok and not r["8"].ok


def test_two_open_frames_after_the_rover_returns_fail_the_rover_check(tmp_path):
    """Check 7 asks for exactly one left-open frame before the new pinch."""
    paths = simulate(tmp_path)
    events = [json.loads(line) for line in paths["lens_log"].read_text().splitlines()]
    i = next(
        k
        for k, e in enumerate(events)
        if e["event"] == "sent"
        and e["frame"]["base"]["engaged"] is False
        and k > next(j for j, x in enumerate(events) if x.get("name") == "c7-open-one")
    )
    extra = json.loads(json.dumps(events[i]))
    extra["t"] += 0.001
    extra["t_send"] += 0.001
    extra["frame"]["seq"] = 10_000
    events.insert(i + 1, extra)
    paths["lens_log"].write_text("".join(json.dumps(e) + "\n" for e in events))
    r = results(paths)
    assert not r["7"].ok


def test_a_rover_the_bridge_sees_again_only_after_the_restart_phase_fails_check_7(tmp_path):
    r = results(simulate(tmp_path, rover_start_s=12.5))
    assert not r["7"].ok
    assert "did not return" in r["7"].value or "did not log the rover present" in r["7"].value


def test_a_bridge_that_never_logs_the_rover_present_again_fails_check_7(tmp_path):
    """The monitor sees the odometry, the bridge does not (the race of the first final run)."""
    paths = simulate(tmp_path)
    events = json.loads(paths["events"].read_text())
    del events["bridge_rover_back"]
    paths["events"].write_text(json.dumps(events))
    r = results(paths)
    assert not r["7"].ok
    assert "did not log the rover present again" in r["7"].value


def test_check_7_reports_when_the_bridge_saw_the_rover_again(tmp_path):
    r = results(simulate(tmp_path, rover_start_s=3.0))
    assert r["7"].ok, r["7"].value
    assert "rover present at the bridge 3.0" in r["7"].value
