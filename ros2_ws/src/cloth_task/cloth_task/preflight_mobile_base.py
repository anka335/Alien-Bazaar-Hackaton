"""The mobile base pre-flight (#45, #58): lens stand-in -> robot bridge -> Twist -> zero on release.

Run it through `preflight_mobile_base.sh` at the repo root, which sources the build, sets the
private domain and runs the robot pytest first. This module is the part after the pytest:

    python3 -m cloth_task.preflight_mobile_base --repo <repo> --run-dir <dir> \
        --pgid-file <file> --results <file>

It starts, each in its own process group (listed in --pgid-file, so the shell script can stop
them if this process dies):

  1. a launch with base_max_vx:=0.5, which must be refused;
  2. the stack: hardware:=real driver_sim:=true enable_motors:=true run_task:=false
     spectacles:=true rover:=true spectacles_port:=9110 use_rviz:=false, then waits for the
     bridge's "teleop mode on";
  3. preflight_monitor (/joint_states gaps, /leo/cmd_vel, /leo/merged_odom) and rover_standin;
  4. the lens stand-in playing SCENARIO. It kills the rover stand-in (SIGKILL to its group) as
     phase c7-kill starts and starts it again as c7-restart starts;
  5. stops the stack with SIGINT (the bridge's zero Twist at shutdown), then the monitor and
     the rover stand-in, also on failure.

Then it reads the files back (load_run) and runs the checks of #45 (evaluate), plus the refused
launch. One line per check goes to --results; the exit code is 0 only if every check passed.

Twist times: the rover stand-in's receive times (its CSV) for every check but 7, which needs the
Twists sent while the stand-in is dead, so it uses the monitor's. Lens send times are the lens
stand-in's. All three are time.monotonic(), system-wide on Linux. Refuses to run unless
ROS_DOMAIN_ID=77 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

PORT = 9110
DOMAIN_ID = "77"
# Thresholds, from #45 "Robot: pre-flight". Do not loosen them.
RELEASE_ZERO_S = 0.050
SILENCE_ZERO_S = 0.350
DISCONNECT_ZERO_S = 0.100
ROVER_KILL_ZERO_S = 0.6
JS_GAP_MAX_S = 0.060
# The launch-set limits (the launch defaults), which the full protocol inputs must be clamped to
MAX_VX, MAX_REVERSE, MAX_WZ = 0.20, 0.10, 0.6
# The Twist that was due when a new command arrived may still carry the previous one
SETTLE_S = 0.050
EPS = 1e-9

LAUNCH = [
    "ros2",
    "launch",
    "cloth_task",
    "task.launch.py",
    "hardware:=real",
    "driver_sim:=true",
    "enable_motors:=true",
    "run_task:=false",
    "spectacles:=true",
    "rover:=true",
    f"spectacles_port:={PORT}",
    "use_rviz:=false",
]
TELEOP_ON = "teleop mode on"
TELEOP_FAILED = ("teleop mode not enabled", "cannot start")
LIMIT_REFUSED = "base limits out of range"


def _base(vx: float = 0.0, wz: float = 0.0) -> dict[str, Any]:
    return {"engaged": True, "vx": vx, "wz": wz}


# Right clutch closed and held still: zero delta, identity (ARM_OPEN's pose)
ARM_HELD = {"engaged": True}

# The lens stand-in's phases. Wire frames as the lens would send them (ADR 0012 for c8: the
# right hand awaiting release sends arm.engaged false).
SCENARIO: list[dict[str, Any]] = [
    {"name": "c1-open", "duration": 1.0},
    {"name": "c2-deadzone", "duration": 1.0, "base": _base()},
    {"name": "c3-forward", "duration": 1.0, "base": _base(vx=0.35)},
    {"name": "c3-left", "duration": 1.0, "base": _base(wz=0.8)},
    {"name": "c3-right", "duration": 1.0, "base": _base(wz=-0.8)},
    {"name": "c3-reverse", "duration": 1.0, "base": _base(vx=-0.15)},
    {"name": "c4-release", "duration": 1.0},
    {"name": "c5-drive", "duration": 1.0, "base": _base(vx=0.35)},
    {"name": "c5-silence", "duration": 0.5, "kind": "silence"},
    {"name": "c5-resume", "duration": 1.0, "base": _base(vx=0.35)},
    {"name": "c6-drive", "duration": 0.5, "base": _base(vx=0.35)},
    {"name": "c6-disconnect", "duration": 1.0, "kind": "disconnect"},
    # a new socket is a replacement: both hands must be seen open
    {"name": "c7-open", "duration": 1.0},
    {"name": "c7-drive", "duration": 1.0, "base": _base(vx=0.35)},
    {"name": "c7-kill", "duration": 2.0, "base": _base(vx=0.35)},  # rover stand-in killed
    {"name": "c7-restart", "duration": 8.0, "base": _base(vx=0.35)},  # started again: ~1-3 s
    {"name": "c7-open-one", "duration": 0.03},  # one frame at 30 Hz
    {"name": "c7-pinch", "duration": 1.0, "base": _base(vx=0.35)},
    {"name": "c8-open", "duration": 0.5},
    {"name": "c8-left-drive", "duration": 1.0, "base": _base(vx=0.35)},  # right closed, waiting
    {"name": "c8-left-release", "duration": 1.0},  # right still closed, still waiting
    {"name": "c8-right-open", "duration": 0.5},
    {"name": "c8-right-closed", "duration": 1.0, "arm": ARM_HELD},
    {"name": "c9-open", "duration": 0.5},
    {"name": "c9-hold", "duration": 10.0, "arm": ARM_HELD},
    {"name": "end-open", "duration": 0.5},
]
KILL_PHASE, RESTART_PHASE = "c7-kill", "c7-restart"


def scenario_phases() -> list[dict[str, Any]]:
    """SCENARIO as the lens stand-in's phase dicts (a deep copy)."""
    return json.loads(json.dumps(SCENARIO))


# --- the files of a run ---


@dataclass(frozen=True)
class Frame:
    t: float  # send time
    sock: int  # socket index, from 0
    seq: int
    phase: str
    base_engaged: bool
    vx: float
    wz: float
    arm_engaged: bool


@dataclass(frozen=True)
class Status:
    t: float  # receive time
    sock: int
    echo: int | None
    base: str
    arm: str
    fault: str | None


@dataclass(frozen=True)
class Twist:
    t: float  # receive time
    vx: float
    wz: float

    @property
    def zero(self) -> bool:
        return self.vx == 0.0 and self.wz == 0.0


@dataclass
class Run:
    frames: list[Frame] = field(default_factory=list)
    statuses: list[Status] = field(default_factory=list)
    phases: dict[str, float] = field(default_factory=dict)  # start times, in order
    lens_ok: bool = False
    twists: list[Twist] = field(default_factory=list)  # the rover stand-in's CSV
    watched: list[Twist] = field(default_factory=list)  # the monitor's /leo/cmd_vel
    odom: list[float] = field(default_factory=list)
    joint_states: list[float] = field(default_factory=list)
    events: dict[str, float] = field(default_factory=dict)

    def __post_init__(self):
        self._frame = {(f.sock, f.seq): f for f in self.frames}

    def frames_of(self, phase: str) -> list[Frame]:
        return [f for f in self.frames if f.phase == phase]

    def window(self, phase: str) -> tuple[float, float]:
        """[start, start of the next phase)."""
        names = list(self.phases)
        i = names.index(phase)
        end = self.phases[names[i + 1]] if i + 1 < len(names) else math.inf
        return self.phases[phase], end

    def echoes(self, phase: str, after: float = -math.inf) -> list[Status]:
        """Statuses whose echoSeq is a frame of `phase` (on its socket) sent after `after`."""
        out = []
        for s in self.statuses:
            f = self._frame.get((s.sock, s.echo))
            if f is not None and f.phase == phase and f.t > after:
                out.append(s)
        return out


def _between(twists: list[Twist], a: float, b: float) -> list[Twist]:
    return [x for x in twists if a <= x.t < b]


def load_lens_log(path: str | Path) -> tuple[list[Frame], list[Status], dict[str, float], bool]:
    frames, statuses, phases, ok = [], [], {}, False
    sock, phase = -1, ""
    with open(path) as f:
        for line in f:
            if not line.strip():
                continue
            e = json.loads(line)
            ev = e.get("event")
            if ev == "phase":
                phase = e["name"]
                phases[phase] = e["t"]
            elif ev == "connect":
                sock += 1
            elif ev == "sent":
                fr, b, a = e["frame"], e["frame"]["base"], e["frame"]["arm"]
                frames.append(
                    Frame(
                        e["t_send"],
                        sock,
                        fr["seq"],
                        phase,
                        b["engaged"],
                        b["vx"],
                        b["wz"],
                        a["engaged"],
                    )
                )
            elif ev == "status":
                s = e["status"]
                statuses.append(
                    Status(
                        e["t"], sock, s.get("echoSeq"), s.get("base"), s.get("arm"), s.get("fault")
                    )
                )
            elif ev == "done":
                ok = bool(e.get("ok"))
    return frames, statuses, phases, ok


def _read_csv(path: str | Path) -> list[list[str]]:
    with open(path, newline="") as f:
        return [row for row in csv.reader(f) if row]


def load_run(
    lens_log: str | Path, rover_csv: str | Path, monitor_csv: str | Path, events: str | Path
) -> Run:
    """A Run from the files; a missing or unreadable file leaves its part empty."""
    run = Run()
    with contextlib.suppress(OSError, ValueError, KeyError, TypeError):
        run.frames, run.statuses, run.phases, run.lens_ok = load_lens_log(lens_log)
    with contextlib.suppress(OSError, ValueError):
        run.twists = [Twist(float(t), float(vx), float(wz)) for t, vx, wz in _read_csv(rover_csv)]
    try:
        for row in _read_csv(monitor_csv):
            kind, t = row[0], float(row[1])
            if kind == "cmd_vel":
                run.watched.append(Twist(t, float(row[2]), float(row[3])))
            elif kind == "odom":
                run.odom.append(t)
            elif kind == "joint_states":
                run.joint_states.append(t)
    except (OSError, ValueError, IndexError):
        pass
    with contextlib.suppress(OSError, ValueError, AttributeError, TypeError):
        run.events = {k: float(v) for k, v in json.loads(Path(events).read_text()).items()}
    run.__post_init__()
    return run


# --- the checks ---


@dataclass(frozen=True)
class Result:
    key: str
    title: str
    ok: bool
    value: str

    def line(self) -> str:
        return f"check {self.key} {self.title}: {'PASS' if self.ok else 'FAIL'} ({self.value})"


class _Fail(Exception):
    """A check that cannot even be measured."""


def _need(cond: Any, why: str) -> None:
    if not cond:
        raise _Fail(why)


def _ms(dt: float) -> str:
    return f"{dt * 1000:.0f} ms"


def _first_zero(twists: list[Twist]) -> Twist | None:
    return next((x for x in twists if x.zero), None)


def _all(statuses: list[Status], pred: Callable[[Status], bool]) -> tuple[bool, str]:
    n = sum(1 for s in statuses if pred(s))
    return bool(statuses) and n == len(statuses), f"{n}/{len(statuses)}"


def check_1(run: Run) -> tuple[bool, str]:
    first = next((f for f in run.frames if f.base_engaged), None)
    _need(first is not None and run.frames, "no pinch frame sent")
    before = [x for x in run.twists if x.t < first.t]
    open_s = first.t - run.frames[0].t
    return not before, f"{len(before)} Twists before the first pinch, after {open_s:.2f} s open"


def check_2(run: Run) -> tuple[bool, str]:
    fr = run.frames_of("c2-deadzone")
    _need(fr, "no dead-zone frame sent")
    tw = _between(run.twists, fr[0].t, run.window("c2-deadzone")[1])
    nonzero = [x for x in tw if not x.zero]
    ok_st, st = _all(run.echoes("c2-deadzone"), lambda s: s.base == "driving")
    ok = bool(tw) and not nonzero and ok_st
    return ok, f"{len(tw)} Twists, {len(nonzero)} non-zero; {st} statuses base: driving"


def check_3(run: Run) -> tuple[bool, str]:
    ok, parts = True, []
    # phase, the axis commanded, its direction, the limit it must not pass; the other axis is 0
    for phase, axis, sign, limit in (
        ("c3-forward", "vx", 1, MAX_VX),
        ("c3-left", "wz", 1, MAX_WZ),
        ("c3-right", "wz", -1, MAX_WZ),
        ("c3-reverse", "vx", -1, MAX_REVERSE),
    ):
        fr = run.frames_of(phase)
        _need(fr, f"no {phase} frame sent")
        tw = _between(run.twists, fr[0].t + SETTLE_S, run.window(phase)[1])
        vals = [getattr(x, axis) for x in tw]
        other = [x.wz if axis == "vx" else x.vx for x in tw]
        label = f"{axis} {'max' if sign > 0 else 'min'}"
        extreme = (max if sign > 0 else min)(vals) if vals else math.nan
        good = (
            bool(tw)
            and all(not x.zero for x in tw)
            and all(0.0 < sign * v <= limit + EPS for v in vals)
            and all(v == 0.0 for v in other)
        )
        ok_st, st = _all(run.echoes(phase), lambda s: s.base == "driving")
        ok = ok and good and ok_st
        parts.append(f"{label} {extreme:.3f} over {len(tw)} Twists, {st} driving")
    return ok, "; ".join(parts)


def check_4(run: Run) -> tuple[bool, str]:
    fr = run.frames_of("c4-release")
    _need(fr, "no release frame sent")
    tw = _between(run.twists, fr[0].t, run.window("c4-release")[1])
    z = _first_zero(tw)
    _need(z, "no zero Twist after the release")
    after = [x for x in tw if x.t > z.t and not x.zero]
    dt = z.t - fr[0].t
    return dt <= RELEASE_ZERO_S and not after, (
        f"zero {_ms(dt)} after the release frame, {len(after)} non-zero after it"
    )


def check_5(run: Run) -> tuple[bool, str]:
    drive, resume = run.frames_of("c5-drive"), run.frames_of("c5-resume")
    _need(drive and resume, "no c5 frames sent")
    t_last, t_res = drive[-1].t, resume[0].t
    tw = _between(run.twists, t_last, t_res)
    _need(any(not x.zero for x in _between(run.twists, t_last - 0.1, t_last)), "not driving")
    z = _first_zero(tw)
    _need(z, "no zero Twist in the silence")
    moved = [x for x in tw if x.t > z.t and not x.zero]
    idle = [
        s for s in run.statuses if t_last < s.t < t_res and s.base == "idle" and s.fault is None
    ]
    back = [x for x in _between(run.twists, t_res, run.window("c5-resume")[1]) if not x.zero]
    ok_st, st = _all(run.echoes("c5-resume"), lambda s: s.base == "driving")
    dt = z.t - t_last
    ok = dt <= SILENCE_ZERO_S and not moved and bool(idle) and bool(back) and ok_st
    resumed = _ms(back[0].t - t_res) if back else "never"
    return ok, (
        f"zero {_ms(dt)} after the last frame, {len(idle)} statuses idle with fault null, "
        f"resumed {resumed} after the first frame with {st} driving"
    )


def check_6(run: Run) -> tuple[bool, str]:
    _need("c6-disconnect" in run.phases, "no disconnect phase")
    t_d = run.phases["c6-disconnect"]
    _need(any(not x.zero for x in _between(run.twists, t_d - 0.1, t_d)), "not driving")
    z = _first_zero(_between(run.twists, t_d, run.window("c6-disconnect")[1]))
    _need(z, "no zero Twist after the disconnect")
    dt = z.t - t_d
    return dt <= DISCONNECT_ZERO_S, f"zero {_ms(dt)} after the disconnect"


def check_7(run: Run) -> tuple[bool, str]:
    _need("rover_kill" in run.events and "rover_restart" in run.events, "rover not cycled")
    t_k, t_r = run.events["rover_kill"], run.events["rover_restart"]
    t_open = run.phases.get("c7-open-one", math.inf)
    _need(any(not x.zero for x in _between(run.watched, t_k - 0.1, t_k)), "not driving")
    z = _first_zero([x for x in run.watched if x.t >= t_k])
    _need(z, "no zero Twist after the kill")
    dt = z.t - t_k
    moved = [x for x in run.watched + run.twists if z.t < x.t < t_open and not x.zero]
    held = run.echoes(KILL_PHASE, after=z.t) + run.echoes(RESTART_PHASE, after=z.t)
    ok_idle, idle = _all(held, lambda s: s.base == "idle")
    back = next((t for t in run.odom if t > t_r), None)
    _need(back is not None and back < t_open, "the rover stand-in's odometry did not return")
    ok_held, held_back = _all(run.echoes(RESTART_PHASE, after=back), lambda s: s.base == "idle")
    one = run.frames_of("c7-open-one")
    ok_one = len(one) == 1 and not one[0].base_engaged
    pinch = run.frames_of("c7-pinch")
    _need(pinch, "no new pinch sent")
    drove = [x for x in _between(run.twists, pinch[0].t, run.window("c7-pinch")[1]) if not x.zero]
    ok_drive, drive = _all(run.echoes("c7-pinch"), lambda s: s.base == "driving")
    ok = (
        dt <= ROVER_KILL_ZERO_S
        and not moved
        and ok_idle
        and ok_held
        and ok_one
        and bool(drove)
        and ok_drive
    )
    return ok, (
        f"zero {_ms(dt)} after the kill, {len(moved)} non-zero while absent, {idle} idle; "
        f"odometry back {back - t_r:.2f} s after the restart, held pinch {held_back} idle; "
        f"{len(one)} open frame, then {len(drove)} non-zero Twists and {drive} driving"
    )


def check_8(run: Run) -> tuple[bool, str]:
    ok_ld, ld = _all(
        run.echoes("c8-left-drive"), lambda s: s.base == "driving" and s.arm == "holding"
    )
    rel = run.frames_of("c8-left-release")
    _need(rel, "no left release sent")
    # as in check 4, a tick due just before the release frame arrives still carries the command
    tw = _between(run.twists, rel[0].t, run.window("c8-left-release")[1])
    z = _first_zero(tw)
    _need(z, "no zero Twist after the left release")
    moved = [x for x in tw if x.t > z.t and not x.zero]
    ok_idle, idle = _all(run.echoes("c8-left-release"), lambda s: s.base == "idle")
    _need("c8-right-open" in run.phases, "no right open phase")
    right = _between(run.twists, run.phases["c8-right-open"], run.window("c8-right-closed")[1])
    ok_tr, tr = _all(run.echoes("c8-right-closed"), lambda s: s.arm == "tracking")
    ok = ok_ld and not moved and ok_idle and not right and ok_tr
    return ok, (
        f"left driving with right waiting: {ld} driving+holding; left release: zero "
        f"{_ms(z.t - rel[0].t)} after, {len(moved)} non-zero after it, {idle} idle; "
        f"right closed: {tr} tracking, {len(right)} Twists"
    )


def check_9(run: Run) -> tuple[bool, str]:
    hold = run.frames_of("c9-hold")
    _need(hold, "no hold frame sent")
    t0, t1 = hold[0].t, hold[-1].t
    during = [s for s in run.statuses if t0 <= s.t <= t1 + 0.2]
    timeouts = [s for s in during if s.fault == "timeout"]
    echoes = run.echoes("c9-hold")
    holding = [s for s in echoes if s.arm == "holding"]
    js = [t for t in run.joint_states if t0 <= t <= t1]
    _need(len(js) >= 2, "no /joint_states during the hold")
    gap = max(b - a for a, b in zip(js, js[1:], strict=False))
    stops = len(timeouts) + len(holding)
    ok = bool(echoes) and stops == 0 and gap <= JS_GAP_MAX_S
    return ok, (
        f"{stops} stops over {t1 - t0:.1f} s ({len(timeouts)} fault timeout, {len(holding)} "
        f"holding of {len(echoes)} echoes), max /joint_states gap {_ms(gap)}"
    )


def check_shutdown(run: Run) -> tuple[bool, str]:
    _need("stack_stop" in run.events, "the stack was not stopped")
    t_s = run.events["stack_stop"]
    z = _first_zero([x for x in run.watched if x.t >= t_s])
    _need(z, "no zero Twist after SIGINT")
    return True, f"zero {_ms(z.t - t_s)} after SIGINT to the stack"


CHECKS: list[tuple[str, str, Callable[[Run], tuple[bool, str]]]] = [
    ("1", "no Twist before the first pinch", check_1),
    ("2", "dead-zone pinch", check_2),
    ("3", "full inputs clamped", check_3),
    ("4", "release", check_4),
    ("5", "0.5 s silence", check_5),
    ("6", "disconnect", check_6),
    ("7", "rover stand-in killed", check_7),
    ("8", "one hand at a time", check_8),
    ("9", "arm regression", check_9),
    ("shutdown", "zero Twist at shutdown", check_shutdown),
]


def evaluate(run: Run) -> list[Result]:
    """Every check of #45 on the run, in order. A lens stand-in that failed fails them all."""
    out = []
    for key, title, fn in CHECKS:
        try:
            ok, value = fn(run)
        except _Fail as e:
            ok, value = False, str(e)
        except (ValueError, KeyError, IndexError) as e:  # a malformed run
            ok, value = False, f"cannot measure: {e!r}"
        if key != "shutdown" and not run.lens_ok:
            ok, value = False, f"the lens stand-in failed; {value}"
        out.append(Result(key, title, ok, value))
    return out


# --- the run ---


class Procs:
    """Child processes, each in its own session (process group = pid). Every pgid is appended
    to `pgid_file` before anything else happens, for the shell script's trap."""

    def __init__(self, pgid_file: Path, env: dict[str, str]):
        self.pgid_file, self.env = pgid_file, env
        self.procs: dict[str, subprocess.Popen] = {}

    def start(self, name: str, cmd: list[str], log: Path) -> subprocess.Popen:
        with open(log, "ab") as out:
            p = subprocess.Popen(
                cmd,
                stdin=subprocess.DEVNULL,
                stdout=out,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=self.env,
            )
        with open(self.pgid_file, "a") as f:
            f.write(f"{p.pid}\n")
        self.procs[name] = p
        return p

    @staticmethod
    def _signal(p: subprocess.Popen, sig: int) -> None:
        with contextlib.suppress(ProcessLookupError, PermissionError):
            os.killpg(p.pid, sig)

    def stop(self, name: str, sig: int = signal.SIGINT, timeout: float = 10.0) -> int | None:
        """Signals the group, waits, then SIGTERM and SIGKILL; also kills leftovers."""
        p = self.procs.pop(name, None)
        if p is None:
            return None
        for s, wait in ((sig, timeout), (signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0)):
            if p.poll() is not None:
                break
            self._signal(p, s)
            with contextlib.suppress(subprocess.TimeoutExpired):
                p.wait(wait)
        self._signal(p, signal.SIGKILL)  # anything left in the group
        return p.poll()

    def stop_all(self) -> None:
        for name in reversed(list(self.procs)):
            self.stop(name, timeout=90.0 if name == "stack" else 10.0)


def _wait_for(pred: Callable[[], bool], timeout: float, what: str) -> None:
    end = time.monotonic() + timeout
    while not pred():
        if time.monotonic() > end:
            raise RuntimeError(f"timed out after {timeout:.0f} s waiting for {what}")
        time.sleep(0.05)


def _text(path: Path) -> str:
    try:
        return path.read_text(errors="replace")
    except OSError:
        return ""


def _limit_refusal(procs: Procs, run_dir: Path) -> Result:
    log = run_dir / "launch_refused.log"
    p = procs.start("refused", [*LAUNCH, "base_max_vx:=0.5"], log)
    with contextlib.suppress(subprocess.TimeoutExpired):
        p.wait(60.0)
    rc = procs.stop("refused", timeout=30.0)
    refused = LIMIT_REFUSED in _text(log)
    ok = rc not in (None, 0) and refused
    return Result(
        "limits",
        "launch refuses base_max_vx:=0.5",
        ok,
        f"exit {rc}, {'refused' if refused else 'no refusal'} in the launch output",
    )


def _play(procs: Procs, run_dir: Path, files: dict[str, Path], rover_cmd: list[str]) -> int:
    """Runs the lens stand-in, cycling the rover stand-in on the kill and restart phases."""
    scenario = run_dir / "scenario.json"
    scenario.write_text(json.dumps(scenario_phases(), indent=1))
    events: dict[str, float] = {}
    lens = procs.start(
        "lens",
        [
            sys.executable,
            "-m",
            "cloth_task.spectacles_standin_lens",
            "--url",
            f"ws://127.0.0.1:{PORT}",
            "--scenario",
            str(scenario),
            "--log",
            str(files["lens_log"]),
        ],
        run_dir / "lens.out",
    )
    deadline = time.monotonic() + sum(p["duration"] for p in SCENARIO) + 60.0
    pos = 0
    try:
        while True:
            done = lens.poll() is not None
            try:
                with open(files["lens_log"]) as f:
                    f.seek(pos)
                    chunk = f.read()
            except FileNotFoundError:
                chunk = ""
            # only whole lines
            chunk = chunk[: chunk.rfind("\n") + 1]
            pos += len(chunk.encode())
            for line in chunk.splitlines():
                e = json.loads(line)
                if e.get("event") != "phase":
                    continue
                if e["name"] == KILL_PHASE:
                    events["rover_kill"] = time.monotonic()
                    procs.stop("rover", sig=signal.SIGKILL, timeout=5.0)
                elif e["name"] == RESTART_PHASE:
                    events["rover_restart"] = time.monotonic()
                    procs.start("rover", rover_cmd, run_dir / "rover.log")
            if done:
                return lens.returncode
            if time.monotonic() > deadline:
                raise RuntimeError("the lens stand-in did not finish")
            time.sleep(0.005)
    finally:
        files["events"].write_text(json.dumps(events))


def run_preflight(repo: Path, run_dir: Path, pgid_file: Path) -> list[Result]:
    env = dict(os.environ)
    if env.get("ROS_DOMAIN_ID") != DOMAIN_ID or env.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != (
        "LOCALHOST"
    ):
        raise SystemExit(
            "refusing: set ROS_DOMAIN_ID=77 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
        )
    run_dir.mkdir(parents=True, exist_ok=True)
    files = {
        "lens_log": run_dir / "lens.jsonl",
        "rover_csv": run_dir / "rover.csv",
        "monitor_csv": run_dir / "monitor.csv",
        "events": run_dir / "events.json",
    }
    for f in files.values():
        f.unlink(missing_ok=True)
    rover_exe = repo / "ros2_ws" / "install" / "cloth_task" / "lib" / "cloth_task" / "rover_standin"
    rover_cmd = [str(rover_exe), "--ros-args", "-p", f"csv_path:={files['rover_csv']}"]
    procs = Procs(pgid_file, env)
    results: list[Result] = []
    errors: list[str] = []
    try:
        results.append(_limit_refusal(procs, run_dir))
        launch_log = run_dir / "launch.log"
        launch_log.unlink(missing_ok=True)
        stack = procs.start("stack", LAUNCH, launch_log)

        def teleop_on() -> bool:
            text = _text(launch_log)
            if any(t in text for t in TELEOP_FAILED) or stack.poll() is not None:
                raise RuntimeError(f"the stack did not reach teleop mode (see {launch_log})")
            return TELEOP_ON in text

        _wait_for(teleop_on, 180.0, "teleop mode")
        procs.start(
            "monitor",
            [
                sys.executable,
                "-m",
                "cloth_task.preflight_monitor",
                "--csv",
                str(files["monitor_csv"]),
            ],
            run_dir / "monitor.log",
        )
        procs.start("rover", rover_cmd, run_dir / "rover.log")

        def monitor_sees(kind: str) -> Callable[[], bool]:
            return lambda: re.search(rf"^{kind},", _text(files["monitor_csv"]), re.M) is not None

        _wait_for(monitor_sees("joint_states"), 30.0, "/joint_states at the monitor")
        _wait_for(monitor_sees("odom"), 30.0, "the rover stand-in's odometry")
        time.sleep(1.0)  # the bridge sees the rover present
        rc = _play(procs, run_dir, files, rover_cmd)
        if rc != 0:
            errors.append(f"the lens stand-in exited {rc} (see {run_dir / 'lens.out'})")
        time.sleep(0.5)
        with open(files["events"]) as f:
            events = json.load(f)
        events["stack_stop"] = time.monotonic()
        files["events"].write_text(json.dumps(events))
        rc = procs.stop("stack", timeout=90.0)
        time.sleep(1.0)  # the monitor receives the shutdown zero
    except (RuntimeError, OSError, ValueError) as e:
        errors.append(str(e))
    finally:
        procs.stop_all()
    results += evaluate(load_run(**files))
    results += [Result("run", "pre-flight run", False, e) for e in errors]
    return results


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--repo", required=True, type=Path, help="the worktree root")
    ap.add_argument("--run-dir", required=True, type=Path, help="logs of this run")
    ap.add_argument("--pgid-file", required=True, type=Path, help="started process groups")
    ap.add_argument("--results", required=True, type=Path, help="one line per check")
    args = ap.parse_args(argv)
    # SIGTERM tears down like Ctrl+C
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        results = run_preflight(args.repo, args.run_dir, args.pgid_file)
    except KeyboardInterrupt:
        results = [Result("run", "pre-flight run", False, "interrupted")]
    lines = [r.line() for r in results]
    args.results.write_text("".join(line + "\n" for line in lines))
    print("\n".join(lines))
    return 0 if results and all(r.ok for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
