"""Session recorder for runs on the rig: everything needed to see afterwards what happened.

`run --record` (on by default without `--sim`) writes `data/sessions/<stamp>/`:
- `session.json`: start time (unix + ISO, to sync a phone video), git commit, argv, config;
- `sorter.log`: every log record at DEBUG, wall clock with ms;
- `telemetry.csv`: the arm at `hz`: measured and commanded joints, their gap, velocity,
  torque, gripper, motor temperature, fault, TCP; the state machine's phase and cycle;
- `events.jsonl`: status changes, the arm's and detectors' calls (args, result, duration,
  error), decisions (their frame in `decisions/`), marks typed in the terminal;
- `wrist_NNN.mp4` + `wrist_frames.csv`: the wrist camera, color | depth, time and phase
  burned in; a new file every `segment_s` so a crash loses at most one segment.

Press Enter in the terminal (optionally type a note first) to drop a mark in the log.
The run log (`data/runs/<run_id>/`) still keeps each decision's observation; `events.jsonl`
names the run ids. Nothing here changes what the system does; errors are logged, never raised.
"""

from __future__ import annotations

import csv
import functools
import json
import logging
import math
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from rebot_b601 import config as rc

from sorter.arm import kinematics as kin
from sorter.core.system import System
from sorter.orchestrator.runlog import to_jsonable

log = logging.getLogger(__name__)

ARM_CALLS = (
    "start", "shutdown", "home", "look", "go_to", "aim_camera", "pick", "drop_to_cargo",
    "drop_to_laundry", "hold", "recover", "move_joints", "move_tcp", "lift", "set_gripper",
    "release", "clear_fault",
)  # fmt: skip
NOISY_LOGGERS = ("PIL", "matplotlib", "asyncio", "httpcore", "urllib3", "websockets", "multipart")


class Recorder:
    def __init__(
        self,
        system: System,
        root: Path | str = "data/sessions",
        *,
        hz: float = 50.0,
        video_fps: float = 10.0,
        segment_s: float = 120.0,
    ):
        self.s = system
        self.dir = Path(root) / time.strftime("%Y%m%d-%H%M%S")
        self.dir.mkdir(parents=True, exist_ok=True)
        self.hz, self.video_fps, self.segment_s = hz, video_fps, segment_s
        self.t0 = time.time()
        self._stop = threading.Event()
        self._events_lock = threading.Lock()
        self._events = open(self.dir / "events.jsonl", "a", buffering=1)  # noqa: SIM115
        self._threads: list[threading.Thread] = []
        self._log_handler: logging.Handler | None = None
        self._root_level: int | None = None
        self._marks = 0
        self._in_flight: dict[int, tuple[str, float]] = {}  # traced calls under way
        self._dumped: set[int] = set()
        self._rois = {z: list(v.roi) for z, v in system.cfg.views.items()}

    # --- lifecycle ---

    def start(self, argv: list[str] | None = None) -> Recorder:
        self._start_log()
        self._write_session(argv or sys.argv)
        self._trace(self.s.arm, "arm", ARM_CALLS, brief_args=False)
        self._trace(self.s.floor_detector, "floor_detector", ("detect",), brief_args=True)
        self._trace(self.s.box_detector, "box_detector", ("detect", "classify"), brief_args=True)
        self._trace(self.s.observer, "observer", ("observe", "observe_point"), brief_args=False)
        for name, fn in (("telemetry", self._telemetry_loop), ("video", self._video_loop)):
            t = threading.Thread(target=self._guard(fn), name=f"rec-{name}", daemon=True)
            t.start()
            self._threads.append(t)
        if sys.stdin is not None and sys.stdin.isatty():
            threading.Thread(target=self._marks_loop, name="rec-marks", daemon=True).start()
        print(f"Recording to {self.dir}  (Enter in this terminal = a mark)", flush=True)
        log.info("recording to %s", self.dir)
        return self

    def close(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=3)
        self.event("session_end", duration_s=round(time.time() - self.t0, 3))
        with self._events_lock:
            self._events.close()
        if self._log_handler is not None:
            logging.getLogger().removeHandler(self._log_handler)
            self._log_handler.close()
        if self._root_level is not None:
            logging.getLogger().setLevel(self._root_level)
        print(f"Recorded: {self.dir}", flush=True)

    # --- events ---

    def event(self, kind: str, **data: Any) -> None:
        now = time.time()
        rec = {"t": round(now, 3), "rel": round(now - self.t0, 3), "wall": _wall(now), "type": kind}
        try:
            line = json.dumps(rec | to_jsonable(data), default=str)
        except (TypeError, ValueError) as e:
            line = json.dumps(rec | {"unserializable": str(e)})
        with self._events_lock:
            if not self._events.closed:
                self._events.write(line + "\n")

    def mark(self, note: str = "") -> None:
        self._marks += 1
        log.warning("MARK %d %s", self._marks, note)
        self.event("mark", n=self._marks, note=note)

    # --- setup ---

    def _start_log(self) -> None:
        root = logging.getLogger()
        if root.level > logging.DEBUG or root.level == logging.NOTSET:
            level = root.level or logging.WARNING
            for h in root.handlers:  # the console keeps its level
                if h.level == logging.NOTSET:
                    h.setLevel(level)
            self._root_level = root.level
            root.setLevel(logging.DEBUG)
        for name in NOISY_LOGGERS:
            logging.getLogger(name).setLevel(logging.INFO)
        h = logging.FileHandler(self.dir / "sorter.log")
        h.setLevel(logging.DEBUG)
        fmt = "%(asctime)s %(levelname)-7s [%(threadName)s] %(name)s: %(message)s"
        h.setFormatter(_MsFormatter(fmt))
        root.addHandler(h)
        self._log_handler = h

    def _write_session(self, argv: list[str]) -> None:
        cfg = self.s.cfg.model_dump(mode="json")
        sam = cfg.get("color_classifier", {}).get("sam", {})
        if sam.get("api_key"):
            sam["api_key"] = "***"
        info = {
            "start_unix": self.t0,
            "start_wall": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(self.t0))
            + f".{int(self.t0 % 1 * 1000):03d}",
            "argv": argv,
            "git": _git(),
            "backends": cfg.get("backends"),
            "telemetry_hz": self.hz,
            "video_fps": self.video_fps,
            "config": cfg,
        }
        (self.dir / "session.json").write_text(json.dumps(info, indent=2, default=str))

    def _trace(self, obj: Any, source: str, names: tuple[str, ...], *, brief_args: bool) -> None:
        """Wrap `obj`'s methods (on the instance) to log each call: args, result, time, error."""
        for name in names:
            fn = getattr(obj, name, None)
            if not callable(fn):
                continue
            try:
                setattr(obj, name, self._wrap(fn, f"{source}.{name}", brief_args))
            except (AttributeError, TypeError):
                log.debug("recorder: can't trace %s.%s", source, name)

    def _wrap(self, fn, call: str, brief_args: bool):
        @functools.wraps(fn)
        def traced(*args, **kwargs):
            shown = {"args": [type(a).__name__ for a in args]} if brief_args else {"args": args}
            self.event("call", call=call, **_short(shown | {"kwargs": kwargs}))
            t = time.monotonic()
            key = id(object())
            self._in_flight[key] = (call, t)
            try:
                out = fn(*args, **kwargs)
            except BaseException as e:
                self.event(
                    "raise", call=call, dur_s=round(time.monotonic() - t, 3),
                    error=f"{type(e).__name__}: {e}",
                )  # fmt: skip
                raise
            finally:
                self._in_flight.pop(key, None)
            self.event(
                "return", call=call, dur_s=round(time.monotonic() - t, 3), **_short({"result": out})
            )
            return out

        return traced

    # --- threads ---

    def _guard(self, fn):
        def run():
            try:
                fn()
            except Exception:
                log.exception("recorder thread %s died", fn.__name__)

        return run

    def _telemetry_loop(self) -> None:
        period = 1.0 / self.hz
        rebot = getattr(getattr(self.s.arm, "driver", None), "arm", None)
        with open(self.dir / "telemetry.csv", "w", newline="") as f:
            w = csv.writer(f)
            j = range(1, 7)
            w.writerow(
                ["t", "rel", "phase", "mode", "cycle", "moving", "torque", "fault"]
                + [f"q{i}" for i in j]
                + [f"qcmd{i}" for i in j]
                + ["err_max"]
                + [f"dq{i}" for i in j]
                + [f"tau{i}" for i in j]
                + ["grip_open", "grip_deg", "grip_target_deg", "temp_max"]
                + ["tcp_x", "tcp_y", "tcp_z"]
            )
            last_status, last_decision, rows = None, None, 0
            next_t = time.monotonic()
            while not self._stop.is_set():
                st = self.s.hub.status()
                key = (st.phase, st.next_phase, st.mode, st.run_id, st.cycle, st.error,
                       st.failures, tuple(st.counters.values()))  # fmt: skip
                if key != last_status:
                    last_status = key
                    self.event(
                        "status", phase=st.phase, next=st.next_phase, mode=st.mode,
                        run_id=st.run_id, cycle=st.cycle, counters=st.counters,
                        failures=st.failures, error=st.error,
                    )  # fmt: skip
                d = self.s.hub.decision()
                if d is not None and d is not last_decision:
                    last_decision = d
                    self._save_decision(d)
                self._check_hangs()
                row = self._arm_row(rebot)
                if row is not None:
                    now = time.time()
                    w.writerow([f"{now:.3f}", f"{now - self.t0:.3f}", st.phase.value, st.mode,
                                st.cycle] + row)  # fmt: skip
                    rows += 1
                    if rows % int(self.hz) == 0:
                        f.flush()
                next_t += period
                self._stop.wait(max(0.0, next_t - time.monotonic()))
                if time.monotonic() - next_t > 1.0:  # fell behind: don't burst to catch up
                    next_t = time.monotonic()

    def _check_hangs(self, after_s: float = 15.0) -> None:
        """A traced call under way for `after_s`: log every thread's stack, once per call."""
        now = time.monotonic()
        for key, (call, t) in list(self._in_flight.items()):
            if now - t < after_s or key in self._dumped:
                continue
            self._dumped.add(key)
            names = {t.ident: t.name for t in threading.enumerate()}
            stacks = "\n".join(
                f"--- {names.get(ident, ident)}\n" + "".join(traceback.format_stack(frame))
                for ident, frame in sys._current_frames().items()
            )
            log.warning("%s has run %.0f s; thread stacks:\n%s", call, now - t, stacks)
            self.event("hang", call=call, after_s=round(now - t, 1))

    def _arm_row(self, rebot) -> list | None:
        if rebot is None or not getattr(rebot, "connected", False):
            return None
        with rebot._lock:
            m = rebot._meas
            if m is None:
                return None
            q, dq, tau, temps = m.q.copy(), m.dq.copy(), m.tau.copy(), m.temps.copy()
            grip = m.grip_pos
            qc = rebot._q_cmd.copy()
            moving = rebot._traj is not None
            fault, enabled = rebot._fault, rebot._enabled
            grip_target = getattr(rebot, "_grip_target", float("nan"))
        tcp = kin.fk_tcp(q)[:3, 3]
        grip_open = grip / math.radians(rc.GRIPPER_OPEN_DEG) if rc.GRIPPER_ENABLED else float("nan")
        finite = temps[np.isfinite(temps)]
        deg = np.degrees
        return (
            [int(moving), int(enabled), fault or ""]
            + _f(deg(q), 2)
            + _f(deg(qc), 2)
            + _f([np.abs(deg(q - qc)).max()], 2)
            + _f(deg(dq), 1)
            + _f(tau, 2)
            + _f([grip_open], 3)
            + _f([math.degrees(grip), math.degrees(grip_target)], 1)
            + _f([finite.max() if finite.size else float("nan")], 1)
            + _f(tcp, 1)
        )

    def _save_decision(self, d) -> None:
        from sorter.dashboard.render import render_decision

        now = time.time()
        name = f"{_wall(now).replace(':', '')}_{d.phase.value}.jpg"
        try:
            (self.dir / "decisions").mkdir(exist_ok=True)
            cv2.imwrite(str(self.dir / "decisions" / name), render_decision(d, self._rois))
        except Exception as e:
            log.warning("recorder: can't save the decision frame: %s", e)
            name = None
        self.event("decision", phase=d.phase, summary=d.summary, image=name)

    def _video_loop(self) -> None:
        period = 1.0 / self.video_fps
        writer, seg, seg_t0, size, last_seq, idx = None, -1, 0.0, None, None, 0
        with open(self.dir / "wrist_frames.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["segment", "index", "t", "rel", "frame_seq", "phase"])
            try:
                while not self._stop.wait(period):
                    frame = self.s.camera.latest()
                    if frame is None or frame.seq == last_seq:
                        continue
                    last_seq = frame.seq
                    now = time.time()
                    phase = self.s.hub.status().phase.value
                    img = _compose(frame, now, phase, self.t0)
                    if writer is None or now - seg_t0 >= self.segment_s:
                        if writer is not None:
                            writer.release()
                        seg, seg_t0, idx, size = seg + 1, now, 0, (img.shape[1], img.shape[0])
                        path = self.dir / f"wrist_{seg:03d}.mp4"
                        writer = cv2.VideoWriter(
                            str(path), cv2.VideoWriter_fourcc(*"mp4v"), self.video_fps, size
                        )
                    if (img.shape[1], img.shape[0]) != size:
                        img = cv2.resize(img, size)
                    writer.write(img)
                    w.writerow([seg, idx, f"{now:.3f}", f"{now - self.t0:.3f}", frame.seq, phase])
                    idx += 1
                    if idx % 20 == 0:
                        f.flush()
            finally:
                if writer is not None:
                    writer.release()

    def _marks_loop(self) -> None:
        while not self._stop.is_set():
            try:
                line = sys.stdin.readline()
            except (OSError, ValueError):
                return
            if not line:  # EOF
                return
            self.mark(line.strip())


# --- helpers ---


class _MsFormatter(logging.Formatter):
    def formatTime(self, record, datefmt=None):  # noqa: N802
        return _wall(record.created)


def _wall(t: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t % 1 * 1000):03d}"


def _f(xs, nd: int) -> list[str]:
    return [f"{float(x):.{nd}f}" for x in xs]


def _short(d: dict[str, Any], limit: int = 600) -> dict[str, Any]:
    """JSON-safe values (arrays dropped), each at most `limit` characters."""
    out = {}
    for k, v in d.items():
        try:
            j = to_jsonable(v)
            s = json.dumps(j, default=str)
        except Exception:
            j = s = repr(v)
        out[k] = j if len(s) <= limit else s[:limit] + "…"
    return out


def _compose(frame, now: float, phase: str, t0: float) -> np.ndarray:
    """Color | depth (100..800 mm, turbo), time, phase and frame number on top."""
    color = np.ascontiguousarray(frame.color)
    d = frame.depth_mm.astype(np.float32)
    d8 = np.clip((d - 100.0) / 700.0 * 255.0, 0, 255).astype(np.uint8)
    depth = cv2.applyColorMap(d8, cv2.COLORMAP_TURBO)
    depth[frame.depth_mm == 0] = 0
    if depth.shape[:2] != color.shape[:2]:
        depth = cv2.resize(depth, (color.shape[1], color.shape[0]))
    img = np.hstack([color, depth])
    text = f"{_wall(now)}  +{now - t0:7.2f}s  {phase}  #{frame.seq}"
    cv2.rectangle(img, (0, 0), (img.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(img, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def _git() -> dict[str, Any]:
    def run(*cmd: str) -> str:
        try:
            return subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return ""

    return {
        "commit": run("git", "rev-parse", "HEAD"),
        "branch": run("git", "rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": run("git", "status", "--porcelain").splitlines(),
    }
