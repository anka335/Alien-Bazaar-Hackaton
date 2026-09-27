"""The Mission tab's backend: Start runs the whole mission, Stop stops the rover and the arm.

On the real rig (`run` without `--sim`): the real Leo Rover and its OAK-D over rosbridge
(`nav.real`) and the real arm (this process's `System`). Per sock: the fast approach
(`nav.hunt.approach_sock`, SAM3 when configured) stops the front bumper `gap_m` before the
sock, which puts it in the arm's floor zone; then the load run (the state machine in the load
mode, the same as the Load tab's Start) picks every sock in reach and ends at home; only then
does the rover drive again. A load run that ends in an error or a hold ends the mission with
the rover stopped. At `capacity` socks, or with none found: GO TO BOX (the station's tag
`nav.boxes.target_id`, the remembered layout `nav.boxes.memory` if any), after driving back on
the odometry to 1.3 m in front of the station as seen at the start (**start the rover facing the
station**, 1-2 m from it; not seen: back to the start point). Tag not found: back off, again.
Parked: the unload run (the unload mode, stage B's loop) sorts the cargo box into the bins.

On the sim (`run --sim`): `mission.run_mission` (the nav world and fresh arm worlds in turn,
D-052); the dashboard's own arm world stays idle.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from pathlib import Path

import cv2
from pydantic import BaseModel

from sorter.core.config import DEFAULT_CONFIG_DIR
from sorter.core.types import Command, OperatorMode, Phase

log = logging.getLogger(__name__)

GAP_M = 0.10  # bumper to the sock's near edge: its center ~0.35-0.45 m ahead, in the arm's zone
LOAD_TIMEOUT_S = 600.0
UNLOAD_TIMEOUT_S = 1800.0  # up to ~90 s per sock
STATION_LOOK_M = 1.3  # back at the station: this far in front of its target tag


class Stopped(Exception):
    pass


class MissionControl:
    def __init__(self, system, sm, sim: bool, config_dir: str | Path = DEFAULT_CONFIG_DIR):
        self.system, self.sm, self.sim = system, sm, sim
        self.cfg = system.cfg
        self.config_dir = config_dir
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        # idle / searching / loading / to_station / unloading / done / stopped / error
        self.phase = "idle"
        self.note = ""
        self.cargo: dict[str, int] = {}
        self.sorted: dict[str, int] = {}  # unload: socks in the bin of their color
        self.stops = 0
        self.events: deque[dict] = deque(maxlen=60)
        self._jpeg: bytes = b""
        self._t0 = 0.0
        self.make_session = None  # tests: a session over the nav sim instead of the real Leo

    # --- the API ---

    def status(self) -> dict:
        with self._lock:
            return {
                "phase": self.phase,
                "note": self.note,
                "running": self.running,
                "real": not self.sim,
                "cargo": dict(self.cargo),
                "sorted": dict(self.sorted),
                "loaded": sum(self.cargo.values()),
                "stops": self.stops,
                "events": list(self.events),
            }

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, capacity: int = 6, gap_m: float = GAP_M, detector: str | None = None) -> None:
        if self.running:
            raise RuntimeError("the mission is running: Stop first")
        if self.sm.mode != "idle":
            raise RuntimeError("the arm's run is going: stop it in the Load tab first")
        self._stop.clear()
        with self._lock:
            self.cargo, self.sorted, self.stops, self.note = {}, {}, 0, ""
            self.events.clear()
        self._t0 = time.time()
        target = self._run_sim if self.sim else self._run_real
        args = (target, capacity, gap_m, detector)
        self._thread = threading.Thread(target=self._guard, args=args, name="mission", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the rover (its command is cancelled at the next tick) and the arm's run."""
        self._stop.set()
        if self.sm.mode != "idle":
            self.system.hub.send(Command.STOP)

    def view(self) -> bytes:
        return self._jpeg

    # --- the loop ---

    def _event(self, kind: str, **data) -> None:
        e = {"event": kind, "t_s": round(time.time() - self._t0, 1)} | data
        log.info("mission %s", e)
        with self._lock:
            self.events.append(e)

    def _set(self, phase: str, note: str = "") -> None:
        with self._lock:
            self.phase, self.note = phase, note
        self._event(phase, note=note)

    def _frame(self, frame) -> None:
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR))
        if ok:
            self._jpeg = buf.tobytes()

    def _guard(self, target, *args) -> None:
        from sorter.nav.commands import Cancelled

        try:
            target(*args)
        except (Cancelled, Stopped):
            self._set("stopped", "stopped by the operator")
        except Exception as e:  # noqa: BLE001 - shown on the tab, the rover is stopped
            log.exception("mission failed")
            self._set("error", str(e))

    def _check(self) -> None:
        if self._stop.is_set():
            raise Stopped

    def _run_sim(self, capacity: int, gap_m: float, detector: str | None) -> None:
        from sorter.mission.mission import run_mission

        self._set("searching", "the nav sim and a fresh arm world per stop")

        def on_event(e: dict) -> None:
            kind = e.get("event")
            with self._lock:
                self.events.append(e)
                if kind == "stopped":
                    self.phase = "loading"
                elif kind == "loaded":
                    self.phase, self.cargo = "searching", dict(e.get("cargo", {}))
                    self.stops += 1
                elif kind == "to_station":
                    self.phase = "to_station"
                elif kind == "unloading":
                    self.phase = "unloading"
                elif kind == "unloaded":
                    self.sorted = dict(e.get("sorted", {}))

        report = run_mission(
            config_dir=self.config_dir,
            capacity=capacity,
            detector=detector or "classic",
            on_event=on_event,
            cancelled=self._stop.is_set,
            on_frame=self._frame,
        )
        st, un = report.station or {}, report.unload
        if not st.get("ok"):
            self._set("error", st.get("note", ""))
        elif un is not None:
            note = f"{un['right']} of {report.loaded} socks in the bin of their color"
            ok = un["end"] == "done" or un["right"] == report.loaded  # all sorted: the loop's end
            self._set("done" if ok else "error", f"{note} (unload loop: {un['end']})")
        else:
            self._set("done", st.get("note", ""))

    def _run_real(self, capacity: int, gap_m: float, detector: str | None) -> None:
        from sorter.mission.mission import drive_to
        from sorter.nav import detect
        from sorter.nav.boxes import approach_box, load_memory
        from sorter.nav.hunt import CachedDetector, _PoseTagger, _SkipVisited, approach_sock
        from sorter.nav.real import RealSession

        nav = self.cfg.nav
        sam = self.cfg.color_classifier.sam
        name = detector or ("sam3" if sam is not None else "classic")
        self._set("searching", f"connecting to the Leo at {nav.real.rosbridge_url}")
        session = (self.make_session or RealSession)(nav)
        rover = session.rover
        rover.cancelled = self._stop.is_set
        run = rover.run

        def framed(cmd, *a, **k):
            res = run(cmd, *a, **k)
            if res.frame is not None:
                self._frame(res.frame)
            return res

        rover.run = framed
        try:

            def odom():
                return rover.sim.odom

            det = _SkipVisited(CachedDetector(detect.make(name, None, sam, nav.sam_prompt)), odom)
            _PoseTagger(rover.camera, odom)
            b = nav.boxes
            station = self._station(rover, b)
            self._event("station", seen=station is not None)
            ended = "capacity"
            while sum(self.cargo.values()) < capacity:
                self._check()
                self._set("searching", f"detector {name}")
                fr = approach_sock(rover, det, gap_m)
                self._check()
                if not fr.ok or fr.sock is None:
                    ended = f"no sock: {fr.note}"
                    break
                det.visited.append(det.to_odom(fr.frame, fr.sock.x, fr.sock.y))
                self._event("at_sock", note=fr.note, commands=fr.commands)
                got = self._load()
                with self._lock:
                    self.stops += 1
                    for c, n in got.items():
                        self.cargo[c] = self.cargo.get(c, 0) + n
                self._event("loaded", got=got, cargo=dict(self.cargo))
            self._check()
            self._set("to_station", f"{ended}; driving back")
            if station is not None:  # in front of the station, as seen at the start
                sx, sy, nx, ny = station
                gx, gy = sx + nx * STATION_LOOK_M, sy + ny * STATION_LOOK_M
                drive_to(rover, rover.sim, gx, gy, heading=math.atan2(-ny, -nx))
            else:
                drive_to(rover, rover.sim, 0.0, 0.0, heading=0.0)
            self._set("to_station", "the station's tags")
            memory = load_memory(b.memory)
            res = approach_box(rover, b.target_id, b.stop_m, b.tag_size_m, memory)
            if not res.ok:  # too close, or the odometry drifted: back off, search again
                self._check()
                rover.run("forward", -0.5, 0.2)
                res = approach_box(rover, b.target_id, b.stop_m, b.tag_size_m, memory)
            if not res.ok:
                self._set("error", res.note)
                return
            self._event("at_station", note=res.note)
            if sum(self.cargo.values()):
                try:
                    got = self._arm_run(
                        OperatorMode.UNLOAD, "unloading", "the arm sorts into the bins"
                    )
                    end = ""
                except RuntimeError as e:  # e.g. B's loop on the emptied box: still count
                    got = {c.value: n for c, n in self.sm.counters.items() if n}
                    end = f" ({e})"
                with self._lock:
                    self.sorted = got
                n, total = sum(got.values()), sum(self.cargo.values())
                note = f"{n} of {total} socks in the bin of their color{end}"
                self._set("done" if n >= total else "error", note)
            else:
                self._set("done", "nothing to unload")
        finally:
            rover.run = run
            session.close()  # stops the rover

    def _station(self, rover, b) -> tuple[float, float, float, float] | None:
        """Where the station is (its target tag: x, y and the face's normal, odometry frame),
        if the rover sees it now; None: the mission goes back to its start instead."""
        from sorter.nav.boxes import detect_tags

        res = rover.run("look")
        tag = next((t for t in detect_tags(res.frame, b.tag_size_m) if t.id == b.target_id), None)
        if tag is None:
            return None
        o = rover.sim.odom
        c, s = math.cos(o.yaw), math.sin(o.yaw)
        return (
            o.x + c * tag.x - s * tag.y,
            o.y + s * tag.x + c * tag.y,
            c * tag.nx - s * tag.ny,
            s * tag.nx + c * tag.ny,
        )

    def _load(self) -> dict[str, int]:
        return self._arm_run(OperatorMode.LOAD, "loading", "the arm picks what is in reach")

    def _arm_run(self, mode: OperatorMode, phase: str, note: str) -> dict[str, int]:
        """One run of the arm with the rover standing: the state machine in `mode` until it is
        done; the arm ends at home. Returns its counters by color class (load: socks put in
        the box; unload: socks landed in the bin of their color)."""
        hub, sm = self.system.hub, self.sm
        self._set(phase, note)
        hub.set_mode(mode)
        hub.send(Command.START)
        t0 = time.time()
        while sm.mode == "idle" and time.time() - t0 < 5:  # the state machine takes it
            time.sleep(0.05)
        while True:
            self._check()
            if sm.mode == "idle":
                break
            if sm.phase in (Phase.ERROR, Phase.HELD):
                raise RuntimeError(f"the {mode} run stopped: {sm.phase.value} {sm.error or ''}")
            limit = UNLOAD_TIMEOUT_S if mode is OperatorMode.UNLOAD else LOAD_TIMEOUT_S
            if time.time() - t0 > limit:
                hub.send(Command.STOP)
                raise RuntimeError(f"the {mode} run took too long")
            time.sleep(0.1)
        if sm.phase is not Phase.DONE and sm.error:
            raise RuntimeError(f"the {mode} run ended: {sm.error}")
        return {c.value: n for c, n in sm.counters.items() if n}


class StartReq(BaseModel):
    capacity: int = 6
    gap_m: float = GAP_M
    detector: str | None = None


def router(mission: MissionControl):
    from fastapi import APIRouter, HTTPException, Response

    r = APIRouter(prefix="/api/mission")

    @r.get("/state")
    def state() -> dict:
        return mission.status()

    @r.post("/start")
    def start(req: StartReq) -> dict:
        try:
            mission.start(req.capacity, req.gap_m, req.detector)
        except RuntimeError as e:
            raise HTTPException(409, str(e)) from None
        return {"ok": True}

    @r.post("/stop")
    def stop() -> dict:
        mission.stop()
        return {"ok": True}

    @r.get("/view.jpg")
    def view() -> Response:
        data = mission.view()
        if not data:
            raise HTTPException(503, "no frame yet")
        return Response(data, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    return r
