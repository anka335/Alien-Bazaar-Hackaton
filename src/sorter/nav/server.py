"""The Rover tab's backend: a live navigation episode in a thread, driven over HTTP.

One thread owns the simulator and its renderer (OpenGL wants one thread). It runs one command
at a time in real time (`nav.realtime`) and, while a command runs, renders the OAK-D's RGB and
depth plus the debug views (chase, overview) as JPEGs at `nav.stream_fps`. HTTP handlers only
queue work and read the latest JPEGs.

    GET  /api/nav/state                  episode, busy, last result, log, odometry, truth
    GET  /api/nav/view/{rgb|depth|chase|overview}.jpg
    GET  /api/nav/stream/{kind}.mjpg     the same as an MJPEG stream
    POST /api/nav/reset    {scenario, seed, overrides}
    POST /api/nav/command  {name, args}  one of Rover.COMMANDS
    POST /api/nav/auto     {detector}    the approach algorithm
    POST /api/nav/run      {detector, gap_m}  RUN ROBOT: find the nearest sock, stop gap_m before it
    POST /api/nav/boxes/remember           save the AprilTag boxes in view (nav.boxes.memory)
    POST /api/nav/box      {target, stop_m}  GO TO BOX: stop the bumper stop_m from the tag
    POST /api/nav/stop                   stop the running command / algorithm
    POST /api/nav/detect   {detector}    detections on the last frame
"""

from __future__ import annotations

import asyncio
import logging
import math
import os
import queue
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from fastapi import APIRouter, FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, HTMLResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from sorter.nav import detect
from sorter.nav.camera import colorize_depth
from sorter.nav.commands import Cancelled, Rover
from sorter.nav.config import NavConfig
from sorter.nav.episode import Episode, apply_overrides, goal_pixels
from sorter.nav.scenario import PRESETS, make

os.environ.setdefault("MUJOCO_GL", "egl")
log = logging.getLogger(__name__)
VIEWS = ("rgb", "depth", "chase", "overview")
BOUNDARY = "frame"
LOG_KEEP = 60


class ResetRequest(BaseModel):
    scenario: str = "easy"
    seed: int = 0
    overrides: dict[str, str] = Field(default_factory=dict)


class CommandRequest(BaseModel):
    name: str
    args: dict = Field(default_factory=dict)


class AutoRequest(BaseModel):
    detector: str = "classic"


class RunRequest(BaseModel):
    detector: str | None = None  # default: sam3 on the real rover (if configured), else classic
    gap_m: float = Field(0.30, ge=0.0, le=1.0)


class BoxRequest(BaseModel):
    target: int | None = None  # default nav.boxes.target_id
    stop_m: float | None = Field(None, ge=0.05, le=1.0)  # default nav.boxes.stop_m


class NavLive:
    def __init__(self, base: NavConfig | None = None, sam_cfg=None, real: bool = False):
        self.base = base or NavConfig()
        self.sam_cfg = sam_cfg
        self.real = real  # the real Leo and OAK-D (`sorter.nav.real`), not the simulator
        self.jobs: queue.Queue = queue.Queue()
        self.lock = threading.Lock()
        self.jpeg: dict[str, bytes] = {}
        self.jpeg_seq = 0
        self.busy: str | None = None
        self.error: str | None = None
        self.log: list[dict] = []
        self.episode: Episode | None = None
        self.meta = {"scenario": "easy", "seed": 0, "overrides": {}}
        self.detections: list[dict] = []
        self._cancel = threading.Event()
        self._last_render = 0.0
        self._t_wall = 0.0
        self._t_sim = 0.0
        self.thread = threading.Thread(target=self._loop, name="nav-live", daemon=True)
        self.thread.start()
        self.submit("reset", ResetRequest())

    # --- the API side ---

    def submit(self, kind: str, payload) -> None:
        if kind in ("command", "auto", "run", "box", "remember") and self.busy:
            raise HTTPException(409, f"busy: {self.busy}")
        self.jobs.put((kind, payload))

    def stop(self) -> None:
        self._cancel.set()

    def state(self) -> dict:
        with self.lock:
            ep = self.episode
            out = {
                "episode": self.meta,
                "busy": self.busy,
                "error": self.error,
                "log": self.log[-LOG_KEEP:],
                "detections": self.detections,
                "scenarios": [] if self.real else list(PRESETS),
                "real": self.real,
                "commands": list(Rover.COMMANDS),
                "jpeg_seq": self.jpeg_seq,
            }
            if ep is not None:
                o = ep.sim.odom
                score = ep.score()
                if score is not None:  # the simulator: ground truth for the debug readout
                    x, y, yaw = ep.sim.true_pose()
                    sx, sy = score.sock_x_m, score.sock_y_m
                    out |= {
                        "truth": {
                            "x": x,
                            "y": y,
                            "yaw_deg": math.degrees(yaw),
                            "sock_rover": [sx, sy],
                        },
                        "score": score.summary(),
                    }
                out |= {
                    "odom": {
                        "x": o.x,
                        "y": o.y,
                        "yaw_deg": math.degrees(o.yaw),
                        "v": o.v,
                        "w": o.w,
                        "distance": o.distance,
                    },
                    "commands_done": ep.commands,
                    "sim_t": ep.sim.t,
                    "goal": {
                        "center_m": list(ep.cfg.goal.center_m),
                        "half_size_m": list(ep.cfg.goal.half_size_m),
                    },
                    "frame_size": (
                        [ep.rover.frame.K.width, ep.rover.frame.K.height]
                        if ep.rover.frame is not None
                        else [ep.cfg.camera.width, ep.cfg.camera.height]
                    ),
                }
                if self.real:
                    out["hardware"] = {
                        "rosbridge": ep.cfg.real.rosbridge_url,
                        "camera": getattr(ep, "camera_kind", ""),
                        "odometry": getattr(ep.sim, "odom_source", None),
                        "fault": getattr(ep.sim, "fault", None),
                        "max_mps": ep.cfg.real.max_linear_mps,
                        "max_rps": ep.cfg.real.max_angular_rps,
                    }
            return out

    def view(self, kind: str) -> bytes:
        with self.lock:
            return self.jpeg.get(kind, b"")

    # --- the sim thread ---

    def _loop(self) -> None:
        while True:
            kind, payload = self.jobs.get()
            self._cancel.clear()
            try:
                if kind == "reset":
                    self._reset(payload)
                elif kind == "command":
                    self._command(payload)
                elif kind == "auto":
                    self._auto(payload)
                elif kind == "run":
                    self._run(payload)
                elif kind == "box":
                    self._box(payload)
                elif kind == "remember":
                    self._remember()
                elif kind == "detect":
                    self._detect(payload)
                self.error = None
            except Cancelled:
                self._note({"command": "stop", "note": "stopped by the operator"})
                self._cancel.clear()  # else the stop below is cancelled too and kills this thread
                if self.episode:
                    self.episode.sim.set_cmd(0, 0)
                    try:
                        self.episode.rover.stop()
                    except Exception:  # noqa: BLE001 - the loop must survive
                        log.exception("stop after cancel failed")
            except Exception as e:  # noqa: BLE001 - shown in the panel, the loop goes on
                log.exception("nav job %s failed", kind)
                self.error = f"{kind}: {e}"
            finally:
                self.busy = None
                if self.episode:
                    self._render(force=True)

    def _reset(self, req: ResetRequest) -> None:
        self.busy = "reset"
        old = self.episode
        cfg = apply_overrides(self.base, req.overrides)
        if self.real:
            from sorter.nav.real import RealSession

            if old is not None:  # one connection to the rover and the camera at a time
                old.close()
                old = None
                self.episode = None
            ep = RealSession(cfg)
        else:
            ep = Episode(make(req.scenario, req.seed), cfg)
        ep.rover.look()
        ep.rover.on_tick = self._on_tick  # after the first frame: it paces and renders `episode`
        ep.rover.cancelled = self._cancel.is_set
        with self.lock:
            self.episode = ep
            self.meta = (
                {"scenario": "real rover", "seed": 0, "overrides": req.overrides}
                if self.real
                else {"scenario": req.scenario, "seed": req.seed, "overrides": req.overrides}
            )
            self.log = []
            self.detections = []
        if old is not None:
            old.close()

    def _command(self, req: CommandRequest) -> None:
        ep = self._need()
        self.busy = req.name
        self._pace_start()
        res = ep.rover.run(req.name, **req.args)
        ep.commands += 1
        self._note(res.summary())

    def _auto(self, req: AutoRequest) -> None:
        from sorter.nav.controller import Approach

        ep = self._need()
        self.busy = f"auto ({req.detector})"
        det = detect.make(req.detector, ep.camera, self.sam_cfg, ep.cfg.sam_prompt)
        ap = Approach(ep.rover, det, ep.cfg.goal)
        run = ep.rover.run

        def logged(name, *a, **k):
            res = run(name, *a, **k)
            ep.commands += 1
            self._note(res.summary() | {"auto": True})
            self._detect_now(det)
            return res

        ep.rover.run = logged
        self._pace_start()
        try:
            ok = ap.run()
        finally:
            ep.rover.run = run
        self._note({"command": "auto", "note": "done: sock in the zone" if ok else "gave up"})

    def _run(self, req: RunRequest) -> None:
        """RUN ROBOT: the fast approach (`hunt.approach_sock`)."""
        from sorter.nav.hunt import CachedDetector, approach_sock

        ep = self._need()
        if hasattr(ep.sim, "reset_odom"):  # the real rover: every run starts clean
            ep.sim.reset_odom()
        name = req.detector or ("sam3" if self.real and self.sam_cfg is not None else "classic")
        self.busy = f"RUN ROBOT ({name})"
        det = CachedDetector(detect.make(name, ep.camera, self.sam_cfg, ep.cfg.sam_prompt))
        run = ep.rover.run

        def logged(name, *a, **k):
            res = run(name, *a, **k)
            ep.commands += 1
            self._note(res.summary() | {"auto": True})
            self._detect_now(det)
            return res

        ep.rover.run = logged
        self._pace_start()
        try:
            r = approach_sock(ep.rover, det, req.gap_m)
        finally:
            ep.rover.run = run
        self._note({"command": "RUN ROBOT", "note": r.note, "ok": r.ok})

    def _tags_now(self) -> None:
        """The AprilTags in the last frame, shown like detections on the camera view."""
        from sorter.nav.boxes import detect_tags

        ep = self._need()
        tags = detect_tags(ep.rover.frame, ep.cfg.boxes.tag_size_m)
        with self.lock:
            self.detections = [
                {
                    "u": t.u,
                    "v": t.v,
                    "score": 1.0,
                    "distance_m": t.distance,
                    "bearing_deg": t.bearing_deg,
                    "source": f"tag {t.id}",
                }
                for t in tags
            ]

    def _remember(self) -> None:
        from sorter.nav.boxes import remember_boxes

        ep = self._need()
        self.busy = "remember boxes"
        res = ep.rover.look()
        b = ep.cfg.boxes
        mem = remember_boxes(res.frame, b.tag_size_m, b.target_id, b.memory)
        self._tags_now()
        self._note(
            {
                "command": "remember boxes",
                "ok": True,
                "note": f"tags {list(mem['tags'])} -> {b.memory}",
            }
        )

    def _box(self, req: BoxRequest) -> None:
        """GO TO BOX: `boxes.approach_box` on the target tag, with the remembered layout."""
        from sorter.nav.boxes import approach_box, load_memory

        ep = self._need()
        if hasattr(ep.sim, "reset_odom"):  # the real rover: every run starts clean
            ep.sim.reset_odom()
        b = ep.cfg.boxes
        target = b.target_id if req.target is None else req.target
        stop = b.stop_m if req.stop_m is None else req.stop_m
        self.busy = f"GO TO BOX (tag {target})"
        run = ep.rover.run

        def logged(name, *a, **k):
            res = run(name, *a, **k)
            ep.commands += 1
            self._note(res.summary() | {"auto": True})
            self._tags_now()
            return res

        ep.rover.run = logged
        self._pace_start()
        try:
            r = approach_box(
                ep.rover,
                target,
                stop,
                b.tag_size_m,
                load_memory(b.memory),
                ep.cfg.real.max_linear_mps,
            )
        finally:
            ep.rover.run = run
        self._note({"command": "GO TO BOX", "note": r.note, "ok": r.ok})

    def _detect(self, req: AutoRequest) -> None:
        ep = self._need()
        self.busy = f"detect ({req.detector})"
        self._detect_now(detect.make(req.detector, ep.camera, self.sam_cfg, ep.cfg.sam_prompt))

    def _detect_now(self, det) -> None:
        ep = self._need()
        dets = det.detect(ep.rover.frame)
        with self.lock:
            self.detections = [d.summary() for d in dets]

    def _need(self) -> Episode:
        if self.episode is None:
            raise RuntimeError("no episode")
        return self.episode

    def _note(self, entry: dict) -> None:
        with self.lock:
            self.log.append(entry | {"t": round(self.episode.sim.t, 2) if self.episode else 0})

    def _pace_start(self) -> None:
        self._t_wall, self._t_sim = time.monotonic(), self.episode.sim.t

    def _on_tick(self, sim) -> None:
        rt = self.episode.cfg.realtime
        if rt > 0:
            ahead = (sim.t - self._t_sim) / rt - (time.monotonic() - self._t_wall)
            if ahead > 0:
                time.sleep(ahead)
        self._render()

    def _render(self, force: bool = False) -> None:
        ep = self.episode
        now = time.monotonic()
        if not force and now - self._last_render < 1 / ep.cfg.stream_fps:
            return
        self._last_render = now
        frame = ep.camera.capture() if not force else ep.rover.frame or ep.camera.capture()
        rgb = frame.rgb.copy()
        poly = goal_pixels(frame, ep.cfg.goal)
        if poly is not None:
            cv2.polylines(rgb, [np.array(poly, np.int32)], True, (0, 255, 120), 1, cv2.LINE_AA)
        views = {
            "rgb": rgb,
            "depth": colorize_depth(frame.depth_mm),
            **(
                {}
                if self.real
                else {
                    "chase": ep.camera.render_view("chase"),
                    "overview": ep.camera.render_view("overview"),
                }
            ),
        }
        out = {}
        for k, img in views.items():
            ok, buf = cv2.imencode(
                ".jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 80]
            )
            if ok:
                out[k] = buf.tobytes()
        with self.lock:
            self.jpeg.update(out)
            self.jpeg_seq += 1


def command_specs() -> list[dict]:
    import inspect

    out = []
    for name in Rover.COMMANDS:
        fn = getattr(Rover, name)
        params = []
        for pn, prm in inspect.signature(fn).parameters.items():
            if pn == "self":
                continue
            ann = str(prm.annotation)
            kind = "bool" if "bool" in ann else "str" if "str" in ann else "number"
            default = None if prm.default is inspect.Parameter.empty else prm.default
            if isinstance(default, float) and math.isinf(default):
                default = None
            params.append(
                {
                    "name": pn,
                    "type": kind,
                    "default": default,
                    "required": prm.default is inspect.Parameter.empty,
                }
            )
        out.append({"name": name, "params": params, "doc": " ".join((fn.__doc__ or "").split())})
    return out


def router(live_factory) -> APIRouter:
    """The /api/nav routes; `live_factory()` gives the NavLive (made on first use)."""
    r = APIRouter(prefix="/api/nav")

    @r.get("/commands")
    def commands() -> list[dict]:
        """Every command with its parameters (name, default, type) and its doc: the panel's form."""
        return command_specs()

    @r.get("/state")
    def state() -> dict:
        return live_factory().state()

    @r.get("/view/{kind}.jpg")
    def view(kind: str) -> Response:
        if kind not in VIEWS:
            raise HTTPException(404)
        data = live_factory().view(kind)
        if not data:
            raise HTTPException(503, "no frame yet")
        return Response(data, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @r.get("/stream/{kind}.mjpg")
    def stream(kind: str, request: Request) -> StreamingResponse:
        if kind not in VIEWS:
            raise HTTPException(404)
        live = live_factory()

        async def gen():
            # every part is closed by the next boundary right away: a browser shows a part only
            # once its boundary arrives, so an idle rover would otherwise show nothing; the
            # latest frame is sent again every second so the stream stays alive
            yield f"--{BOUNDARY}\r\n".encode()
            seq, sent = -1, 0.0
            while not await request.is_disconnected():
                now = time.monotonic()
                if live.jpeg_seq != seq or now - sent > 1.0:
                    seq, sent = live.jpeg_seq, now
                    data = live.view(kind)
                    if data:
                        yield (
                            (
                                f"Content-Type: image/jpeg\r\nContent-Length: {len(data)}\r\n\r\n"
                            ).encode()
                            + data
                            + f"\r\n--{BOUNDARY}\r\n".encode()
                        )
                await asyncio.sleep(0.05)

        return StreamingResponse(
            gen(), media_type=f"multipart/x-mixed-replace; boundary={BOUNDARY}"
        )

    @r.post("/reset")
    def reset(req: ResetRequest) -> dict:
        if req.scenario not in PRESETS:
            raise HTTPException(400, f"unknown scenario {req.scenario}")
        live = live_factory()
        if live.busy:
            live.stop()
        live.submit("reset", req)
        return {"ok": True}

    @r.post("/command")
    def command(req: CommandRequest) -> dict:
        if req.name not in Rover.COMMANDS:
            raise HTTPException(400, f"unknown command {req.name}")
        live_factory().submit("command", req)
        return {"ok": True}

    @r.post("/auto")
    def auto(req: AutoRequest) -> dict:
        live_factory().submit("auto", req)
        return {"ok": True}

    @r.post("/run")
    def run_robot(req: RunRequest) -> dict:
        live_factory().submit("run", req)
        return {"ok": True}

    @r.post("/box")
    def go_to_box(req: BoxRequest) -> dict:
        live_factory().submit("box", req)
        return {"ok": True}

    @r.post("/boxes/remember")
    def remember() -> dict:
        live_factory().submit("remember", None)
        return {"ok": True}

    @r.post("/detect")
    def detect_(req: AutoRequest) -> dict:
        live_factory().submit("detect", req)
        return {"ok": True}

    @r.post("/stop")
    def stop() -> dict:
        live_factory().stop()
        return {"ok": True}

    return r


def lazy(base: NavConfig | None = None, sam_cfg=None, real: bool = False):
    """A factory that makes the NavLive once, on the first request."""
    holder: list[NavLive] = []
    lock = threading.Lock()

    def get() -> NavLive:
        with lock:
            if not holder:
                holder.append(NavLive(base, sam_cfg, real))
            return holder[0]

    return get


def standalone_app(
    base: NavConfig | None = None, config_dir: str = "config", real: bool = False
) -> FastAPI:
    """The admin panel's build with only the Rover tab's API (`python -m sorter.nav serve`)."""
    from sorter.dashboard.server import NOT_BUILT, WEB_DIR

    sam = None
    try:
        from sorter.core.config import load_config

        sam = load_config(Path(config_dir)).color_classifier.sam
    except Exception:  # noqa: BLE001 - SAM3 is optional here
        pass
    app = FastAPI(title="Leo Rover nav sim")
    app.include_router(router(lazy(base, sam, real)))
    if (WEB_DIR / "assets").is_dir():
        app.mount("/assets", StaticFiles(directory=WEB_DIR / "assets"), name="assets")

    @app.get("/{path:path}", include_in_schema=False)
    def page(path: str) -> Response:
        index = WEB_DIR / "index.html"
        if not index.is_file():
            return HTMLResponse(NOT_BUILT)
        return FileResponse(index, headers={"Cache-Control": "no-cache"})

    return app
