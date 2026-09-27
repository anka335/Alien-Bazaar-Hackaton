"""Dashboard: FastAPI server for the React admin panel (`frontend/`, built into `web/`). It
depends only on the Hub, plus the setup controls behind the manual and calibrate modes.

The HTTP API is in docs/architecture.md → Dashboard HTTP API.
"""

from __future__ import annotations

import asyncio
import contextlib
import dataclasses
import math
import threading
import time
from collections.abc import AsyncIterator, Callable, Mapping
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Request, WebSocket
from fastapi.encoders import jsonable_encoder
from fastapi.responses import FileResponse, HTMLResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from rebot_b601.assets import ASSETS_DIR as TWIN_ASSETS_DIR

from sorter.camera.config import ViewConfig
from sorter.core.errors import SorterError, WrongMode
from sorter.core.hub import Hub, SpeedControl
from sorter.core.types import Command, Decision, OperatorMode, Status, Zone
from sorter.dashboard.calibrate import CalibrateControl
from sorter.dashboard.config import DashboardConfig
from sorter.dashboard.manual import Busy, ManualControl
from sorter.dashboard.modes import SETUP, ModeSwitch
from sorter.dashboard.render import PHASE_LABELS, encode_jpeg, placeholder, render_decision

WEB_DIR = Path(__file__).parent / "web"  # `npm run build` in frontend/ (not in git)
BOUNDARY = "frame"
# the admin panel's tabs: each serves the single page, the front end routes it
PAGES = ("/", "/load", "/unload", "/manual", "/calibrate", "/3d", "/rover")
NOT_BUILT = """<!doctype html><meta charset="utf-8"><title>Sorter</title>
<body style="font:16px system-ui;margin:3rem;max-width:40rem">
<h1>The dashboard isn't built</h1>
<p>Build it once (Node 20+), then reload:</p>
<pre>cd frontend && npm install && npm run build</pre>
<p>The API runs meanwhile: <a href="/api/status">/api/status</a>.</p>"""


class CommandRequest(BaseModel):
    cmd: str


class SpeedRequest(BaseModel):
    speed_scale: float


class ManualRequest(BaseModel):
    action: str  # go | tour_next | tour_reset | jog | gripper | release | clear_fault | save
    pose: str | None = None
    joint: int | None = None  # 0..5
    delta_deg: float = 0.0
    open: bool = True


class CalibrateRequest(BaseModel):
    # goto_mark | goto_view | detect | click | delete_click | clear_clicks | save_mount
    # | compute_look | goto_look | save_look | calibrate (save_mount + compute_look + save_look)
    action: str
    mark: str | None = None
    u: float = 0.0
    v: float = 0.0
    index: int = 0  # a view or a click
    pose: str | None = None


def speed_json(speed: SpeedControl | None) -> dict[str, float] | None:
    if speed is None:
        return None
    return {"speed_scale": speed.speed_scale, "max_speed_scale": speed.max_speed_scale}


class ModeRequest(BaseModel):
    mode: str  # load | unload | manual | calibrate


def status_json(
    s: Status,
    speed: dict[str, float] | None = None,
    operator: OperatorMode = OperatorMode.LOAD,
) -> dict[str, Any]:
    """`Status` as JSON, plus `now` (monotonic, same clock as `Event.t`) for event ages,
    `speed` (`speed_json`, null without a speed control) and `operator` (the operator mode)."""
    return {
        **jsonable_encoder(dataclasses.asdict(s)),
        "now": time.monotonic(),
        "speed": speed,
        "operator": operator.value,
    }


class Frames:
    """JPEGs of the decision frame and the live feed. Each is encoded once and cached."""

    def __init__(self, hub: Hub, cfg: DashboardConfig, rois: Mapping[Zone, list]):
        self.hub, self.cfg, self.rois = hub, cfg, rois
        self._lock = threading.Lock()
        self._decision: tuple[Decision | None, bytes] | None = None
        self._live: tuple[int | None, bytes] | None = None

    def decision(self) -> bytes:
        d = self.hub.decision()
        with self._lock:
            if self._decision is None or self._decision[0] is not d:
                img = render_decision(d, self.rois) if d else placeholder("No decision yet")
                self._decision = (d, encode_jpeg(img, self.cfg.jpeg_quality))
            return self._decision[1]

    def live(self) -> bytes:
        f = self.hub.live_frame()
        seq = f.seq if f else None
        with self._lock:
            if self._live is None or self._live[0] != seq:
                img = f.color if f else placeholder("No camera frame")
                self._live = (seq, encode_jpeg(img, self.cfg.jpeg_quality))
            return self._live[1]


def _part(jpeg: bytes) -> bytes:
    # The boundary goes right after each frame, so browsers show it without waiting for the next.
    head = f"Content-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n".encode()
    return head + jpeg + f"\r\n--{BOUNDARY}\r\n".encode()


async def mjpeg(
    request: Request, next_jpeg: Callable[[], bytes], period_s: float
) -> AsyncIterator[bytes]:
    """multipart/x-mixed-replace body: a frame every `period_s` until the client disconnects."""
    yield f"--{BOUNDARY}\r\n".encode()
    while not await request.is_disconnected():
        yield _part(await asyncio.to_thread(next_jpeg))
        await asyncio.sleep(period_s)


def create_app(
    hub: Hub,
    cfg: DashboardConfig,
    views: Mapping[Zone, ViewConfig] | None = None,
    manual: ManualControl | None = None,
    calibrate: CalibrateControl | None = None,
    modes: ModeSwitch | None = None,
    web_dir: Path = WEB_DIR,
    nav: Callable | None = None,
) -> FastAPI:
    """`views` gives the zone ROIs drawn on the decision frame (`cfg.views`). `manual` and
    `calibrate`: the controls of the manual and calibrate modes, switched by `modes` (made
    from `manual` if not given); without them the dashboard has the run modes only."""
    app = FastAPI(title="Sorter")
    if nav is not None:  # the Rover tab: the navigation sim (sorter.nav), made on first use
        from sorter.nav.server import router as nav_router

        app.include_router(nav_router(nav))
    rois = {z: list(v.roi) for z, v in (views or {}).items()}
    frames = Frames(hub, cfg, rois)
    period_s = 1 / cfg.stream_fps
    if modes is None and manual is not None:
        modes = ModeSwitch(hub, manual)

    # the arm's CAD meshes, vendored in rebot_b601 (D-012: no CDN)
    app.mount("/twin-assets", StaticFiles(directory=TWIN_ASSETS_DIR), name="twin-assets")
    if (web_dir / "assets").is_dir():  # hashed names: cached for good
        app.mount("/assets", StaticFiles(directory=web_dir / "assets"), name="assets")

    def page() -> Response:
        index = web_dir / "index.html"
        if not index.is_file():
            return HTMLResponse(NOT_BUILT)
        # revalidated on every load, so a new build shows up on a plain reload
        return FileResponse(index, headers={"Cache-Control": "no-cache"})

    for path in PAGES:
        app.add_api_route(path, page, methods=["GET"], include_in_schema=False)

    @app.get("/favicon.svg", include_in_schema=False)
    def favicon() -> Response:
        icon = web_dir / "favicon.svg"
        if not icon.is_file():
            raise HTTPException(404)
        return FileResponse(icon)

    @app.get("/api/meta")
    def meta() -> dict:
        """What the panel needs once: the phase labels, the modes this server has."""
        return {
            "phase_labels": {p.value: label for p, label in PHASE_LABELS.items()},
            "modes": [m.value for m in OperatorMode if modes is not None or m not in SETUP],
            "calibrate": calibrate is not None,
        }

    # --- operator mode ---

    def mode_json() -> dict:
        return {"mode": hub.mode().value, "busy": manual.busy if manual else False}

    @app.get("/api/mode")
    def mode_state() -> dict:
        return mode_json()

    @app.post("/api/mode")
    def set_mode(req: ModeRequest) -> dict:
        try:
            mode = OperatorMode(req.mode)
        except ValueError:
            raise HTTPException(400, f"unknown mode {req.mode!r}") from None
        if mode in SETUP and modes is None:
            raise HTTPException(404, "no manual control on this server")
        if mode is OperatorMode.CALIBRATE and calibrate is None:
            raise HTTPException(404, "no calibration on this server")
        try:
            if modes is not None:
                modes.set(mode)
            else:
                hub.set_mode(mode)
        except WrongMode as e:
            raise HTTPException(409, str(e)) from None
        except SorterError as e:  # the motors didn't come on
            raise HTTPException(500, str(e)) from None
        return mode_json()

    def in_mode(*allowed: OperatorMode):
        """The mode guard (409 in another mode), or 404 without the setup controls."""
        if modes is None:
            raise HTTPException(404, "no manual control on this server")
        return modes.guard(*allowed)

    # --- manual control (manual and calibrate modes) ---

    def manual_source() -> ManualControl:
        if manual is None:
            raise HTTPException(404, "no manual control on this server")
        return manual

    @app.get("/api/manual")
    def manual_state() -> dict:
        m = manual_source()
        try:
            with in_mode(*SETUP):
                return m.state()
        except WrongMode as e:
            raise HTTPException(409, str(e)) from None

    @app.post("/api/manual")
    def manual_action(req: ManualRequest) -> dict:
        m = manual_source()
        try:
            with in_mode(*SETUP):
                match req.action:
                    case "go":
                        m.go(req.pose or "")
                    case "tour_next":
                        m.tour_next()
                    case "tour_reset":
                        m.tour_reset()
                    case "jog":
                        m.jog(-1 if req.joint is None else req.joint, math.radians(req.delta_deg))
                    case "gripper":
                        m.gripper(req.open)
                    case "release":
                        m.release()
                    case "clear_fault":
                        m.clear_fault()
                    case "save":
                        return {"ok": True, "line": m.save_pose(req.pose or "")}
                    case _:
                        raise HTTPException(400, f"unknown action {req.action!r}")
        except (Busy, WrongMode) as e:
            raise HTTPException(409, str(e)) from None
        except (ValueError, OSError) as e:
            raise HTTPException(400, str(e)) from None
        return {"ok": True}

    # --- camera calibration (calibrate mode) ---

    def calibrate_source() -> CalibrateControl:
        if calibrate is None:
            raise HTTPException(404, "no calibration on this server")
        return calibrate

    @app.get("/api/calibrate")
    def calibrate_state() -> dict:
        c = calibrate_source()
        try:
            with in_mode(*SETUP):
                return c.state()
        except WrongMode as e:
            raise HTTPException(409, str(e)) from None

    @app.post("/api/calibrate")
    def calibrate_action(req: CalibrateRequest) -> dict:
        c = calibrate_source()
        out: dict[str, Any] = {"ok": True}
        try:
            with in_mode(OperatorMode.CALIBRATE):
                match req.action:
                    case "goto_mark":
                        c.goto_mark(req.mark or "")
                    case "goto_view":
                        c.goto_view(req.index)
                    case "click":
                        c.click(req.mark or "", req.u, req.v)
                    case "delete_click":
                        c.delete_click(req.index)
                    case "clear_clicks":
                        c.clear_clicks()
                    case "save_mount":
                        out["file"] = c.save_mount()
                    case "compute_look":
                        c.compute_look()
                    case "goto_look":
                        c.goto_look(req.pose or "")
                    case "save_look":
                        out["lines"] = c.save_look()
                    case "calibrate":
                        out |= c.calibrate()
                    case "detect":
                        out["marks"] = c.detect()
                    case _:
                        raise HTTPException(400, f"unknown action {req.action!r}")
        except (Busy, WrongMode) as e:
            raise HTTPException(409, str(e)) from None
        except (ValueError, OSError, SorterError) as e:
            raise HTTPException(400, str(e)) from None
        return out

    def twin_source():
        twin = hub.twin()
        if twin is None:
            raise HTTPException(404, "no 3D view source")
        return twin

    @app.get("/api/twin/layout")
    def twin_layout() -> dict:
        return twin_source().layout()

    @app.get("/api/twin/state")
    def twin_state() -> dict:
        return twin_source().state()

    @app.get("/api/status")
    def status() -> dict:
        return status_json(hub.status(), speed_json(hub.speed()), hub.mode())

    def speed_source() -> SpeedControl:
        speed = hub.speed()
        if speed is None:
            raise HTTPException(404, "no speed control")
        return speed

    @app.get("/api/speed")
    def speed_state() -> dict:
        return speed_json(speed_source())

    @app.post("/api/speed")
    def set_speed(req: SpeedRequest) -> dict:
        speed = speed_source()
        try:
            speed.set_speed_scale(req.speed_scale)
        except ValueError as e:
            raise HTTPException(400, str(e)) from None
        return speed_json(speed)

    @app.post("/api/command")
    def command(req: CommandRequest) -> dict:
        try:
            cmd = Command(req.cmd)
        except ValueError:
            raise HTTPException(400, f"unknown command {req.cmd!r}") from None
        try:
            hub.send(cmd)  # HOLD calls arm.hold() right here, in this worker thread
        except WrongMode as e:
            raise HTTPException(409, str(e)) from None
        return {"ok": True}

    @app.websocket("/ws")
    async def ws(websocket: WebSocket) -> None:
        await websocket.accept()

        async def push() -> None:
            last = None
            while True:
                s = (hub.status(), speed_json(hub.speed()), hub.mode())
                if s != last:
                    await websocket.send_json(status_json(*s))
                    last = s
                await asyncio.sleep(1 / cfg.status_hz)

        task = asyncio.create_task(push())
        try:
            while (await websocket.receive())["type"] != "websocket.disconnect":
                pass
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await task

    def stream(next_jpeg: Callable[[], bytes]):
        def endpoint(request: Request) -> StreamingResponse:
            return StreamingResponse(
                mjpeg(request, next_jpeg, period_s),
                media_type=f"multipart/x-mixed-replace; boundary={BOUNDARY}",
                headers={"Cache-Control": "no-store"},
            )

        return endpoint

    def snapshot(next_jpeg: Callable[[], bytes]):
        def endpoint() -> Response:
            return Response(
                next_jpeg(), media_type="image/jpeg", headers={"Cache-Control": "no-store"}
            )

        return endpoint

    for name, fn in (("decision", frames.decision), ("live", frames.live)):
        app.add_api_route(f"/stream/{name}.mjpg", stream(fn), methods=["GET"])
        app.add_api_route(f"/snapshot/{name}.jpg", snapshot(fn), methods=["GET"])

    return app
