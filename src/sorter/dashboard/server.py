"""Dashboard: FastAPI server with a single static page. It depends only on the Hub.

The HTTP API is in docs/architecture.md → Dashboard HTTP API.
"""

from __future__ import annotations

import asyncio
import contextlib
import dataclasses
import json
import math
import threading
import time
from collections.abc import AsyncIterator, Callable, Mapping
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Request, WebSocket
from fastapi.encoders import jsonable_encoder
from fastapi.responses import HTMLResponse, RedirectResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from rebot_b601.assets import ASSETS_DIR as TWIN_ASSETS_DIR

from sorter.camera.config import ViewConfig
from sorter.core.hub import Hub
from sorter.core.types import Command, Decision, Status, Zone
from sorter.dashboard.config import DashboardConfig
from sorter.dashboard.manual import Busy, ManualControl
from sorter.dashboard.render import PHASE_LABELS, encode_jpeg, placeholder, render_decision

STATIC_DIR = Path(__file__).parent / "static"
BOUNDARY = "frame"


class CommandRequest(BaseModel):
    cmd: str


class ManualRequest(BaseModel):
    action: str  # go | tour_next | tour_reset | jog | gripper | release | save
    pose: str | None = None
    joint: int | None = None  # 0..5
    delta_deg: float = 0.0
    open: bool = True


def status_json(s: Status) -> dict[str, Any]:
    """`Status` as JSON, plus `now` (monotonic, same clock as `Event.t`) for event ages."""
    return {**jsonable_encoder(dataclasses.asdict(s)), "now": time.monotonic()}


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
) -> FastAPI:
    """`views` gives the zone ROIs drawn on the decision frame (`cfg.views`). `manual`: the
    setup mode (`python -m sorter manual`), where `/` opens the manual control page."""
    app = FastAPI(title="Sorter")
    rois = {z: list(v.roi) for z, v in (views or {}).items()}
    frames = Frames(hub, cfg, rois)
    period_s = 1 / cfg.stream_fps
    labels = json.dumps({p.value: label for p, label in PHASE_LABELS.items()})
    page = (STATIC_DIR / "index.html").read_text().replace("__PHASE_LABELS__", labels)

    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
    # the arm's CAD meshes and three.js, vendored in rebot_b601 (D-012: no CDN)
    app.mount("/twin-assets", StaticFiles(directory=TWIN_ASSETS_DIR), name="twin-assets")

    @app.get("/", response_class=HTMLResponse)
    def index():
        return RedirectResponse("/manual") if manual else page

    @app.get("/manual", response_class=HTMLResponse)
    def manual_page() -> str:
        return (STATIC_DIR / "manual.html").read_text()

    def manual_source() -> ManualControl:
        if manual is None:
            raise HTTPException(404, "manual control is off; start with `python -m sorter manual`")
        return manual

    @app.get("/api/manual")
    def manual_state() -> dict:
        return manual_source().state()

    @app.post("/api/manual")
    def manual_action(req: ManualRequest) -> dict:
        m = manual_source()
        try:
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
                case "save":
                    return {"ok": True, "line": m.save_pose(req.pose or "")}
                case _:
                    raise HTTPException(400, f"unknown action {req.action!r}")
        except Busy as e:
            raise HTTPException(409, str(e)) from None
        except (ValueError, OSError) as e:
            raise HTTPException(400, str(e)) from None
        return {"ok": True}

    @app.get("/twin", response_class=HTMLResponse)
    def twin_page() -> str:
        return (STATIC_DIR / "twin.html").read_text()

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
        return status_json(hub.status())

    @app.post("/api/command")
    def command(req: CommandRequest) -> dict:
        try:
            cmd = Command(req.cmd)
        except ValueError:
            raise HTTPException(400, f"unknown command {req.cmd!r}") from None
        hub.send(cmd)  # HOLD calls arm.hold() right here, in this worker thread
        return {"ok": True}

    @app.websocket("/ws")
    async def ws(websocket: WebSocket) -> None:
        await websocket.accept()

        async def push() -> None:
            last = None
            while True:
                s = hub.status()
                if s != last:
                    await websocket.send_json(status_json(s))
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
