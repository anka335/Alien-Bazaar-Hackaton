import asyncio
import time

import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient

from sorter.app import build_system
from sorter.core.types import Command, Decision, Overlay, Phase, Status, Zone
from sorter.dashboard.server import BOUNDARY, create_app, mjpeg
from sorter.orchestrator.state_machine import StateMachine


@pytest.fixture
def system(sim_config):
    return build_system(sim_config, sim=True)


@pytest.fixture
def client(system):
    with TestClient(create_app(system.hub, system.cfg.dashboard, system.cfg.views)) as c:
        yield c


def _decode(jpeg: bytes) -> np.ndarray:
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert img is not None
    return img


def test_page_has_phase_labels(client):
    r = client.get("/")
    assert r.status_code == 200
    assert "__PHASE_LABELS__" not in r.text
    assert '"sense_box": "Choosing what to grab"' in r.text
    for path in ("/static/app.js", "/static/style.css"):
        assert client.get(path).status_code == 200


def test_status_json(client, system):
    system.hub.publish_status(Status(phase=Phase.LOOK_BG, mode="running", cycle=3))
    s = client.get("/api/status").json()
    assert s["phase"] == "look_bg" and s["mode"] == "running" and s["cycle"] == 3
    assert s["counters"] == {"light": 0, "dark": 0, "colored": 0}
    assert s["now"] == pytest.approx(time.monotonic(), abs=5)


def test_commands(client, system):
    held = []
    system.hub._on_hold = lambda: held.append(True)
    assert client.post("/api/command", json={"cmd": "start"}).status_code == 200
    assert client.post("/api/command", json={"cmd": "hold"}).status_code == 200
    assert held == [True]  # not queued
    assert system.hub.next_command(0) is Command.START
    assert system.hub.next_command(0) is None
    assert client.post("/api/command", json={"cmd": "fly"}).status_code == 400


def test_ws_pushes_status_on_change(client, system):
    with client.websocket_connect("/ws") as ws:
        assert ws.receive_json()["phase"] == "idle"
        system.hub.publish_status(Status(phase=Phase.HELD, mode="paused"))
        assert ws.receive_json()["phase"] == "held"


def test_decision_snapshot_matches_the_published_decision(client, system):
    before = client.get("/snapshot/decision.jpg")
    assert before.headers["content-type"] == "image/jpeg"
    _decode(before.content)  # placeholder

    system.arm.start()
    obs = system.observer.observe(Zone.BACKGROUND)
    system.hub.publish_decision(Decision(Phase.SENSE_BG, obs, Overlay(), "background empty"))
    img = _decode(client.get("/snapshot/decision.jpg").content)
    h, w = obs.frame.color.shape[:2]
    assert img.shape[1] == w and img.shape[0] > h  # the frame plus the caption strip


def test_live_snapshot(client):
    _decode(client.get("/snapshot/live.jpg").content)


def test_decision_frames_come_from_the_running_loop(client, system):
    sm = StateMachine(system)
    system.hub.send(Command.START)
    deadline = time.monotonic() + 5
    while system.hub.decision() is None and time.monotonic() < deadline:
        sm.poll()
    assert system.hub.decision() is not None
    _decode(client.get("/snapshot/decision.jpg").content)


def test_mjpeg_parts():
    class _Request:
        def __init__(self, parts):
            self.left = parts

        async def is_disconnected(self):
            self.left -= 1
            return self.left < 0

    async def collect():
        return [c async for c in mjpeg(_Request(2), lambda: b"JPEG", 0)]

    chunks = asyncio.run(collect())
    body = b"".join(chunks)
    assert body.startswith(f"--{BOUNDARY}\r\n".encode())
    assert body.count(b"Content-Type: image/jpeg") == 2
    assert body.endswith(b"JPEG\r\n--" + BOUNDARY.encode() + b"\r\n")
