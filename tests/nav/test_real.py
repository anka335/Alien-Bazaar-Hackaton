"""The real-rover path without hardware: a fake rosbridge (a unicycle integrating /cmd_vel and
publishing /merged_odom) and a fake camera, driven by the real LeoBase and the real commands."""

import json
import math
import threading
import time

import numpy as np
import pytest
from websockets.sync.server import serve

from sorter.nav.camera import Frame, Intrinsics, _mount
from sorter.nav.config import NavConfig
from sorter.nav.real import RealSession, last_frame, record
from sorter.nav.real_leo import LeoBase, Rosbridge, RosbridgeError


class FakeLeo:
    """A rosbridge server with a kinematic rover behind it."""

    def __init__(self, turn_gain: float = 1.0):
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.turn_gain = turn_gain
        self.last_cmd_t = 0.0
        self.cmds: list[tuple[float, float]] = []
        self.server = serve(self._handler, "127.0.0.1", 0)
        self.port = self.server.socket.getsockname()[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    @property
    def url(self) -> str:
        return f"ws://127.0.0.1:{self.port}"

    def _handler(self, ws):
        stop = threading.Event()

        def publish_odom():
            t = time.monotonic()
            while not stop.is_set():
                now = time.monotonic()
                dt, t = now - t, now
                if now - self.last_cmd_t > 0.5:  # the firmware's timeout
                    self.v = self.w = 0.0
                self.yaw += self.w * self.turn_gain * dt
                self.x += self.v * math.cos(self.yaw) * dt
                self.y += self.v * math.sin(self.yaw) * dt
                msg = {
                    "op": "publish",
                    "topic": "/merged_odom",
                    "msg": {
                        "pose": {
                            "pose": {
                                "position": {"x": self.x, "y": self.y, "z": 0.0},
                                "orientation": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": math.sin(self.yaw / 2),
                                    "w": math.cos(self.yaw / 2),
                                },
                            }
                        },
                        "twist": {
                            "twist": {
                                "linear": {"x": self.v, "y": 0.0, "z": 0.0},
                                "angular": {"x": 0.0, "y": 0.0, "z": self.w * self.turn_gain},
                            }
                        },
                    },
                }
                try:
                    ws.send(json.dumps(msg))
                except Exception:  # noqa: BLE001 - client gone
                    return
                time.sleep(0.02)

        for raw in ws:
            m = json.loads(raw)
            if m["op"] == "subscribe" and m["topic"] == "/merged_odom":
                threading.Thread(target=publish_odom, daemon=True).start()
            elif m["op"] == "publish" and m["topic"] == "/cmd_vel":
                self.v, self.w = m["msg"]["linear"]["x"], m["msg"]["angular"]["z"]
                self.last_cmd_t = time.monotonic()
                self.cmds.append((self.v, self.w))
        stop.set()

    def close(self):
        self.server.shutdown()


class FakeCamera:
    """Empty floor far away: no obstacles for the guard, no socks."""

    def __init__(self, cfg: NavConfig):
        c = cfg.camera
        self.K = Intrinsics(465.6, 465.6, c.width / 2, c.height / 2, c.width, c.height)
        self.T_rover_cam = _mount(c)
        self.n = 0
        self.closed = False

    def capture(self) -> Frame:
        self.n += 1
        h, w = self.K.height, self.K.width
        return Frame(
            self.n,
            0.0,
            np.full((h, w, 3), 128, np.uint8),
            np.zeros((h, w), np.uint16),
            self.K,
            self.T_rover_cam,
        )

    def close(self):
        self.closed = True


@pytest.fixture
def fake():
    f = FakeLeo()
    yield f
    f.close()


def _cfg(url: str) -> NavConfig:
    cfg = NavConfig()
    cfg.real.rosbridge_url = url
    return cfg


def test_rosbridge_unreachable():
    with pytest.raises(RosbridgeError):
        Rosbridge("ws://127.0.0.1:1", timeout_s=1.0)


def test_commands_drive_the_real_base(fake):
    cfg = _cfg(fake.url)
    s = RealSession(cfg, camera=FakeCamera(cfg))
    try:
        res = s.rover.forward(0.3, 0.2)
        assert res.blocked is None
        assert fake.x == pytest.approx(0.3, abs=0.03)
        res = s.rover.turn(45)
        assert math.degrees(fake.yaw) == pytest.approx(45, abs=3)
        assert res.turned_deg == pytest.approx(45, abs=3)
    finally:
        s.close()
    # closing stops the rover, and speeds never exceeded the real caps
    assert fake.cmds[-1] == (0.0, 0.0)
    assert max(abs(v) for v, _ in fake.cmds) <= cfg.real.max_linear_mps + 1e-9
    assert max(abs(w) for _, w in fake.cmds) <= cfg.real.max_angular_rps + 1e-9


def test_turn_closes_the_loop_on_odometry():
    fake = FakeLeo(turn_gain=0.55)  # the rover turns slower than commanded: still lands on 60°
    cfg = _cfg(fake.url)
    s = RealSession(cfg, camera=FakeCamera(cfg))
    try:
        s.rover.turn(60)
        assert math.degrees(fake.yaw) == pytest.approx(60, abs=3)
    finally:
        s.close()
        fake.close()


def test_no_odometry_is_an_error():
    server = serve(lambda ws: [None for _ in ws], "127.0.0.1", 0)  # accepts, never publishes
    threading.Thread(target=server.serve_forever, daemon=True).start()
    cfg = _cfg(f"ws://127.0.0.1:{server.socket.getsockname()[1]}")
    cfg.real.connect_timeout_s = 0.5
    with pytest.raises(RosbridgeError, match="merged_odom"):
        LeoBase(cfg)
    server.shutdown()


def test_frames_are_recorded_for_the_next_process(fake, tmp_path):
    cfg = _cfg(fake.url)
    cam = FakeCamera(cfg)
    s = RealSession(cfg, camera=cam)
    try:
        out = record(tmp_path, s.rover.look(), cfg.goal)
    finally:
        s.close()
    assert (tmp_path / "log.jsonl").is_file() and out["frame"].endswith("_grid.png")
    f = last_frame(tmp_path, cam)
    assert f is not None and f.rgb.shape == (480, 640, 3)


def test_rover_tab_backend_drives_the_real_rover(fake, monkeypatch):
    """The live panel in --real mode: reset connects, a command drives the (fake) rover."""
    import sorter.nav.real as real_mod
    from sorter.nav.server import CommandRequest, NavLive

    cfg = _cfg(fake.url)
    orig = real_mod.RealSession
    monkeypatch.setattr(real_mod, "RealSession", lambda c: orig(c, camera=FakeCamera(c)))
    live = NavLive(cfg, real=True)
    try:
        for _ in range(100):
            if live.episode is not None and live.busy is None:
                break
            time.sleep(0.05)
        st = live.state()
        assert st["real"] and st["error"] is None and "truth" not in st
        live.submit("command", CommandRequest(name="forward", args={"distance_m": 0.2}))
        for _ in range(200):
            time.sleep(0.05)
            if live.busy is None and live.log:
                break
        assert live.state()["log"][-1]["command"] == "forward"
        assert fake.x == pytest.approx(0.2, abs=0.03)
    finally:
        live.stop()
        if live.episode is not None:
            live.episode.close()
