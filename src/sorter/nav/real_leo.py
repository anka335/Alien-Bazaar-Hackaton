"""The real Leo Rover 1.9, driven over rosbridge (ROS 2 topics as JSON on a websocket).

LeoOS runs rosbridge for its web UI (port 9090): no ROS install is needed where this runs, only
the network. On the rover itself the URL is ws://127.0.0.1:9090, from a laptop on the rover's
Wi-Fi ws://10.0.0.1:9090 (the rover's default address).

`LeoBase` looks to the commands exactly like `RoverSim` does: `set_cmd`, `tick`, `odom`, `t`,
`dt`, `moving()`, `ref`, `cfg`. `tick()` sends the current twist on `/cmd_vel` and waits for the
next control period, so the firmware's 0.5 s timeout stops the rover if this process dies.
`odom` follows `/merged_odom` (wheel encoders + IMU, the rover's own fusion) and the IMU's yaw
rate from `/imu/data`.

Safety: speeds are clamped to `nav.real.max_linear_mps` / `max_angular_rps` (lower than the
rover's limits until the commands are trusted on the floor); `close()` sends zero twists.
"""

from __future__ import annotations

import contextlib
import json
import logging
import math
import threading
import time

from sorter.nav.config import NavConfig
from sorter.nav.sim import Odometry

log = logging.getLogger(__name__)


class RosbridgeError(RuntimeError):
    pass


class Rosbridge:
    """A minimal rosbridge v2 client: advertise / publish / subscribe, JSON over a websocket."""

    def __init__(self, url: str, timeout_s: float = 5.0):
        from websockets.sync.client import connect

        self.url = url
        try:
            self._conn = connect(url, open_timeout=timeout_s, max_size=None)
            self.ws = self._conn.__enter__()  # the connection is a context manager
        except (OSError, TimeoutError) as e:
            raise RosbridgeError(f"can't reach rosbridge at {url}: {e}") from None
        self._handlers: dict[str, list] = {}
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._reader = threading.Thread(target=self._read, name="rosbridge", daemon=True)
        self._reader.start()

    def advertise(self, topic: str, msg_type: str) -> None:
        self._send({"op": "advertise", "topic": topic, "type": msg_type})

    def publish(self, topic: str, msg: dict) -> None:
        self._send({"op": "publish", "topic": topic, "msg": msg})

    def subscribe(self, topic: str, msg_type: str, handler, throttle_ms: int = 0) -> None:
        self._handlers.setdefault(topic, []).append(handler)
        self._send(
            {
                "op": "subscribe",
                "topic": topic,
                "type": msg_type,
                "throttle_rate": throttle_ms,
                "queue_length": 1,
            }
        )

    def close(self) -> None:
        self._closed.set()
        with contextlib.suppress(Exception):  # closing anyway
            self._conn.__exit__(None, None, None)

    def _send(self, obj: dict) -> None:
        with self._lock:
            self.ws.send(json.dumps(obj))

    def _read(self) -> None:
        while not self._closed.is_set():
            try:
                raw = self.ws.recv()
            except Exception:  # noqa: BLE001 - connection gone
                if not self._closed.is_set():
                    log.error("rosbridge connection lost")
                return
            try:
                msg = json.loads(raw)
            except ValueError:
                continue
            if msg.get("op") == "publish":
                for h in self._handlers.get(msg.get("topic"), []):
                    try:
                        h(msg["msg"])
                    except Exception:  # noqa: BLE001 - one bad message doesn't stop the reader
                        log.exception("handler for %s failed", msg.get("topic"))


class LeoBase:
    """The real rover behind the same interface as `RoverSim`.

    Odometry: `odom_topic` (the rover's merged_odom) when it publishes; otherwise the wheel
    encoders (`wheel_states_topic`: speed = wheel radius x the mean wheel speed) and the IMU's
    yaw rate (`imu_topic`), integrated here. Whichever arrives first after connecting is used.
    """

    def __init__(self, cfg: NavConfig, bridge: Rosbridge | None = None):
        self.cfg = cfg
        r = cfg.real
        self.bridge = bridge or Rosbridge(r.rosbridge_url)
        self.dt = 1.0 / cfg.control_hz
        self.odom = Odometry()
        self.odom_source: str | None = None
        self.cmd = (0.0, 0.0)
        self.ref = [0.0, 0.0]
        self.fault: str | None = None
        self._t0 = time.monotonic()
        self._next = self._t0
        self._lock = threading.Lock()
        self._odom_seen = threading.Event()
        self._raw = None  # the first merged_odom pose: odometry starts at zero here
        self._wheel_v = 0.0
        self._imu_w: float | None = None
        self._last_int = None  # monotonic time of the last own integration step
        self._wrong_way_since: float | None = None
        self.collisions = 0  # no ground truth on the real rover
        self.bridge.advertise(r.cmd_vel_topic, "geometry_msgs/msg/Twist")
        self.bridge.subscribe(r.odom_topic, "nav_msgs/msg/Odometry", self._on_odom)
        self.bridge.subscribe(r.wheel_states_topic, "leo_msgs/msg/WheelStates", self._on_wheels)
        self.bridge.subscribe(r.imu_topic, "sensor_msgs/msg/Imu", self._on_imu)
        if not self._odom_seen.wait(r.connect_timeout_s):
            raise RosbridgeError(
                f"no odometry from {r.rosbridge_url} within {r.connect_timeout_s} s (neither "
                f"{r.odom_topic} nor {r.wheel_states_topic} + {r.imu_topic}): "
                "is the rover's ROS running?"
            )

    @property
    def t(self) -> float:
        return time.monotonic() - self._t0

    def set_cmd(self, v: float, w: float) -> None:
        r = self.cfg.real
        self.cmd = (
            max(-r.max_linear_mps, min(r.max_linear_mps, float(v))),
            max(-r.max_angular_rps, min(r.max_angular_rps, float(w))),
        )

    def tick(self) -> None:
        v, w = self.cmd
        self._check_direction(v, w)
        if self.fault:
            v = w = 0.0
            self.cmd = (0.0, 0.0)
        self.ref = [v, w]
        self.bridge.publish(self.cfg.real.cmd_vel_topic, _twist(v, w))
        self._next += self.dt
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:  # fell behind (slow frame): don't try to catch up
            self._next = time.monotonic()
        if self.fault:
            raise RosbridgeError(self.fault)

    def settle(self, seconds: float) -> None:
        for _ in range(int(seconds / self.dt)):
            self.tick()

    def moving(self) -> bool:
        return abs(self.odom.v) > 0.005 or abs(self.odom.w) > 0.01

    def clear_fault(self) -> None:
        self.fault = None
        self._wrong_way_since = None

    def close(self) -> None:
        for _ in range(5):
            self.bridge.publish(self.cfg.real.cmd_vel_topic, _twist(0.0, 0.0))
            time.sleep(0.02)
        self.bridge.close()

    def _check_direction(self, v: float, w: float) -> None:
        """Commanded one way, the odometry going clearly the other for 0.4 s: stop (a sign
        error in the odometry would otherwise make every command run away)."""
        o = self.odom
        wrong = (abs(v) > 0.05 and o.v * v < 0 and abs(o.v) > 0.03) or (
            abs(w) > 0.2 and o.w * w < 0 and abs(o.w) > 0.1
        )
        now = time.monotonic()
        if not wrong:
            self._wrong_way_since = None
        elif self._wrong_way_since is None:
            self._wrong_way_since = now
        elif now - self._wrong_way_since > 0.4:
            self.fault = (
                f"stopped: commanded v {v:+.2f} w {w:+.2f} but the odometry says v {o.v:+.2f} "
                f"w {o.w:+.2f} ({self.odom_source}): check the odometry's signs"
            )

    # --- odometry sources ---

    def _on_odom(self, msg: dict) -> None:
        with self._lock:
            if self.odom_source not in (None, "merged_odom"):
                return
            self.odom_source = "merged_odom"
            p = msg["pose"]["pose"]
            q = p["orientation"]
            yaw = math.atan2(
                2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2)
            )
            x, y = p["position"]["x"], p["position"]["y"]
            if self._raw is None:
                self._raw = (x, y, yaw)
            x0, y0, yaw0 = self._raw
            c, s = math.cos(-yaw0), math.sin(-yaw0)
            dx, dy = x - x0, y - y0
            tw = msg["twist"]["twist"]
            o = self.odom
            nx, ny = c * dx - s * dy, s * dx + c * dy
            o.distance += math.hypot(nx - o.x, ny - o.y)
            # unwrap the yaw so turns of more than 180° add up like the sim's odometry
            new_yaw = yaw - yaw0
            o.yaw += (new_yaw - o.yaw + math.pi) % (2 * math.pi) - math.pi
            o.x, o.y = nx, ny
            o.v, o.w = tw["linear"]["x"], tw["angular"]["z"]
        self._odom_seen.set()

    def _on_wheels(self, msg: dict) -> None:
        vel = msg.get("velocity") or []
        if len(vel) < 4:
            return
        # FL, RL, FR, RR in rad/s, positive = forward (the URDF's joint axes)
        self._wheel_v = self.cfg.leo.wheel_radius_m * sum(vel[:4]) / 4
        self._integrate()

    def _on_imu(self, msg: dict) -> None:
        self._imu_w = float(msg["angular_velocity"]["z"])
        self._integrate()

    def _integrate(self) -> None:
        with self._lock:
            if self.odom_source == "merged_odom" or self._imu_w is None:
                return
            self.odom_source = "wheels+imu"
            now = time.monotonic()
            dt = 0.0 if self._last_int is None else min(now - self._last_int, 0.2)
            self._last_int = now
            o = self.odom
            o.v, o.w = self._wheel_v, self._imu_w
            o.yaw += o.w * dt
            o.x += o.v * math.cos(o.yaw) * dt
            o.y += o.v * math.sin(o.yaw) * dt
            o.distance += abs(o.v) * dt
        self._odom_seen.set()


def _twist(v: float, w: float) -> dict:
    return {"linear": {"x": v, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": w}}
