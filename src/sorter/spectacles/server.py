"""The Spectacles bridge for the real Leo Rover's base: the lens's WebSocket in, rosbridge out.

    lens --wss (ngrok)--> ws://0.0.0.0:9100 (this) --rosbridge--> cmd_vel on the rover
                                                   <-rosbridge--  merged_odom / wheel_states

One lens socket at a time: a new one replaces the old (and needs a release, protocol.md).
`status` goes back at 10 Hz. A publisher thread sends `cmd_vel` at 20 Hz while an accepted left
clutch drives, then a few zero twists; while idle it sends nothing, so another controller of the
rover (the nav stack) is not overridden. Rover odometry only marks the rover present.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from sorter.spectacles.session import BaseSession

log = logging.getLogger("sorter.spectacles")

STATUS_PERIOD_S = 0.1
CMD_PERIOD_S = 0.05
STOP_ZEROS = 5  # zero twists sent when driving ends (a lost one is covered by the next)


def twist(vx: float, wz: float) -> dict:
    return {"linear": {"x": vx, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": wz}}


@dataclass
class RoverTopics:
    cmd_vel: str = "cmd_vel"  # relative: rosbridge resolves them in the rover's namespace
    odom: str = "merged_odom"
    wheel_states: str = "firmware/wheel_states"


class RoverLink:
    """rosbridge to the rover, (re)connected in the background; publishes the session's twist."""

    def __init__(
        self,
        session: BaseSession,
        url: str,
        topics: RoverTopics | None = None,
        bridge_factory: Callable | None = None,
    ):
        self.session, self.url, self.topics = session, url, topics or RoverTopics()
        self._factory = bridge_factory or _rosbridge
        self._stop = threading.Event()
        self.bridge = None
        self.sent = 0
        self._thread = threading.Thread(target=self._run, name="rover-link", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        if self.bridge is not None:
            with contextlib.suppress(Exception):
                for _ in range(STOP_ZEROS):
                    self.bridge.publish(self.topics.cmd_vel, twist(0.0, 0.0))
                    time.sleep(0.02)
            with contextlib.suppress(Exception):
                self.bridge.close()

    def _connect(self) -> None:
        while not self._stop.is_set():
            try:
                bridge = self._factory(self.url)
                bridge.advertise(self.topics.cmd_vel, "geometry_msgs/msg/Twist")
                bridge.subscribe(self.topics.odom, "nav_msgs/msg/Odometry", self._on_odom, 50)
                bridge.subscribe(
                    self.topics.wheel_states, "leo_msgs/msg/WheelStates", self._on_odom, 50
                )
                self.bridge = bridge
                log.info("rosbridge %s connected, waiting for rover odometry", self.url)
                return
            except Exception as e:  # noqa: BLE001 - keep trying: the rover may still be booting
                log.warning("rosbridge %s: %s; retrying in 2 s", self.url, e)
                self._stop.wait(2.0)

    def _on_odom(self, _msg) -> None:
        self.session.on_odom()

    def _run(self) -> None:
        self._connect()
        zeros = 0
        was_driving = False
        present = False
        while not self._stop.is_set():
            if not getattr(self.bridge, "alive", True):  # lost: the rover goes absent meanwhile
                log.warning("rosbridge %s lost, reconnecting", self.url)
                with contextlib.suppress(Exception):
                    self.bridge.close()
                self._connect()
            driving, vx, wz = self.session.command()
            now_present = self.session.rover_present()
            if now_present != present:
                present = now_present
                log.info("rover %s", "present (odometry)" if present else "ABSENT: no odometry")
            if driving != was_driving:
                log.info("base %s", "driving" if driving else "idle")
                was_driving = driving
            if driving:
                zeros = STOP_ZEROS
                self._publish(vx, wz)
            elif zeros:
                zeros -= 1
                self._publish(0.0, 0.0)
            self._stop.wait(CMD_PERIOD_S)

    def _publish(self, vx: float, wz: float) -> None:
        try:
            self.bridge.publish(self.topics.cmd_vel, twist(vx, wz))
            self.sent += 1
        except Exception as e:  # noqa: BLE001 - the firmware stops the wheels after 0.5 s
            log.warning("cmd_vel not sent: %s", e)


def _rosbridge(url: str):
    from sorter.spectacles.rosbridge import Rosbridge

    return Rosbridge(url)


class LensServer:
    """The lens's WebSocket server: one accepted socket, `status` at 10 Hz."""

    def __init__(self, session: BaseSession, host: str = "0.0.0.0", port: int = 9100):
        self.session, self.host, self.port = session, host, port
        self._current = None
        self._last_log = 0.0

    async def serve(self, ready: asyncio.Event | None = None, stop: asyncio.Event | None = None):
        server = await _serve(self._handle, self.host, self.port)
        if self.port == 0:
            self.port = next(iter(server.sockets)).getsockname()[1]
        log.info("lens WebSocket on ws://%s:%d", self.host, self.port)
        if ready is not None:
            ready.set()
        try:
            await (stop.wait() if stop is not None else asyncio.Future())
        finally:
            server.close()
            await server.wait_closed()

    async def _handle(self, ws, *_):
        old, self._current = self._current, ws
        if old is not None:
            log.info("a new lens socket replaces the old one")
            await old.close()
        self.session.on_connect()
        log.info("lens connected from %s", _peer(ws))
        sender = asyncio.ensure_future(self._send_status(ws))
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except ValueError:
                    continue
                if not self.session.on_message(msg) and _is_teleop(msg):
                    self._log_malformed(msg)
        except Exception as e:  # noqa: BLE001 - a dropped socket is a close
            log.info("lens socket error: %s", e)
        finally:
            sender.cancel()
            if self._current is ws:
                self._current = None
                self.session.on_disconnect()
                log.info("lens disconnected: base stopped")

    async def _send_status(self, ws):
        with contextlib.suppress(Exception):
            while True:
                await ws.send(json.dumps(self.session.status()))
                await asyncio.sleep(STATUS_PERIOD_S)

    def _log_malformed(self, msg) -> None:
        now = time.monotonic()
        if now - self._last_log > 5:
            self._last_log = now
            log.warning("malformed teleop ignored: %s", json.dumps(msg)[:200])


def _is_teleop(msg) -> bool:
    return isinstance(msg, dict) and msg.get("type") == "teleop"


def _peer(ws) -> str:
    with contextlib.suppress(Exception):
        return str(ws.remote_address)
    return "?"


async def _serve(handler, host: str, port: int):
    try:  # websockets >= 13
        from websockets.asyncio.server import serve
    except ImportError:  # websockets 12
        from websockets.server import serve
    return await serve(handler, host, port, ping_interval=None)
