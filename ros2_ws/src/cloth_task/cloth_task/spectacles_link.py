"""The lens side of the robot bridge: a WebSocket server feeding one Spectacles Session.

Kept free of rclpy (only `websockets`) so it can be tested with a real client and no ROS. Each
socket goes through `Session.on_connect()`: a refused one is closed with 1013 (try again later),
an accepted one replaces the current socket, which is closed. Every text frame from the current
socket goes to `Session.on_teleop()` (undecodable JSON as malformed) and is answered with a
status; a status is also pushed at `status_hz` so a timeout reaches the lens. All Session calls
hold `lock`, shared with the ROS callbacks.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import threading
from collections.abc import Callable
from typing import Any

import websockets
from websockets.exceptions import ConnectionClosed

REFUSED_CODE = 1013
REPLACED_CODE = 1000
MAX_MESSAGE_BYTES = 1 << 16


class LensLink:
    def __init__(
        self,
        session: Any,
        lock: threading.Lock,
        host: str = "127.0.0.1",
        port: int = 9100,
        status_hz: float = 10.0,
        log: Callable[[str], None] = print,
    ):
        self.session, self.lock = session, lock
        self.host, self.port = host, port
        self.status_period = 1.0 / status_hz
        self.log = log
        self.ready = threading.Event()
        self._current: Any = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop: asyncio.Future | None = None

    async def serve(self) -> None:
        """Runs until stop(). `port` is the bound one once `ready` is set (port 0 = any)."""
        self._loop = asyncio.get_running_loop()
        self._stop = self._loop.create_future()
        async with websockets.serve(
            self._handle, self.host, self.port, max_size=MAX_MESSAGE_BYTES
        ) as server:
            self.port = next(iter(server.sockets)).getsockname()[1]
            self.ready.set()
            pusher = asyncio.ensure_future(self._push_status())
            try:
                await self._stop
            finally:
                pusher.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await pusher

    def run(self) -> None:
        """serve() on a fresh event loop in the calling thread."""
        asyncio.run(self.serve())

    def stop(self) -> None:
        """Thread-safe."""
        if self._loop is not None and self._stop is not None:
            self._loop.call_soon_threadsafe(
                lambda: self._stop.done() or self._stop.set_result(None)
            )

    async def _handle(self, ws: Any, *_: Any) -> None:
        with self.lock:
            accepted = self.session.on_connect()
        if not accepted:
            self.log(f"lens {ws.remote_address} refused: teleop mode is off or no joint state yet")
            await ws.close(REFUSED_CODE, "robot not ready for teleop")
            return
        old, self._current = self._current, ws
        self.log(f"lens {ws.remote_address} accepted" + (", replacing the last" if old else ""))
        if old is not None:
            await old.close(REPLACED_CODE, "replaced by a new lens")
        try:
            async for raw in ws:
                if ws is not self._current:
                    break
                try:
                    msg = json.loads(raw)
                except (TypeError, ValueError):
                    msg = None
                with self.lock:
                    self.session.on_teleop(msg)
                    status = self.session.status()
                await ws.send(json.dumps(status))
        except ConnectionClosed:
            pass
        finally:
            if self._current is ws:
                self._current = None
                self.log(f"lens {ws.remote_address} disconnected")

    async def _push_status(self) -> None:
        while True:
            await asyncio.sleep(self.status_period)
            ws = self._current
            if ws is None:
                continue
            with self.lock:
                status = self.session.status()
            with contextlib.suppress(ConnectionClosed):
                await ws.send(json.dumps(status))
