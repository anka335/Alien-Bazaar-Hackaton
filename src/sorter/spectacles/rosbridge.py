"""A small rosbridge v2 client: advertise, publish, subscribe. Only `websockets` is needed.

`sorter.nav.real_leo.Rosbridge` does the same but its module imports MuJoCo; this one keeps the
Spectacles bridge runnable on the rover with no more than `websockets`. A lost connection
raises from `publish`; `RoverLink` then builds a new client.
"""

from __future__ import annotations

import contextlib
import json
import logging
import threading

log = logging.getLogger("sorter.spectacles")


class Rosbridge:
    def __init__(self, url: str, timeout_s: float = 5.0):
        from websockets.sync.client import connect

        self.url = url
        self._conn = connect(url, open_timeout=timeout_s, max_size=None, ping_interval=None)
        self._ws = self._conn.__enter__()  # the connection is a context manager
        self._lock = threading.Lock()
        self._handlers: dict[str, object] = {}
        self.alive = True
        threading.Thread(target=self._read, name="rosbridge", daemon=True).start()

    def advertise(self, topic: str, msg_type: str) -> None:
        self._send({"op": "advertise", "topic": topic, "type": msg_type})

    def publish(self, topic: str, msg: dict) -> None:
        if not self.alive:
            raise ConnectionError(f"rosbridge {self.url}: connection lost")
        self._send({"op": "publish", "topic": topic, "msg": msg})

    def subscribe(self, topic: str, msg_type: str, handler, throttle_ms: int = 0) -> None:
        self._handlers[topic] = handler
        op = {"op": "subscribe", "topic": topic, "type": msg_type}
        self._send(op | {"throttle_rate": throttle_ms, "queue_length": 1})

    def close(self) -> None:
        self.alive = False
        with contextlib.suppress(Exception):
            self._ws.close()

    def _send(self, obj: dict) -> None:
        with self._lock:
            self._ws.send(json.dumps(obj))

    def _read(self) -> None:
        try:
            for raw in self._ws:
                with contextlib.suppress(ValueError, KeyError, TypeError):
                    msg = json.loads(raw)
                    if msg.get("op") == "publish" and msg["topic"] in self._handlers:
                        self._handlers[msg["topic"]](msg["msg"])
        except Exception as e:  # noqa: BLE001 - any failure is a lost connection
            log.warning("rosbridge %s: %s", self.url, e)
        finally:
            self.alive = False
