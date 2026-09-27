"""Runs a LeoSim in its own thread, in real time, for the web UI and the jevomir agent.

MuJoCo's offscreen GL context belongs to the thread that made it, so every sim call goes
through `call()`, which runs a function on the sim thread and returns its result.
"""

from __future__ import annotations

import io
import math
import queue
import threading
import time
from collections.abc import Callable
from typing import Any

import numpy as np
from PIL import Image

from .rover import LeoSim

STEP = 0.02  # sim seconds per loop iteration
FRAME_HZ = 10  # UI frames per second, for cameras asked for in the last 2 s
FRAME_SIZE = {"leo": (480, 360), "oak": (480, 270), "chase": (480, 360), "map": (480, None)}


def jpeg(image: np.ndarray, quality: int = 85) -> bytes:
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="JPEG", quality=quality)
    return buffer.getvalue()


class SimRunner:
    def __init__(self, seed: int | None = None, realtime: float = 1.0):
        self.seed = seed
        self.realtime = realtime  # sim seconds per wall second; 0 = as fast as possible
        self.goal: tuple[float, float] | None = None  # map marker (the agent's target)
        self._calls: queue.Queue = queue.Queue()
        self._frames: dict[str, bytes] = {}
        self._wanted: dict[str, float] = {}
        self._state: dict[str, Any] = {}
        self._lock = threading.Lock()
        self._tick = threading.Condition()
        self._stop = threading.Event()
        self._idle = threading.Event()  # wakes an idle sim thread when a call arrives
        self._ready = threading.Event()
        self._error: BaseException | None = None
        self._thread = threading.Thread(target=self._loop, name="leo-sim", daemon=True)
        self._thread.start()
        self._ready.wait()
        if self._error:
            raise self._error

    # ---- sim thread --------------------------------------------------------------------

    def _loop(self) -> None:
        try:
            self.sim = LeoSim(seed=self.seed)
        except BaseException as error:  # noqa: BLE001 - reported to the constructor
            self._error = error
            self._ready.set()
            return
        self._ready.set()
        next_frame = 0.0
        wall0, sim0 = time.monotonic(), self.sim.time
        while not self._stop.is_set():
            while not self._calls.empty():
                fn, box, done = self._calls.get()
                try:
                    box["result"] = fn(self.sim)
                except BaseException as error:  # noqa: BLE001 - handed to the caller
                    box["error"] = error
                done.set()
                if box.get("reset_clock"):
                    wall0, sim0 = time.monotonic(), self.sim.time
            parked = self.realtime == 0 and self.sim.at_rest()
            if not parked:
                self.sim.step(STEP)
            self._publish_state()
            if time.monotonic() >= next_frame:
                self._render_frames()
                next_frame = time.monotonic() + 1 / FRAME_HZ
            if parked:
                # as fast as possible, but no sim time passes while the agent waits for
                # the API: the rover is parked, so stepping would only burn CPU
                self._idle.wait(0.02)
                continue
            with self._tick:
                self._tick.notify_all()
            if self.realtime > 0:
                ahead = (self.sim.time - sim0) / self.realtime - (time.monotonic() - wall0)
                if ahead > 0:
                    time.sleep(ahead)
                elif ahead < -0.5:  # fell behind (slow machine): don't try to catch up
                    wall0, sim0 = time.monotonic(), self.sim.time
        self.sim.close()

    def _publish_state(self) -> None:
        s = self.sim
        x, y, yaw = s.pose()
        ox, oy, oh = s.odom()
        v, w = s.velocity()
        motion = s.motion
        state = {
            "time": round(s.time, 2),
            "pose": {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(math.degrees(yaw), 1)},
            "odom": {
                "x": round(ox, 3),
                "y": round(oy, 3),
                "heading_deg": round(math.degrees(oh), 1),
            },
            "velocity": {"linear": round(v, 3), "angular_deg": round(math.degrees(w), 1)},
            "tilt_deg": round(s.tilt(), 1),
            "motion": None
            if motion is None
            else {"kind": motion.kind, "outcome": motion.outcome, "bumped": sorted(motion.bumped)},
        }
        with self._lock:
            self._state = state

    def _render_frames(self) -> None:
        now = time.monotonic()
        with self._lock:
            wanted = [c for c, t in self._wanted.items() if now - t < 2.0]
        for camera in wanted:
            width, height = FRAME_SIZE.get(camera, (480, 360))
            if camera == "map":
                image = self.sim.render_map(width, goal=self.goal)
            else:
                image = self.sim.render(camera, width, height)
            data = jpeg(image)
            with self._lock:
                self._frames[camera] = data

    # ---- any thread --------------------------------------------------------------------

    def call(
        self, fn: Callable[[LeoSim], Any], timeout: float = 30.0, reset_clock: bool = False
    ) -> Any:
        box: dict[str, Any] = {"reset_clock": reset_clock}
        done = threading.Event()
        self._calls.put((fn, box, done))
        self._idle.set()
        self._idle.clear()
        if not done.wait(timeout):
            raise TimeoutError("sim thread did not answer")
        if "error" in box:
            raise box["error"]
        return box.get("result")

    def state(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._state)

    def frame(self, camera: str) -> bytes | None:
        """Latest JPEG of `camera`; asking keeps it rendered at FRAME_HZ for 2 s."""
        with self._lock:
            first = camera not in self._wanted
            self._wanted[camera] = time.monotonic()
            data = self._frames.get(camera)
        if data is None and first:
            with self._tick:
                self._tick.wait(0.5)
            with self._lock:
                data = self._frames.get(camera)
        return data

    def render(self, camera: str, width: int, height: int | None = None) -> np.ndarray:
        """A fresh render on the sim thread (for the agent)."""
        return self.call(lambda s: s.render(camera, width, height))

    def motion(
        self,
        kind: str,
        amount: float,
        speed: float | None = None,
        cancel: threading.Event | None = None,
    ) -> dict[str, Any]:
        """Run a move (m) or turn (rad) to its end in sim time; returns outcome and bumps."""

        def start(s: LeoSim):
            if kind == "move":
                return s.start_move(amount, speed or 0.2)
            return s.start_turn(amount, speed or math.radians(45))

        m = self.call(start)
        while m.outcome == "running":
            if self._stop.is_set() or (cancel is not None and cancel.is_set()):
                self.call(lambda s: s.stop())
                break
            with self._tick:
                self._tick.wait(0.2)
        end = self.call(lambda s: s.time) + 0.3  # come to rest before the next look
        while self.state().get("time", end) < end and not self._stop.is_set():
            with self._tick:
                self._tick.wait(0.2)
        return {"outcome": m.outcome, "bumped": sorted(m.bumped)}

    def reset(self, seed: int | None) -> None:
        def rebuild(s: LeoSim):
            from .model import default_world

            s.close()
            s.__init__(default_world(seed))

        self.seed = seed
        self.goal = None
        self.call(rebuild, reset_clock=True)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=5)
