"""Hub: status, decision frames, and commands between the state machine and the dashboard."""

from __future__ import annotations

import logging
import queue
import threading
from collections import deque
from collections.abc import Callable
from dataclasses import replace
from typing import Any, Protocol

from sorter.core.protocols import Camera
from sorter.core.types import Command, Decision, Event, Frame, Status

log = logging.getLogger(__name__)


class TwinSource(Protocol):
    """Data for the dashboard's 3D view: the table layout and the live arm (+ sim items)."""

    def layout(self) -> dict[str, Any]: ...
    def state(self) -> dict[str, Any]: ...


class SpeedControl(Protocol):
    """The arm's speed, set from the dashboard; the next motion uses it (block 5's Controller)."""

    @property
    def speed_scale(self) -> float: ...
    @property
    def max_speed_scale(self) -> float: ...
    def set_speed_scale(self, scale: float) -> float: ...


class Hub:
    def __init__(
        self,
        camera: Camera,
        on_hold: Callable[[], None],
        max_events: int = 50,
        twin: TwinSource | None = None,
        speed: SpeedControl | None = None,
    ):
        self._camera = camera
        self._on_hold = on_hold
        self._twin = twin
        self._speed = speed
        self._lock = threading.Lock()
        self._status = Status()
        self._decision: Decision | None = None
        self._events: deque[Event] = deque(maxlen=max_events)
        self._commands: queue.Queue[Command] = queue.Queue()

    # --- state machine side ---

    def publish_status(self, s: Status) -> None:
        with self._lock:
            self._status = s

    def publish_decision(self, d: Decision) -> None:
        with self._lock:
            self._decision = d

    def next_command(self, timeout_s: float) -> Command | None:
        """The next queued command, or None after `timeout_s` (0 = don't wait)."""
        try:
            if timeout_s <= 0:
                return self._commands.get_nowait()
            return self._commands.get(timeout=timeout_s)
        except queue.Empty:
            return None

    # --- dashboard side ---

    def status(self) -> Status:
        with self._lock:
            return replace(self._status, events=list(self._events))

    def decision(self) -> Decision | None:
        with self._lock:
            return self._decision

    def live_frame(self) -> Frame | None:
        return self._camera.latest()

    def twin(self) -> TwinSource | None:
        return self._twin

    def speed(self) -> SpeedControl | None:
        return self._speed

    def send(self, cmd: Command | str) -> None:
        """HOLD calls on_hold() immediately, in the caller's thread. Others are queued."""
        cmd = Command(cmd)
        if cmd is Command.HOLD:
            log.warning("HOLD requested")
            self._on_hold()
        else:
            self._commands.put(cmd)

    # --- logging ---

    def add_event(self, e: Event) -> None:
        with self._lock:
            self._events.append(e)
