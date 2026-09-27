"""Hub: status, decision frames, and commands between the state machine and the dashboard.

It also holds the operator mode (`OperatorMode`): run commands reach the state machine only in
a run mode (LOAD / UNLOAD, the loop it runs), and a run mode is left only while no run is going
(the state machine idle, no command pending).
"""

from __future__ import annotations

import logging
import queue
import threading
from collections import deque
from collections.abc import Callable
from dataclasses import replace
from typing import Any, Protocol

from sorter.core.errors import WrongMode
from sorter.core.protocols import Camera
from sorter.core.types import RUN_MODES, Command, Decision, Event, Frame, OperatorMode, Status

log = logging.getLogger(__name__)


class TwinSource(Protocol):
    """Data for the dashboard's 3D view: the scene's static parts and the live arm (+ sim
    items)."""

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
        mode: OperatorMode = OperatorMode.LOAD,
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
        self._mode = mode
        self._in_flight = False  # a command taken by the state machine, its status not out yet

    # --- state machine side ---

    def publish_status(self, s: Status) -> None:
        with self._lock:
            self._status = s
            self._in_flight = False

    def publish_decision(self, d: Decision) -> None:
        with self._lock:
            self._decision = d

    def next_command(self, timeout_s: float) -> Command | None:
        """The next queued command, or None after `timeout_s` (0 = don't wait)."""
        try:
            if timeout_s <= 0:
                cmd = self._commands.get_nowait()
            else:
                cmd = self._commands.get(timeout=timeout_s)
        except queue.Empty:
            return None
        with self._lock:
            if self._mode not in RUN_MODES:  # the mode changed while it was queued
                log.warning("command %s dropped: mode is %s", cmd, self._mode)
                return None
            self._in_flight = True
        return cmd

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
        """HOLD calls on_hold() immediately, in the caller's thread, in any mode. Others are
        queued, in a run mode only (WrongMode otherwise)."""
        cmd = Command(cmd)
        if cmd is Command.HOLD:
            log.warning("HOLD requested")
            self._on_hold()
            return
        with self._lock:
            if self._mode not in RUN_MODES:
                raise WrongMode(f"{cmd} works in the load / unload mode; the mode is {self._mode}")
            self._commands.put(cmd)

    # --- operator mode ---

    def mode(self) -> OperatorMode:
        with self._lock:
            return self._mode

    def set_mode(self, mode: OperatorMode | str) -> None:
        """Change the operator mode. Leaving a run mode needs the state machine idle (WrongMode
        otherwise): Stop the run first."""
        mode = OperatorMode(mode)
        with self._lock:
            if mode is self._mode:
                return
            leaving_run = self._mode in RUN_MODES
            if leaving_run and (
                self._status.mode != "idle" or self._in_flight or not self._commands.empty()
            ):
                raise WrongMode("a run is going: press Stop first")
            log.info("operator mode: %s → %s", self._mode, mode)
            self._mode = mode

    # --- logging ---

    def add_event(self, e: Event) -> None:
        with self._lock:
            self._events.append(e)
