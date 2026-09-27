"""The dashboard's operator mode switch: LOAD / UNLOAD (the state machine's loops), MANUAL,
CALIBRATE (D-031, D-032).

One process serves all of them. The mode itself lives in the Hub (it gates the run commands); this
adds what the setup side needs: a change waits for no manual motion to run, the arm's motors go
on when a setup mode starts (the state machine does it itself at Start), and `on_change` lets the
simulator show the calibration's tape marks only in CALIBRATE. Manual and calibration actions run
under `guard`, so a mode change can't slip in between the mode check and the motion's start.
"""

from __future__ import annotations

import contextlib
import logging
import threading
from collections.abc import Callable, Iterator
from typing import TYPE_CHECKING

from sorter.core.errors import WrongMode
from sorter.core.types import OperatorMode

if TYPE_CHECKING:
    from sorter.core.hub import Hub
    from sorter.dashboard.manual import ManualControl

log = logging.getLogger(__name__)

SETUP = (OperatorMode.MANUAL, OperatorMode.CALIBRATE)


class ModeSwitch:
    def __init__(
        self,
        hub: Hub,
        manual: ManualControl,
        on_change: Callable[[OperatorMode], None] | None = None,
    ):
        self.hub, self.manual = hub, manual
        self._on_change = on_change
        self._lock = threading.RLock()
        with self._lock:
            mode = hub.mode()
            if mode in SETUP:
                self._enter_setup()
            if on_change is not None:
                on_change(mode)

    @property
    def mode(self) -> OperatorMode:
        return self.hub.mode()

    def _enter_setup(self) -> None:
        arm = self.manual.arm
        if not getattr(arm, "connected", True):
            arm.start()  # motors on; the state machine's Start does the same in a run mode

    def set(self, mode: OperatorMode | str) -> OperatorMode:
        """Switch to `mode`; WrongMode if it can't be done now (a run or a motion is going)."""
        mode = OperatorMode(mode)
        with self._lock:
            now = self.hub.mode()
            if mode is now:
                return mode
            if now in SETUP and self.manual.busy:
                raise WrongMode("the arm is moving: wait until it stops")
            self.hub.set_mode(mode)  # raises if a run is going
            if mode in SETUP:
                try:
                    self._enter_setup()
                except Exception:
                    self.hub.set_mode(now)
                    raise
            if self._on_change is not None:
                self._on_change(mode)
        return mode

    @contextlib.contextmanager
    def guard(self, *allowed: OperatorMode) -> Iterator[None]:
        """Run the body only in one of the `allowed` modes, with no mode change meanwhile."""
        with self._lock:
            mode = self.hub.mode()
            if mode not in allowed:
                names = " or ".join(m.value for m in allowed)
                raise WrongMode(f"this works in the {names} mode; the mode is {mode.value}")
            yield
