"""State machine: runs the loop of the operator mode (load or unload), see docs/architecture.md
→ Main loop.

This file is the part both loops share: commands (start, pause, step, stop, reset), hold and
errors, the status for the dashboard, the run log. The phases themselves are in `load.py`
(stage A) and `unload.py` (stage B). Each phase is atomic; commands are applied between phases.
"""

from __future__ import annotations

import logging
import threading
import time
import uuid
from typing import Any

from sorter.core.errors import EStopped, SorterError
from sorter.core.system import System
from sorter.core.types import (
    RUN_MODES,
    ColorClass,
    Command,
    Decision,
    Mode,
    Observation,
    OperatorMode,
    Overlay,
    Phase,
    Status,
)
from sorter.orchestrator.runlog import RunLog

log = logging.getLogger(__name__)

_STOPPED = (Phase.IDLE, Phase.DONE, Phase.HELD, Phase.ERROR)


class Loop:
    """One mode's phases. A phase is a method `_<phase value>` that returns the next phase;
    it may use and update the shared run state on `self.sm` (counters, failures, obs, cycle)."""

    first: Phase  # the first phase of a run, and where a run goes on after a hold or an error

    def __init__(self, sm: StateMachine):
        self.sm = sm
        self.s = sm.s

    def reset(self) -> None:
        """Clear the loop's own memory at the start of a run."""

    def run(self, phase: Phase) -> Phase:
        return getattr(self, f"_{phase.value}")()


class StateMachine:
    def __init__(self, system: System):
        from sorter.orchestrator.load import LoadLoop
        from sorter.orchestrator.unload import UnloadLoop

        self.s = system
        self.cfg = system.cfg.state_machine
        self.loops: dict[OperatorMode, Loop] = {
            OperatorMode.LOAD: LoadLoop(self),
            OperatorMode.UNLOAD: UnloadLoop(self),
        }
        self.loop: Loop = self.loops[OperatorMode.LOAD]
        self.phase = Phase.IDLE  # the phase running now, or the last one run
        self.next: Phase | None = None  # the phase to run next
        self.mode: Mode = "idle"
        self.error: str | None = None
        self.run_id: str | None = None
        self.runlog: RunLog | None = None
        self._reset_run()
        self._publish()

    def _reset_run(self) -> None:
        self.cycle = 0
        self.counters = {c: 0 for c in ColorClass}
        self.failures = 0
        self.last_cycle_s: float | None = None
        self._cycle_t0: float | None = None
        self.obs: Observation | None = None
        self.loop.reset()

    # --- loop ---

    def run(self, stop: threading.Event) -> None:
        while not stop.is_set():
            self.poll(timeout_s=0.1)

    def poll(self, timeout_s: float = 0.0) -> None:
        """Apply at most one command, then run at most one phase."""
        cmd = self.s.hub.next_command(0.0 if self.mode == "running" else timeout_s)
        step = cmd is not None and self._apply(cmd)
        if (self.mode == "running" or step) and self.next is not None:
            self._run_phase()

    def _apply(self, cmd: Command) -> bool:
        """Apply a command between phases. True means: run one phase (STEP)."""
        log.info("command %s", cmd)
        step = False
        try:
            if cmd is Command.START and self.mode == "idle":
                self._start_run()
            elif cmd is Command.PAUSE and self.mode == "running":
                self.mode = "paused"
                self._cycle_t0 = None  # a paused cycle is not timed
            elif cmd is Command.RESUME and self.mode == "paused" and self.phase is not Phase.HELD:
                if self.phase is Phase.ERROR:
                    self.failures, self.error = 0, None
                self.mode = "running"
            elif cmd is Command.STEP and self.mode == "paused" and self.phase not in _STOPPED:
                step = True
            elif cmd is Command.STOP and self.mode != "idle":
                if self.phase is Phase.HELD:
                    self.s.arm.recover()
                else:
                    self.s.arm.home()
                self.phase, self.next, self.mode = Phase.IDLE, None, "idle"
                self._finish_run("stopped")
            elif cmd is Command.RESET and self.phase in (Phase.HELD, Phase.ERROR):
                self.s.arm.recover()
                self.failures, self.error = 0, None
                self.phase, self.next, self.mode = self.loop.first, self.loop.first, "running"
            elif cmd is Command.HOLD:  # normally never queued: the Hub calls arm.hold() itself
                self.s.arm.hold()
            else:
                log.warning("command %s ignored in %s / %s", cmd, self.phase, self.mode)
        except Exception as e:
            self._fail(f"{cmd}: {e}", unexpected=not isinstance(e, SorterError))
        self._publish()
        return step

    def _start_run(self) -> None:
        mode = self.s.hub.mode()
        if mode not in RUN_MODES:  # the Hub queues commands in a run mode only
            raise SorterError(f"no loop for the {mode} mode")
        self.loop = self.loops[mode]
        self._reset_run()
        self.run_id = time.strftime("%Y%m%d-%H%M%S-") + mode.value + "-" + uuid.uuid4().hex[:4]
        self.error = None
        self.next, self.mode = Phase.STARTING, "running"
        self.runlog = RunLog(self.cfg.runs_dir, self.run_id) if self.cfg.save_runs else None
        if self.runlog:
            self.runlog.start(self.s.cfg.model_dump(mode="json"))
        log.info("run %s started", self.run_id)

    def _finish_run(self, reason: str) -> None:
        log.info(
            "run %s %s: %s", self.run_id, reason, {c.value: n for c, n in self.counters.items()}
        )
        if self.runlog:
            self.runlog.finish(reason, counters=self.counters, cycles=self.cycle)

    def _run_phase(self) -> None:
        phase = self.next
        assert phase is not None
        self.phase, self.next = phase, None
        self._publish()
        try:
            if phase is Phase.STARTING:
                self.next = self._starting()
            elif phase is Phase.DONE:
                self._done()
            else:
                self.next = self.loop.run(phase)
        except EStopped:
            log.warning("arm held during %s", phase)
            self.phase, self.next, self.mode = Phase.HELD, self.loop.first, "paused"
            self._cycle_t0 = None
        except Exception as e:  # a bug in a module must not kill the loop thread
            self._fail(f"{phase}: {e}", unexpected=not isinstance(e, SorterError))
        else:
            if self.failures >= self.cfg.max_consecutive_failures:
                self._fail(f"{self.failures} consecutive failures")
        self._publish()

    def _fail(self, msg: str, unexpected: bool = False) -> None:
        if unexpected:
            log.exception(msg)
        else:
            log.error(msg)
        self.error = msg
        self.phase, self.next, self.mode = Phase.ERROR, self.loop.first, "paused"
        self._cycle_t0 = None

    def _publish(self) -> None:
        self.s.hub.publish_status(
            Status(
                phase=self.phase,
                next_phase=self.next,
                mode=self.mode,
                run_id=self.run_id,
                cycle=self.cycle,
                counters=dict(self.counters),
                failures=self.failures,
                last_cycle_s=self.last_cycle_s,
                error=self.error,
            )
        )

    # --- helpers for the loops ---

    def new_cycle(self) -> None:
        """A cycle starts (one item handled): count it and time the last one."""
        now = time.monotonic()
        if self._cycle_t0 is not None and self.mode == "running":
            self.last_cycle_s = now - self._cycle_t0
        self._cycle_t0 = now
        self.cycle += 1

    def decide(
        self, result: Any, overlay: Overlay, summary: str, next_phase: Phase, **extra: Any
    ) -> Phase:
        """Publish the decision frame for `self.obs` and log it. Returns `next_phase`."""
        assert self.obs is not None
        self.s.hub.publish_decision(Decision(self.phase, self.obs, overlay, summary))
        if self.runlog:
            self.runlog.record(
                self.cycle,
                self.phase.value,
                self.obs,
                result=result,
                summary=summary,
                next_phase=next_phase,
                counters=self.counters,
                failures=self.failures,
                **extra,
            )
        return next_phase

    # --- the phases every loop shares ---

    def _starting(self) -> Phase:
        self.s.arm.start()
        self.s.arm.home()
        return self.loop.first

    def _done(self) -> None:
        self.s.arm.home()
        self.mode = "idle"
        self._finish_run("done")
