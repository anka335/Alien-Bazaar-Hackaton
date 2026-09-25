"""Placeholder state machine from block 0, enough for the smoke test. Block 6 replaces it.

Follows docs/architecture.md → Main loop. Missing on purpose (block 6 scope): run logs,
per-failure-path tests, finer control semantics.
"""

from __future__ import annotations

import logging
import threading
import time
import uuid
from dataclasses import dataclass

from sorter.core.errors import EStopped, SorterError, TargetRejected
from sorter.core.system import System
from sorter.core.types import (
    BackgroundResult,
    BoxResult,
    BoxStatus,
    ColorClass,
    Command,
    Decision,
    ItemResult,
    Mode,
    Observation,
    Overlay,
    Phase,
    PixelPoint,
    Status,
    Zone,
)

log = logging.getLogger(__name__)

_STOPPED = (Phase.IDLE, Phase.DONE, Phase.HELD, Phase.ERROR)


@dataclass(frozen=True)
class PlacedFromBox:
    px: PixelPoint


@dataclass(frozen=True)
class Dropped:
    color: ColorClass
    n_before: int


class StateMachine:
    def __init__(self, system: System):
        self.s = system
        self.cfg = system.cfg.state_machine
        self.phase = Phase.IDLE  # the phase running now, or the last one run
        self.next: Phase | None = None  # the phase to run next
        self.mode: Mode = "idle"
        self.error: str | None = None
        self.run_id: str | None = None
        self._reset_run()
        self._publish()

    def _reset_run(self) -> None:
        self.cycle = 0
        self.counters = {c: 0 for c in ColorClass}
        self.failures = 0
        self.avoid: list[PixelPoint] = []
        self.pending: PlacedFromBox | Dropped | None = None
        self.empty_streak = 0
        self.last_cycle_s: float | None = None
        self._cycle_t0: float | None = None
        self.obs: Observation | None = None
        self.bg: BackgroundResult | None = None
        self.item: ItemResult | None = None
        self.box: BoxResult | None = None

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
                self._reset_run()
                self.run_id = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:4]
                self.error = None
                self.next, self.mode = Phase.STARTING, "running"
            elif cmd is Command.PAUSE and self.mode == "running":
                self.mode = "paused"
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
            elif cmd is Command.RESET and self.phase in (Phase.HELD, Phase.ERROR):
                self.s.arm.recover()
                self.failures, self.error = 0, None
                self.phase, self.next, self.mode = Phase.LOOK_BG, Phase.LOOK_BG, "running"
            elif cmd is Command.HOLD:  # normally never queued: the Hub calls arm.hold() itself
                self.s.arm.hold()
            else:
                log.warning("command %s ignored in %s / %s", cmd, self.phase, self.mode)
        except SorterError as e:
            self._fail(f"{cmd}: {e}")
        self._publish()
        return step

    def _run_phase(self) -> None:
        phase = self.next
        assert phase is not None
        self.phase, self.next = phase, None
        self._publish()
        try:
            self.next = getattr(self, f"_{phase.value}")()
        except EStopped:
            log.warning("arm held during %s", phase)
            self.phase, self.next, self.mode = Phase.HELD, Phase.LOOK_BG, "paused"
        except SorterError as e:
            self._fail(f"{phase}: {e}")
        else:
            if self.failures >= self.cfg.max_consecutive_failures:
                self._fail(f"{self.failures} consecutive failures")
        self._publish()

    def _fail(self, msg: str) -> None:
        log.error(msg)
        self.error = msg
        self.phase, self.next, self.mode = Phase.ERROR, Phase.LOOK_BG, "paused"

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

    def _decide(self, overlay: Overlay, summary: str) -> None:
        assert self.obs is not None
        self.s.hub.publish_decision(Decision(self.phase, self.obs, overlay, summary))

    # --- phases: each returns the next phase ---

    def _starting(self) -> Phase:
        self.s.arm.start()
        self.s.arm.home()
        return Phase.LOOK_BG

    def _look_bg(self) -> Phase:
        now = time.monotonic()
        if self._cycle_t0 is not None:
            self.last_cycle_s = now - self._cycle_t0
        self._cycle_t0 = now
        self.cycle += 1
        self.obs = self.s.observer.observe(Zone.BACKGROUND)
        return Phase.SENSE_BG

    def _sense_bg(self) -> Phase:
        assert self.obs is not None
        self.bg = self.s.color_classifier.classify(self.obs.frame)
        self._resolve_pending(len(self.bg.items))
        if not self.bg.items:
            self._decide(self.bg.overlay, "background empty")
            return Phase.LOOK_BOX
        self.item = self.bg.items[0]
        self._decide(self.bg.overlay, f"{self.item.color} {self.item.confidence:.2f}")
        if self.item.confidence < 0.5:
            log.warning(
                "low color confidence %.2f, sorting as %s", self.item.confidence, self.item.color
            )
        return Phase.PICK_FROM_BG

    def _resolve_pending(self, n_items: int) -> None:
        pending, self.pending = self.pending, None
        match pending:
            case PlacedFromBox(px) if n_items == 0:
                log.warning("missed grasp from the box at (%d, %d)", px.u, px.v)
                self.avoid.append(px)
                self.failures += 1
            case PlacedFromBox():
                self.avoid.clear()
                self.failures = 0
            case Dropped(color, n_before) if n_items < n_before:
                self.counters[color] += 1
                self.failures = 0
            case Dropped():
                log.warning("pick from the background failed, retrying")
                self.failures += 1

    def _pick_from_bg(self) -> Phase:
        assert self.obs is not None and self.item is not None
        target = self.s.calibration.to_arm(self.obs, self.item.grasp)
        if self.s.arm.pick(target, Zone.BACKGROUND).likely_empty:
            log.warning("gripper empty after pick from the background")
            self.failures += 1
            return Phase.LOOK_BG
        return Phase.DROP_TO_BIN

    def _drop_to_bin(self) -> Phase:
        assert self.bg is not None and self.item is not None
        self.s.arm.drop_to_bin(self.item.color)
        self.pending = Dropped(self.item.color, len(self.bg.items))
        return Phase.LOOK_BG

    def _look_box(self) -> Phase:
        self.obs = self.s.observer.observe(Zone.BOX)
        return Phase.SENSE_BOX

    def _sense_box(self) -> Phase:
        assert self.obs is not None
        self.box = self.s.box_detector.detect(self.obs.frame, tuple(self.avoid))
        self._decide(self.box.overlay, f"box: {self.box.status}")
        match self.box.status:
            case BoxStatus.GRASP:
                self.empty_streak = 0
                return Phase.PICK_FROM_BOX
            case BoxStatus.EMPTY:
                self.empty_streak += 1
                if self.empty_streak >= self.cfg.empty_confirmations:
                    return Phase.DONE
                return Phase.LOOK_BOX
            case _:  # NO_GRASP
                if not self.avoid:
                    raise SorterError("cloth in the box but no grasp candidate")
                log.warning("no grasp left, clearing %d avoided points", len(self.avoid))
                self.avoid.clear()
                return Phase.SENSE_BOX

    def _pick_from_box(self) -> Phase:
        assert self.obs is not None and self.box is not None and self.box.grasp is not None
        px = self.box.grasp.px
        target = self.s.calibration.to_arm(self.obs, self.box.grasp)
        try:
            result = self.s.arm.pick(target, Zone.BOX)
        except TargetRejected as e:
            log.warning("grasp at (%d, %d) rejected: %s", px.u, px.v, e)
            self.avoid.append(px)
            self.failures += 1
            return Phase.SENSE_BOX  # same observation
        if result.likely_empty:
            log.warning("gripper empty after pick from the box")
            self.avoid.append(px)
            self.failures += 1
            return Phase.LOOK_BOX
        self._grasp_px = px
        return Phase.PLACE_ON_BG

    def _place_on_bg(self) -> Phase:
        self.s.arm.place_on_background()
        self.pending = PlacedFromBox(self._grasp_px)
        return Phase.LOOK_BG

    def _done(self) -> None:
        self.s.arm.home()
        self.mode = "idle"
        log.info("run %s done: %s", self.run_id, dict(self.counters))
