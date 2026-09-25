"""State machine: the main loop, see docs/architecture.md → Main loop.

Observation-driven (D-008): every cycle starts at the background; the box is visited only when the
background is empty. Memory between phases: counters, `avoid`, `pending`, `failures`,
`empty_streak`. Each phase is atomic; commands are applied between phases.
"""

from __future__ import annotations

import logging
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Any

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
    Phase,
    PixelPoint,
    Status,
    Zone,
)
from sorter.orchestrator.runlog import RunLog

log = logging.getLogger(__name__)

_STOPPED = (Phase.IDLE, Phase.DONE, Phase.HELD, Phase.ERROR)


@dataclass(frozen=True)
class PlacedFromBox:
    px: PixelPoint  # grasp pixel in the box view


@dataclass(frozen=True)
class Dropped:
    color: ColorClass
    n_before: int  # items on the background before the pick


class StateMachine:
    def __init__(self, system: System):
        self.s = system
        self.cfg = system.cfg.state_machine
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
        self.avoid: list[PixelPoint] = []
        self.pending: PlacedFromBox | Dropped | None = None
        self.empty_streak = 0
        self.last_cycle_s: float | None = None
        self._cycle_t0: float | None = None
        self.obs: Observation | None = None
        self.bg: BackgroundResult | None = None
        self.item: ItemResult | None = None
        self.box: BoxResult | None = None
        self._grasp_px: PixelPoint | None = None

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
                self.phase, self.next, self.mode = Phase.LOOK_BG, Phase.LOOK_BG, "running"
            elif cmd is Command.HOLD:  # normally never queued: the Hub calls arm.hold() itself
                self.s.arm.hold()
            else:
                log.warning("command %s ignored in %s / %s", cmd, self.phase, self.mode)
        except Exception as e:
            self._fail(f"{cmd}: {e}", unexpected=not isinstance(e, SorterError))
        self._publish()
        return step

    def _start_run(self) -> None:
        self._reset_run()
        self.run_id = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:4]
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
            self.next = getattr(self, f"_{phase.value}")()
        except EStopped:
            log.warning("arm held during %s", phase)
            self.phase, self.next, self.mode = Phase.HELD, Phase.LOOK_BG, "paused"
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
        self.phase, self.next, self.mode = Phase.ERROR, Phase.LOOK_BG, "paused"
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

    def _decide(
        self,
        result: BackgroundResult | BoxResult,
        summary: str,
        next_phase: Phase,
        **extra: Any,
    ) -> Phase:
        """Publish the decision frame and log it. Returns `next_phase`."""
        assert self.obs is not None
        self.s.hub.publish_decision(Decision(self.phase, self.obs, result.overlay, summary))
        if self.runlog:
            self.runlog.record(
                self.cycle,
                self.phase.value,
                self.obs,
                result=result,
                summary=summary,
                next_phase=next_phase,
                avoid=self.avoid,
                counters=self.counters,
                failures=self.failures,
                **extra,
            )
        return next_phase

    # --- phases: each returns the next phase ---

    def _starting(self) -> Phase:
        self.s.arm.start()
        self.s.arm.home()
        return Phase.LOOK_BG

    def _look_bg(self) -> Phase:
        now = time.monotonic()
        if self._cycle_t0 is not None and self.mode == "running":
            self.last_cycle_s = now - self._cycle_t0
        self._cycle_t0 = now
        self.cycle += 1
        self.obs = self.s.observer.observe(Zone.BACKGROUND)
        return Phase.SENSE_BG

    def _sense_bg(self) -> Phase:
        assert self.obs is not None
        self.bg = self.s.color_classifier.classify(self.obs.frame)
        resolved = self._resolve_pending(len(self.bg.items))
        if not self.bg.items:
            return self._decide(self.bg, "background empty", Phase.LOOK_BOX, resolved=resolved)
        self.item = self.bg.items[0]
        if self.item.confidence < self.cfg.low_confidence:
            log.warning(
                "low color confidence %.2f, sorting as %s", self.item.confidence, self.item.color
            )
        summary = f"{self.item.color} {self.item.confidence:.2f} ({len(self.bg.items)} on bg)"
        return self._decide(self.bg, summary, Phase.PICK_FROM_BG, resolved=resolved)

    def _resolve_pending(self, n_items: int) -> str | None:
        """Check the last action against what the camera sees now. Returns what was concluded."""
        pending, self.pending = self.pending, None
        match pending:
            case PlacedFromBox(px) if n_items == 0:
                log.warning("missed grasp from the box at (%d, %d)", px.u, px.v)
                self.avoid.append(px)
                self.failures += 1
                return "missed grasp from the box"
            case PlacedFromBox():
                self.avoid.clear()
                self.failures = 0
                return "placed from the box"
            case Dropped(color, n_before) if n_items < n_before:
                self.counters[color] += 1
                self.failures = 0
                return f"verified drop: {color}"
            case Dropped():
                log.warning("pick from the background failed, retrying")
                self.failures += 1
                return "drop not verified, retrying"
        return None

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
        self.box = box = self.s.box_detector.detect(self.obs.frame, tuple(self.avoid))
        match box.status:
            case BoxStatus.GRASP:
                assert box.grasp is not None
                self.empty_streak = 0
                g = box.grasp
                depth = "" if g.depth_mm is None else f" depth {g.depth_mm:.0f} mm"
                summary = f"grasp ({g.px.u}, {g.px.v}){depth}"
                return self._decide(box, summary, Phase.PICK_FROM_BOX)
            case BoxStatus.EMPTY:
                self.empty_streak += 1
                n = self.cfg.empty_confirmations
                summary = f"box empty ({self.empty_streak}/{n})"
                return self._decide(
                    box, summary, Phase.DONE if self.empty_streak >= n else Phase.LOOK_BOX
                )
            case _:  # NO_GRASP
                self.empty_streak = 0
                if not self.avoid:
                    self._decide(box, "no grasp candidate", Phase.ERROR)
                    raise SorterError("cloth in the box but no grasp candidate")
                log.warning("no grasp left, clearing %d avoided points", len(self.avoid))
                self._decide(box, f"no grasp, clearing {len(self.avoid)} avoided", Phase.SENSE_BOX)
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
        assert self._grasp_px is not None
        self.s.arm.place_on_background()
        self.pending = PlacedFromBox(self._grasp_px)
        return Phase.LOOK_BG

    def _done(self) -> None:
        self.s.arm.home()
        self.mode = "idle"
        self._finish_run("done")
