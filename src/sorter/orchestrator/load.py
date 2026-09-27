"""The load loop (stage A): socks from the floor into the cargo box (D-035).

Stage 0's baseline, observation-driven like the table loop (D-008): look at the floor, pick the
detector's best sock, drop it into the box, and count it once the next look shows one
sock fewer. Stage A makes it good (scan poses, grasp yaw, verification, retries).

    SCAN → SENSE_FLOOR → PICK_FROM_FLOOR → DROP_TO_CARGO → SCAN …   no sock × N → DONE
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

from sorter.core.errors import TargetRejected
from sorter.core.types import ColorClass, FloorResult, Phase, PixelPoint, Sock, Zone
from sorter.orchestrator.state_machine import Loop

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Dropped:
    color: ColorClass
    n_before: int  # socks in view before the pick


class LoadLoop(Loop):
    first = Phase.SCAN

    def reset(self) -> None:
        self.avoid: list[PixelPoint] = []
        self.pending: Dropped | None = None
        self.empty_streak = 0
        self.floor: FloorResult | None = None
        self.sock: Sock | None = None

    def _scan(self) -> Phase:
        self.sm.new_cycle()
        self.sm.obs = self.s.observer.observe(Zone.FLOOR)
        return Phase.SENSE_FLOOR

    def _sense_floor(self) -> Phase:
        sm = self.sm
        assert sm.obs is not None
        self.floor = floor = self.s.floor_detector.detect(sm.obs.frame, tuple(self.avoid))
        resolved = self._resolve_pending(len(floor.socks))
        if not floor.socks:
            self.empty_streak += 1
            n = sm.cfg.empty_confirmations
            nxt = Phase.DONE if self.empty_streak >= n else Phase.SCAN
            return sm.decide(floor, floor.overlay, f"no sock ({self.empty_streak}/{n})", nxt)
        self.empty_streak = 0
        self.sock = sock = floor.socks[0]
        if sock.confidence < sm.cfg.low_confidence:
            log.warning("low color confidence %.2f, loading as %s", sock.confidence, sock.color)
        summary = f"{sock.color} {sock.confidence:.2f} ({len(floor.socks)} in view)"
        return sm.decide(floor, floor.overlay, summary, Phase.PICK_FROM_FLOOR, resolved=resolved)

    def _resolve_pending(self, n_socks: int) -> str | None:
        """Check the last drop against what the camera sees now."""
        pending, self.pending = self.pending, None
        if pending is None:
            return None
        if n_socks < pending.n_before:
            self.sm.counters[pending.color] += 1
            self.sm.failures = 0
            self.avoid.clear()
            return f"verified: {pending.color} loaded"
        log.warning("the sock is still on the floor, retrying")
        self.sm.failures += 1
        return "not verified, retrying"

    def _pick_from_floor(self) -> Phase:
        sm = self.sm
        assert sm.obs is not None and self.sock is not None and self.floor is not None
        px = self.sock.grasp.px
        target = self.s.calibration.to_arm(sm.obs, self.sock.grasp)
        try:
            result = self.s.arm.pick(target, Zone.FLOOR)
        except TargetRejected as e:
            log.warning("grasp at (%d, %d) rejected: %s", px.u, px.v, e)
            self.avoid.append(px)
            sm.failures += 1
            return Phase.SCAN
        if result.likely_empty:
            log.warning("gripper empty after the pick from the floor")
            self.avoid.append(px)
            sm.failures += 1
            return Phase.SCAN
        return Phase.DROP_TO_CARGO

    def _drop_to_cargo(self) -> Phase:
        assert self.sock is not None and self.floor is not None
        self.s.arm.drop_to_cargo(self.sock.color)
        self.pending = Dropped(self.sock.color, len(self.floor.socks))
        return Phase.SCAN
