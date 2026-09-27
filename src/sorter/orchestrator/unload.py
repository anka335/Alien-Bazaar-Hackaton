"""The unload loop (stage B): every sock from cargo compartment c into laundry bin c.

Stage 0's baseline: look into the cargo box, run the depth box detector on each compartment's
image area in turn, pick the first grasp found, drop it into that compartment's bin. Stage B
makes it good (look poses per compartment, verification, parking tolerance).

    LOOK_CARGO → SENSE_CARGO → PICK_FROM_CARGO → DROP_TO_LAUNDRY → LOOK_CARGO …
    every compartment empty × N → DONE
"""

from __future__ import annotations

import logging
import math

import numpy as np

from sorter.box_detector.detector import DepthBoxDetector
from sorter.core.errors import TargetRejected
from sorter.core.types import BoxResult, BoxStatus, ColorClass, Observation, Phase, PixelPoint, Zone
from sorter.orchestrator.state_machine import Loop
from sorter.sim.config import RectConfig

log = logging.getLogger(__name__)


def project_rect(obs: Observation, rect: RectConfig, z_mm: float) -> list[tuple[int, int]]:
    """The corners of `rect` at height `z_mm`, in the image of `obs`, clipped to the image."""
    assert obs.T_base_cam is not None
    k = obs.frame.intrinsics
    T = np.linalg.inv(obs.T_base_cam)
    x0, x1, y0, y1 = rect.bounds()
    out = []
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        c = T @ np.array([x, y, z_mm, 1.0])
        u = k.fx * c[0] / c[2] + k.cx
        v = k.fy * c[1] / c[2] + k.cy
        out.append(
            (int(np.clip(round(u), 0, k.width - 1)), int(np.clip(round(v), 0, k.height - 1)))
        )
    return out


class UnloadLoop(Loop):
    first = Phase.LOOK_CARGO

    def reset(self) -> None:
        self.avoid: list[PixelPoint] = []
        self.empty_streak = 0
        self.box: BoxResult | None = None
        self.color: ColorClass | None = None

    def _look_cargo(self) -> Phase:
        self.sm.new_cycle()
        self.sm.obs = self.s.observer.observe(Zone.CARGO)
        return Phase.SENSE_CARGO

    def _sense_cargo(self) -> Phase:
        sm = self.sm
        obs = sm.obs
        assert obs is not None
        cargo = sm.s.cfg.sim.layout.cargo  # the rover layout (the rig is computed from it)
        results = []
        for color in cargo.compartments:
            roi = project_rect(obs, cargo.compartment(color), cargo.floor_z_mm)
            box = DepthBoxDetector(sm.s.cfg.box_detector, roi).detect(obs.frame, self.avoid)
            results.append((color, box))
            if box.status is BoxStatus.GRASP:
                break
        color, box = results[-1]
        self.box, self.color = box, color
        if box.status is BoxStatus.GRASP:
            assert box.grasp is not None
            self.empty_streak = 0
            g = box.grasp
            summary = f"{color}: grasp ({g.px.u}, {g.px.v}) depth {g.depth_mm:.0f} mm"
            return sm.decide(box, box.overlay, summary, Phase.PICK_FROM_CARGO)
        if any(b.status is BoxStatus.NO_GRASP for _, b in results) and self.avoid:
            log.warning("no grasp left, clearing %d avoided points", len(self.avoid))
            self.avoid.clear()
            return sm.decide(box, box.overlay, "no grasp, clearing avoided", Phase.SENSE_CARGO)
        self.empty_streak += 1
        n = sm.cfg.empty_confirmations
        nxt = Phase.DONE if self.empty_streak >= n else Phase.LOOK_CARGO
        return sm.decide(box, box.overlay, f"cargo empty ({self.empty_streak}/{n})", nxt)

    def _pick_from_cargo(self) -> Phase:
        sm = self.sm
        assert sm.obs is not None and self.box is not None and self.box.grasp is not None
        px = self.box.grasp.px
        target = self.s.calibration.to_arm(sm.obs, self.box.grasp)
        try:  # the fingers open along the compartment's long side (y)
            result = self.s.arm.pick(target, Zone.CARGO, math.pi / 2)
        except TargetRejected as e:
            log.warning("grasp at (%d, %d) rejected: %s", px.u, px.v, e)
            self.avoid.append(px)
            sm.failures += 1
            return Phase.SENSE_CARGO  # same observation
        if result.likely_empty:
            log.warning("gripper empty after the pick from the cargo box")
            self.avoid.append(px)
            sm.failures += 1
            return Phase.LOOK_CARGO
        return Phase.DROP_TO_LAUNDRY

    def _drop_to_laundry(self) -> Phase:
        assert self.color is not None
        self.s.arm.drop_to_laundry(self.color)
        self.sm.counters[self.color] += 1  # not verified yet (stage B: B3)
        self.sm.failures = 0
        self.avoid.clear()
        return Phase.LOOK_CARGO
