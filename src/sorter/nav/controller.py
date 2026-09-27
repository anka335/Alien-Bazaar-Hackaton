"""The approach algorithm: bring a sock into the arm's pick spot using the camera and the
navigation commands only.

    LOOK ─ sock seen? ─ no ─► close a moment ago? ─ yes ─► LOST: back off once, LOOK
     │ yes                     │ no
     │                         ▼
     │                        SEARCH: `scan` (one command, a full circle of frames), turn to
     │                        the nearest sock in them; nothing all round ─► EXPLORE open floor
     ▼
    cut off by the image's side? ─ yes ─► CENTER: turn toward it, LOOK (its estimate is biased)
     │ no
     ▼
    in the goal zone (camera estimate)? ─ yes ─► DONE
     │ no
     ▼
    far (> APPROACH_M)? ─ yes ─► APPROACH: go part of the way, LOOK again
     │ no
     ▼
    FINAL: one go_to_pixel on the sock's floor center, stopping at the pick distance (it backs
    up when the sock is too close); repeat until the camera sees it in the zone

Every move aims at the sock's center on the floor (the detector's x, y) projected back into
the frame, not at the blob's pixel centroid: for an L-shaped sock those differ.

Decisions come from the camera only: the target is the nearest confident sock in the current
frame; the one memory is the last detection (a sock that was close and is gone is most likely
under the camera's view, right in front).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from sorter.nav.commands import FRONT_M, HALF_WIDTH_M, SOCK_MAX_H, Result, Rover
from sorter.nav.config import GoalConfig
from sorter.nav.detect import Detection

APPROACH_M = 1.2  # beyond this, go in legs and look again
LEG_FRACTION = 0.6
SCAN_STEP_DEG = 45.0  # the RGB's HFOV is 69 deg: steps overlap
MAX_STEPS = 40
CONFIDENT = 0.5  # of the best score in the frame
LOST_NEAR_M = 0.6  # a sock last seen closer than this and gone is under the camera's view
LOST_BACKUP_M = 0.25
EDGE_PX = 3  # a blob this close to the image's side is cut off
DONE_FRACTION = 0.7  # of the goal's half size: margin for the camera's estimate
MAX_CENTER = 3  # CENTER turns in a row before trusting a cut-off blob anyway
MAX_EXPLORE = 3
MAX_DETOURS = 4
DETOUR_DEG = 60.0


@dataclass
class Log:
    steps: list[dict] = field(default_factory=list)

    def add(self, state: str, res: Result | None, det: Detection | None, note: str = "") -> None:
        self.steps.append(
            {
                "state": state,
                "result": res.summary() if res else None,
                "target": det.summary() if det else None,
                "note": note,
            }
        )


class Approach:
    """Run with `run()`. `detector.detect(frame)` gives the socks in a frame."""

    def __init__(self, rover: Rover, detector, goal: GoalConfig, max_steps: int = MAX_STEPS):
        self.rover, self.detector, self.goal = rover, detector, goal
        rover.detector = detector  # `seek` and the pixel commands look with the same detector
        self.max_steps = max_steps
        self.log = Log()
        self.last: Detection | None = None  # the last sock seen
        self.commands = 0

    def run(self) -> bool:
        res = self.rover.look()
        gx, gy = self.goal.center_m
        hx, hy = self.goal.half_size_m
        lost = centered = explored = detours = 0
        while self.commands < self.max_steps:
            det = self._pick(res.frame)
            if det is None:
                last, self.last = self.last, None
                if last is not None and last.x < LOST_NEAR_M and lost < 1:
                    lost += 1
                    res = self._do("LOST", "forward", -LOST_BACKUP_M, 0.15, det=last)
                    continue
                res, found = self._search()
                if found:
                    continue
                if explored >= MAX_EXPLORE:
                    break
                explored += 1
                res = self._explore(res)
                continue
            lost = 0
            side = self._cut_off(det, res.frame)
            if side and centered < MAX_CENTER and abs(det.bearing_deg) > 3:
                # the visible part's center lies toward the image's middle: turn past it a bit
                centered += 1
                turn = det.bearing_deg + math.copysign(8.0, det.bearing_deg)
                res = self._do("CENTER", "turn", turn, 0.6, det=det, note=f"cut off {side}")
                continue
            centered = 0
            if abs(det.x - gx) <= hx * DONE_FRACTION and abs(det.y - gy) <= hy * DONE_FRACTION:
                self.log.add("DONE", None, det, "sock in the goal zone (camera estimate)")
                return True
            dist = det.distance
            if self._stuck(res, det) and detours < MAX_DETOURS:
                detours += 1
                res = self._detour(res, det)
                continue
            if dist > APPROACH_M:
                stop = max(gx, dist - LEG_FRACTION * dist)
                res = self._goto("APPROACH", res.frame, det, stop, 0.3)
            else:
                res = self._goto("FINAL", res.frame, det, gx, 0.15)
        self.log.add("GIVE_UP", None, None, f"no success in {self.commands} commands")
        return False

    # --- helpers ---

    def _do(
        self, state: str, name: str, *args, det: Detection | None = None, note: str = ""
    ) -> Result:
        self.commands += 1
        res = self.rover.run(name, *args)
        self.log.add(state, res, det, note)
        return res

    def _goto(self, state: str, frame, det: Detection, stop: float, speed: float) -> Result:
        """go_to_pixel on the pixel where the sock's floor center is (maybe outside the image:
        go_to_pixel then uses the floor ray, which is right for a point on the floor)."""
        uv = frame.project((det.x, det.y, 0.0))
        u, v = uv if uv is not None else (det.u, det.v)
        # the algorithm aims at the sock's floor center itself: no click snapping, no far re-find
        return self._do(
            state, "go_to_pixel", float(u), float(v), stop, speed, 0.0, math.inf, False, det=det
        )

    def _search(self) -> tuple[Result, bool]:
        """A full circle of frames; turn to the nearest sock in them. (result, found)."""
        res = self._do("SEARCH", "scan", SCAN_STEP_DEG)
        best: tuple[float, Detection] | None = None
        for heading, frame in res.frames:
            det = self._pick(frame, remember=False)
            if det is not None and (best is None or det.distance < best[1].distance):
                best = (heading, det)
        if best is None:
            return res, False
        heading, det = best
        turn = (heading + det.bearing_deg + 180) % 360 - 180
        return self._do("SEARCH", "turn", turn, 0.8, det=det, note=f"seen at {heading:+.0f}"), True

    def _explore(self, scan: Result) -> Result:
        """Nothing in sight all round: turn to the scan heading with the longest free run of
        floor in the rover's path and drive along it (a sock may hide behind something)."""
        best, best_h = -1.0, 0.0
        for heading, frame in scan.frames:
            h = (heading + 180) % 360 - 180
            free = self._free_ahead(frame)
            score = min(free, 2.5) - 0.4 * abs(h) / 180  # ties: keep going the same way
            if score > best:
                best, best_h, best_free = score, h, free
        if abs(best_h) > 1:
            self._do("EXPLORE", "turn", best_h, 0.8, note=f"free {best_free:.2f} m")
        return self._do("EXPLORE", "forward", float(np.clip(best_free - 0.5, 0.3, 1.5)), 0.3)

    @staticmethod
    def _free_ahead(frame) -> float:
        """Floor run (m, from the rover's center) before something taller than a sock in the
        rover's path, as far as this frame's depth sees (4 m if nothing)."""
        p = frame.points()
        x, y, z = p[..., 0], p[..., 1], p[..., 2]
        hit = (np.abs(y) < HALF_WIDTH_M + 0.08) & (z > SOCK_MAX_H) & (z < 0.6) & ~np.isnan(x)
        return float(np.percentile(x[hit], 2)) if np.count_nonzero(hit) > 40 else 4.0

    @staticmethod
    def _stuck(res: Result, det: Detection) -> bool:
        """The last move stopped for an obstacle short of a sock that is still well ahead."""
        if res.command != "go_to_pixel" or not (res.blocked or "").startswith("obstacle"):
            return False
        return det.distance > FRONT_M + 0.45

    def _detour(self, res: Result, det: Detection) -> Result:
        """Step around what blocks the way: turn off the line to the side where the obstacle
        ends sooner, drive until the rover's side clears it, turn back toward the sock."""
        p = res.frame.points()
        x, y, z = p[..., 0], p[..., 1], p[..., 2]
        tall = (z > SOCK_MAX_H) & (z < 0.6) & (x < FRONT_M + 0.8) & ~np.isnan(x)
        ys = y[tall]
        if ys.size < 40:  # nothing in view (it is under the camera): sidestep toward the sock
            left_edge, right_edge = 0.3, -0.3
        else:
            left_edge, right_edge = float(np.percentile(ys, 98)), float(np.percentile(ys, 2))
        # which way round is shorter, with the sock's side as the tie-break
        go_left = left_edge < -right_edge or (abs(left_edge + right_edge) < 0.05 and det.y > 0)
        edge = left_edge if go_left else -right_edge
        lateral = max(edge, 0.0) + HALF_WIDTH_M + 0.15
        a = DETOUR_DEG if go_left else -DETOUR_DEG
        d = float(np.clip(lateral / math.sin(math.radians(DETOUR_DEG)), 0.3, 1.0))
        note = f"obstacle y {right_edge:+.2f}..{left_edge:+.2f}"
        self._do("DETOUR", "turn", a, 0.6, det=det, note=note)
        self._do("DETOUR", "forward", d, 0.2, det=det)
        return self._do("DETOUR", "turn", -a, 0.6, det=det)

    @staticmethod
    def _cut_off(det: Detection, frame) -> str:
        x, _, w, _ = det.bbox
        if x <= EDGE_PX:
            return "left"
        if x + w >= frame.K.width - EDGE_PX:
            return "right"
        return ""

    def _pick(self, frame, remember: bool = True) -> Detection | None:
        dets = [d for d in self.detector.detect(frame) if d.x is not None]
        if not dets:
            return None
        best = max(d.score for d in dets)
        det = min((d for d in dets if d.score >= CONFIDENT * best), key=lambda d: d.distance)
        if remember:
            self.last = det
        return det
