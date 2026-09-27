"""The load loop (stage A): socks from the floor all around the rover into the cargo box (D-040).

The rover stands still; only the arm moves. The loop looks at the floor from the scan poses
(`scan_1` … `scan_N`, a ring around the arm) one after another, turns every sock the floor
detector finds into a target in arm coordinates (grasp point, gripper yaw, color), and picks the
nearest one as soon as a view has one. Before the pick it aims the camera at the sock if the
scan view saw it cut off or far off-center. After the drop it checks: the spot on the floor is
empty, and the cargo box holds more cloth than before. Only then is the sock counted.

    SCAN(k) → SENSE_FLOOR ─sock─► [AIM] → PICK_FROM_FLOOR → DROP_TO_CARGO → CHECK_LOAD → SCAN(k)
                          └─no sock─► SCAN(k + 1) … a whole round of views empty → DONE

A sock that can't be picked (out of reach, the plan rejected, the gripper empty, still on the
floor after the drop) is tried `load.max_attempts` times, then left: its spot is avoided.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Sequence
from dataclasses import dataclass, field, replace

import cv2
import numpy as np

from sorter.arm.config import SCAN_POSES
from sorter.arm.controller import in_polygon
from sorter.core.errors import SorterError, TargetRejected
from sorter.core.types import (
    ArmPoint,
    ColorClass,
    FloorResult,
    GraspPoint,
    Observation,
    Overlay,
    Phase,
    PixelPoint,
    Sock,
    Zone,
)
from sorter.orchestrator.state_machine import Loop

log = logging.getLogger(__name__)


@dataclass
class Target:
    """A sock on the floor, in arm coordinates."""

    point: ArmPoint  # the grasp point on the cloth surface
    yaw: float | None  # the gripper's yaw across the sock, arm frame
    color: ColorClass
    confidence: float
    cut: bool  # seen cut off by the frame's edge
    off_center: float  # 0 at the image center .. 1 at a corner
    cloth_xy: np.ndarray  # Nx2: points of the sock's mask on the floor, arm frame (mm)
    attempts: int = 0

    def gap_mm(self, x: float, y: float) -> float:
        """How far (x, y) is from this sock's cloth: 0 on it."""
        if not len(self.cloth_xy):
            return math.hypot(x - self.point.x, y - self.point.y)
        return float(np.hypot(*(self.cloth_xy - (x, y)).T).min())

    def same_sock(self, other: Target, tol_mm: float) -> float | None:
        """The gap between the two socks' cloth if they are one sock seen twice (each one's
        grasp point lies on or near the other's cloth; a grasp point moves between views, the
        cloth doesn't), else None."""
        gap = min(
            self.gap_mm(other.point.x, other.point.y), other.gap_mm(self.point.x, self.point.y)
        )
        return gap if gap < tol_mm else None


@dataclass
class BoxView:
    """The cargo box seen from `look_cargo` (a fixed pose: the pixels line up between looks)."""

    height_mm: np.ndarray  # HxW: the surface above the box floor; NaN outside the box / no depth
    footprint_mm2: np.ndarray  # HxW: the floor area a pixel covers
    socks: int  # socks the floor detector tells apart in the box


@dataclass
class _Run:
    scan_k: int = 0  # the scan pose to look from next
    empty_views: int = 0  # views in a row without a sock
    target: Target | None = None
    tried: list[Target] = field(default_factory=list)  # socks that failed, with their attempts
    given_up: list[Target] = field(default_factory=list)  # left after `max_attempts`: ignored
    box: BoxView | None = None  # the cargo box from `look_cargo`, last check


class LoadLoop(Loop):
    first = Phase.SCAN

    def reset(self) -> None:
        self.r = _Run()
        self.floor: FloorResult | None = None
        self.scan_obs: Observation | None = None

    @property
    def cfg(self):
        return self.s.cfg.load

    # --- looking around ---

    def _scan(self) -> Phase:
        sm, r = self.sm, self.r
        if r.box is None:  # the box at the start: what a drop is compared with
            r.box = self.box_view(self.s.observer.observe(Zone.CARGO))
            log.info("cargo box at the start: %d sock(s)", r.box.socks)
        pose = SCAN_POSES[self.r.scan_k % len(SCAN_POSES)]
        sm.obs = self.scan_obs = self.s.observer.observe(Zone.FLOOR, pose)
        return Phase.SENSE_FLOOR

    def _sense_floor(self) -> Phase:
        sm, r = self.sm, self.r
        assert sm.obs is not None
        pose = SCAN_POSES[r.scan_k % len(SCAN_POSES)]
        self.floor = floor = self.s.floor_detector.detect(sm.obs.frame)
        targets = self._targets(sm.obs, floor)
        if targets:
            r.empty_views = 0
            sm.new_cycle()
            r.target = t = min(targets, key=lambda t: math.hypot(t.point.x, t.point.y))
            seen = self._same_sock(t, r.tried)
            t.attempts = seen.attempts if seen is not None else 0
            aim = t.cut or t.off_center > self.cfg.aim_off_center
            summary = (
                f"{pose}: {len(targets)} sock(s); {t.color} {t.confidence:.2f} at "
                f"({t.point.x:.0f}, {t.point.y:.0f})" + (", closer look" if aim else "")
            )
            nxt = Phase.AIM if aim else Phase.PICK_FROM_FLOOR
            return sm.decide(floor, floor.overlay, summary, nxt)
        r.empty_views += 1
        r.scan_k += 1
        n = len(SCAN_POSES)
        if r.empty_views >= n * self.cfg.empty_rounds:
            return sm.decide(floor, floor.overlay, f"{pose}: no sock; all around empty", Phase.DONE)
        return sm.decide(floor, floor.overlay, f"{pose}: no sock", Phase.SCAN)

    def _aim(self) -> Phase:
        """A closer look, the sock in the middle of the image: a better mask, grasp and yaw."""
        sm, r = self.sm, self.r
        t = r.target
        assert t is not None
        obs = self.s.observer.observe_point(Zone.FLOOR, t.point, self.cfg.aim_heights_mm)
        if obs is None:
            log.info("no closer look at (%.0f, %.0f): the scan view's grasp", t.point.x, t.point.y)
            return Phase.PICK_FROM_FLOOR
        sm.obs = obs
        floor = self.s.floor_detector.detect(obs.frame)
        best = self._same_sock(t, self._targets(obs, floor))
        if best is None:
            # counted as a failed attempt: a sock the closer look never finds is left after
            # `max_attempts`, the scan doesn't pick it again and again
            sm.decide(floor, floor.overlay, "not found from closer", Phase.SCAN)
            return self._failed(t, "not found from closer")
        best.attempts = t.attempts
        r.target = best
        summary = f"{best.color} {best.confidence:.2f} at ({best.point.x:.0f}, {best.point.y:.0f})"
        return sm.decide(floor, floor.overlay, summary, Phase.PICK_FROM_FLOOR)

    # --- pick, drop, check ---

    def _pick_from_floor(self) -> Phase:
        t = self.r.target
        assert t is not None
        log.info(
            "pick %s at (%.0f, %.0f, %.0f), yaw %s, attempt %d",
            t.color,
            t.point.x,
            t.point.y,
            t.point.z,
            "any" if t.yaw is None else f"{math.degrees(t.yaw):.0f}°",
            t.attempts + 1,
        )
        result = None
        for yaw in self._yaws(t.yaw):  # a rejected pick doesn't move the arm: the next yaw
            try:
                result = self.s.arm.pick(t.point, Zone.FLOOR, yaw)
                break
            except TargetRejected as e:
                err = e
        if result is None:
            log.warning("pick at (%.0f, %.0f) rejected: %s", t.point.x, t.point.y, err)
            return self._failed(t, "rejected")
        if result.likely_empty:
            log.warning("gripper empty after the pick at (%.0f, %.0f)", t.point.x, t.point.y)
            self.s.arm.home()
            return self._failed(t, "missed")
        return Phase.DROP_TO_CARGO

    @staticmethod
    def _yaws(yaw: float | None) -> list[float | None]:
        """The detector's yaw, then ever further off it up to 90° (fingers along the sock still
        pinch cloth), then any: near the rover the 184 mm finger rail rules out some yaws."""
        if yaw is None:
            return [None]
        offs = [0] + [s * d for d in range(15, 91, 15) for s in (1, -1)][:-1]  # ±90° is one
        return [yaw + math.radians(d) for d in offs] + [None]

    def _drop_to_cargo(self) -> Phase:
        t = self.r.target
        assert t is not None
        self.s.arm.drop_to_cargo(t.color)
        return Phase.CHECK_LOAD

    def _check_load(self) -> Phase:
        """The spot on the floor must be empty and the box must hold more cloth than before."""
        sm, r = self.sm, self.r
        t = r.target
        assert t is not None
        obs = self.s.observer.observe_point(Zone.FLOOR, t.point, self.cfg.aim_heights_mm)
        if obs is not None:
            floor = self.s.floor_detector.detect(obs.frame)
            if self._same_sock(t, self._targets(obs, floor)) is not None:
                sm.obs = obs
                sm.decide(floor, floor.overlay, "still on the floor", Phase.SCAN)
                return self._failed(t, "still on the floor")
        sm.obs = box_obs = self.s.observer.observe(Zone.CARGO)
        before, after = r.box, self.box_view(box_obs)
        r.box = after
        # a new sock lies on top: where it landed the surface rose (the socks under it may have
        # settled elsewhere, so the volume says little); or it slid into a gap: one more sock
        raised = np.zeros(after.height_mm.shape, dtype=bool)
        if before is not None:
            with np.errstate(invalid="ignore"):
                raised = after.height_mm - before.height_mm >= self.cfg.raised_mm
        area = float(after.footprint_mm2[raised].sum())
        more = after.socks - (before.socks if before is not None else 0)
        overlay = Overlay(mask=raised)
        if area >= self.cfg.min_raised_mm2 or more > 0:
            sm.counters[t.color] += 1
            sm.failures = 0
            if (seen := self._same_sock(t, r.tried)) is not None:
                r.tried.remove(seen)
            summary = f"verified: {t.color} in the box ({area / 100:.0f} cm² rose, {more:+d} sock)"
        else:
            # it left the spot but didn't reach the box: on the rover or elsewhere on the floor,
            # where a later look finds it
            log.warning("the sock left the floor but isn't seen in the box (%.0f mm² rose)", area)
            sm.failures += 1
            summary = f"not in the box ({area / 100:.0f} cm² rose)"
        overlay.text.append(summary)
        r.target = None
        return sm.decide(None, overlay, summary, Phase.SCAN)

    def _failed(self, t: Target, why: str) -> Phase:
        r, sm = self.r, self.sm
        sm.failures += 1
        # the sock as seen now (its grasp point moves between views) with one attempt more
        if (seen := self._same_sock(t, r.tried)) is not None:
            r.tried.remove(seen)
        t.attempts += 1
        r.tried.append(t)
        if t.attempts >= self.cfg.max_attempts:
            log.warning(
                "sock at (%.0f, %.0f) left after %d attempts (%s)",
                t.point.x,
                t.point.y,
                t.attempts,
                why,
            )
            r.tried.remove(t)
            r.given_up.append(t)
            sm.failures = 0  # given up: not a streak of failures any more
        r.target = None
        return Phase.SCAN

    # --- helpers ---

    def _same_sock(self, t: Target, seen: list[Target]) -> Target | None:
        """The one of `seen` that is the sock `t` (its cloth closest to t's), or None."""
        gaps = [(g, u) for u in seen if (g := t.same_sock(u, self.cfg.same_sock_mm)) is not None]
        return min(gaps, key=lambda gu: (gu[0], gu[1].off_center))[1] if gaps else None

    def _targets(self, obs: Observation, floor: FloorResult) -> list[Target]:
        """The detected socks that lie on the floor where the arm may pick, in arm coordinates."""
        out = []
        cal = self.s.calibration
        zone = self.s.cfg.zones[Zone.FLOOR]
        floor_z = zone.z_floor_mm - 8.0  # the fingertips stop 8 mm above the floor
        h, w = obs.frame.depth_mm.shape
        for sock in floor.socks:
            try:
                p = cal.to_arm(obs, sock.grasp)
            except SorterError as e:
                log.warning("sock without arm coordinates: %s", e)
                continue
            if not (floor_z - 20 <= p.z <= floor_z + self.cfg.max_sock_height_mm):
                continue  # not on the floor (on the rover, in the box)
            if not in_polygon(p.x, p.y, zone.workspace_mm):
                # the sock's best grasp is out of reach: the best one of its mask within reach
                grasp = self._grasp_in_reach(obs, sock, zone.workspace_mm)
                if grasp is None:
                    continue  # out of reach
                sock = replace(sock, grasp=grasp)
                p = cal.to_arm(obs, grasp)
            u, v = sock.grasp.px.u, sock.grasp.px.v
            off = math.hypot((u - w / 2) / (w / 2), (v - h / 2) / (h / 2)) / math.sqrt(2)
            cloth = self._mask_points(obs, sock.mask, step=4)
            t = Target(
                point=p,
                yaw=self._arm_yaw(obs, sock),
                color=sock.color,
                confidence=sock.confidence,
                cut=sock.touches_roi_edge,
                off_center=off,
                cloth_xy=np.zeros((0, 2)) if cloth is None else cloth[2][:2].T,
            )
            if self._same_sock(t, self.r.given_up) is None:  # not a sock left where it lies
                out.append(t)
        return out

    @staticmethod
    def _mask_points(
        obs: Observation, mask: np.ndarray | None, step: int = 1
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """The pixels of `mask` with depth (every `step`-th row and column): (u, v, points),
        points 4xN in arm coordinates (mm). None without a mask, a camera pose or depth."""
        if mask is None or obs.T_base_cam is None:
            return None
        depth = obs.frame.depth_mm
        sub = np.zeros_like(mask, dtype=bool)
        sub[::step, ::step] = mask[::step, ::step]
        v, u = np.nonzero(sub & (depth > 0))
        if not len(u):
            return None
        k = obs.frame.intrinsics
        z = depth[v, u].astype(np.float64)
        cam = np.stack([(u - k.cx) / k.fx * z, (v - k.cy) / k.fy * z, z, np.ones_like(z)])
        return u, v, obs.T_base_cam @ cam

    def _grasp_in_reach(
        self, obs: Observation, sock: Sock, workspace: Sequence[tuple[float, float]]
    ) -> GraspPoint | None:
        """The mask pixel deepest inside the sock (at least `grasp_inset_px` in, or half its
        width) whose floor point lies in `workspace`; None if none does."""
        if sock.mask is None:
            return None
        inset = self.s.cfg.color_classifier.grasp_inset_px
        dist = cv2.distanceTransform(sock.mask.astype(np.uint8), cv2.DIST_L2, 5)
        pts = self._mask_points(obs, dist >= min(inset, 0.5 * float(dist.max())))
        if pts is None:
            return None
        u, v, p = pts
        z = obs.frame.depth_mm[v, u].astype(np.float64)
        inside = np.array([in_polygon(x, y, workspace) for x, y in zip(p[0], p[1], strict=True)])
        if not inside.any():
            return None
        i = int(np.argmax(np.where(inside, dist[v, u], -1)))
        return GraspPoint(PixelPoint(int(u[i]), int(v[i])), float(z[i]))

    def _arm_yaw(self, obs: Observation, sock: Sock) -> float | None:
        """The detector's yaw (image frame) as the gripper's yaw in the arm frame: two pixels
        along it, both at the grasp's depth, into arm coordinates."""
        a = sock.grasp_angle_rad
        if a is None:
            return None
        g = sock.grasp
        d = 20.0
        p0 = self.s.calibration.to_arm(obs, g)
        px = PixelPoint(int(round(g.px.u + d * math.cos(a))), int(round(g.px.v + d * math.sin(a))))
        p1 = self.s.calibration.to_arm(obs, GraspPoint(px, g.depth_mm))
        return math.atan2(p1.y - p0.y, p1.x - p0.x)

    def box_view(self, obs: Observation) -> BoxView:
        """The cargo box in `obs` (from `look_cargo`): the surface's height above the box floor
        per pixel inside the box (a margin in from its walls), and the socks in it."""
        cargo = self.s.cfg.sim.layout.cargo  # the rover layout the rig is computed from
        assert obs.T_base_cam is not None
        k = obs.frame.intrinsics
        z = obs.frame.depth_mm.astype(np.float64)
        v, u = np.nonzero(z > 0)
        zz = z[v, u]
        cam = np.stack([(u - k.cx) / k.fx * zz, (v - k.cy) / k.fy * zz, zz, np.ones_like(zz)])
        p = obs.T_base_cam @ cam
        x0, x1, y0, y1 = cargo.bounds(-self.cfg.cargo_margin_mm)
        inside = (p[0] > x0) & (p[0] < x1) & (p[1] > y0) & (p[1] < y1)
        height = np.full(z.shape, np.nan)
        height[v[inside], u[inside]] = p[2][inside] - cargo.floor_z_mm
        footprint = (z / k.fx) ** 2  # mm² a pixel covers at that depth
        n = 0
        for sock in self.s.floor_detector.detect(obs.frame).socks:
            q = self.s.calibration.to_arm(obs, sock.grasp)
            n += cargo.contains(q.x, q.y) and q.z < cargo.rim_z_mm
        return BoxView(height, footprint, n)
