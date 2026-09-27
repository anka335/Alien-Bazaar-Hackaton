"""The load loop (stage A): socks from the floor all around the rover into the cargo box (D-035).

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
    attempts: int = 0


@dataclass
class Spot:
    """Where a sock was left after `load.max_attempts`: socks seen there are ignored."""

    x: float
    y: float


@dataclass
class _Run:
    scan_k: int = 0  # the scan pose to look from next
    empty_views: int = 0  # views in a row without a sock
    target: Target | None = None
    tries: dict[tuple[int, int], int] = field(default_factory=dict)  # attempts per 50 mm cell
    given_up: list[Spot] = field(default_factory=list)
    cargo_cloth: float | None = None  # cloth in the box (mm³ above its floor), last check
    cargo_socks: int = 0  # socks the camera tells apart in the box, last check


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
        if r.cargo_cloth is None:  # the box's contents at the start: what a drop adds to
            box = self.s.observer.observe(Zone.CARGO)
            r.cargo_cloth, r.cargo_socks = self.cargo_contents(box)
            log.info(
                "cargo box at the start: %.0f cm³ of cloth, %d sock(s)",
                r.cargo_cloth / 1000,
                r.cargo_socks,
            )
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
            t.attempts = r.tries.get(self._cell(t), 0)
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
        near = [
            u
            for u in self._targets(obs, floor)
            if math.hypot(u.point.x - t.point.x, u.point.y - t.point.y) < self.cfg.same_sock_mm
        ]
        if not near:
            return sm.decide(floor, floor.overlay, "the sock isn't there any more", Phase.SCAN)
        best = min(near, key=lambda u: u.off_center)
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
        try:
            result = self.s.arm.pick(t.point, Zone.FLOOR, t.yaw)
        except TargetRejected as e:
            log.warning("pick at (%.0f, %.0f) rejected: %s", t.point.x, t.point.y, e)
            return self._failed(t, "rejected")
        if result.likely_empty:
            log.warning("gripper empty after the pick at (%.0f, %.0f)", t.point.x, t.point.y)
            self.s.arm.home()
            return self._failed(t, "missed")
        return Phase.DROP_TO_CARGO

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
            left = [
                u
                for u in self._targets(obs, floor)
                if math.hypot(u.point.x - t.point.x, u.point.y - t.point.y) < self.cfg.same_sock_mm
            ]
            if left:
                sm.obs = obs
                sm.decide(floor, floor.overlay, "still on the floor", Phase.SCAN)
                return self._failed(t, "still on the floor")
        box = self.s.observer.observe(Zone.CARGO)
        sm.obs = box
        cloth, n_socks = self.cargo_contents(box)
        gained = cloth - (r.cargo_cloth or 0.0)
        more = n_socks - r.cargo_socks
        r.cargo_cloth, r.cargo_socks = cloth, n_socks
        # a sock on top of others adds little volume, one that slid into a gap no new mask:
        # either sign will do
        if gained >= self.cfg.min_sock_volume_mm3 or more > 0:
            sm.counters[t.color] += 1
            sm.failures = 0
            r.tries.pop(self._cell(t), None)
            summary = f"verified: {t.color} in the box ({gained / 1000:+.0f} cm³, {more:+d} sock)"
        else:
            # it left the spot but didn't reach the box: on the rover or elsewhere on the floor,
            # where a later look finds it
            log.warning("the sock left the floor but isn't seen in the box (%+.0f mm³)", gained)
            sm.failures += 1
            summary = f"not in the box ({gained / 1000:+.0f} cm³)"
        r.target = None
        return sm.decide(None, _box_overlay(summary), summary, Phase.SCAN)

    def _failed(self, t: Target, why: str) -> Phase:
        r, sm = self.r, self.sm
        sm.failures += 1
        cell = self._cell(t)
        r.tries[cell] = r.tries.get(cell, 0) + 1
        if r.tries[cell] >= self.cfg.max_attempts:
            log.warning(
                "sock at (%.0f, %.0f) left after %d attempts (%s)",
                t.point.x,
                t.point.y,
                r.tries[cell],
                why,
            )
            r.given_up.append(Spot(t.point.x, t.point.y))
            sm.failures = 0  # given up: not a streak of failures any more
        r.target = None
        return Phase.SCAN

    # --- helpers ---

    def _cell(self, t: Target) -> tuple[int, int]:
        return (round(t.point.x / 50), round(t.point.y / 50))

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
            same = self.cfg.same_sock_mm
            if any(math.hypot(p.x - g.x, p.y - g.y) < same for g in self.r.given_up):
                continue
            u, v = sock.grasp.px.u, sock.grasp.px.v
            off = math.hypot((u - w / 2) / (w / 2), (v - h / 2) / (h / 2)) / math.sqrt(2)
            out.append(
                Target(
                    point=p,
                    yaw=self._arm_yaw(obs, sock),
                    color=sock.color,
                    confidence=sock.confidence,
                    cut=sock.touches_roi_edge,
                    off_center=off,
                )
            )
        return out

    def _grasp_in_reach(
        self, obs: Observation, sock: Sock, workspace: Sequence[tuple[float, float]]
    ) -> GraspPoint | None:
        """The mask pixel deepest inside the sock (at least `grasp_inset_px` in, or half its
        width) whose floor point lies in `workspace`; None if none does."""
        if sock.mask is None or obs.T_base_cam is None:
            return None
        inset = self.s.cfg.color_classifier.grasp_inset_px
        dist = cv2.distanceTransform(sock.mask.astype(np.uint8), cv2.DIST_L2, 5)
        depth = obs.frame.depth_mm
        v, u = np.nonzero((dist >= min(inset, 0.5 * float(dist.max()))) & (depth > 0))
        if not len(u):
            return None
        k = obs.frame.intrinsics
        z = depth[v, u].astype(np.float64)
        cam = np.stack([(u - k.cx) / k.fx * z, (v - k.cy) / k.fy * z, z, np.ones_like(z)])
        p = obs.T_base_cam @ cam
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

    def cargo_contents(self, obs: Observation) -> tuple[float, int]:
        """What the camera sees in the cargo box: the cloth's volume (mm³) and the socks the
        floor detector tells apart there."""
        cargo = self.s.cfg.sim.layout.cargo
        n = 0
        for sock in self.s.floor_detector.detect(obs.frame).socks:
            p = self.s.calibration.to_arm(obs, sock.grasp)
            n += cargo.contains(p.x, p.y) and p.z < cargo.rim_z_mm
        return self.cargo_cloth(obs), n

    def cargo_cloth(self, obs: Observation) -> float:
        """Cloth in the cargo box, mm³: every pixel whose 3D point lies inside the box (a margin
        in from the walls) counts with its height above the box floor times its footprint."""
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
        hgt = np.clip(p[2][inside] - cargo.floor_z_mm, 0, cargo.wall_mm)
        hgt[hgt < self.cfg.cloth_min_mm] = 0
        footprint = (zz[inside] / k.fx) ** 2  # mm² a pixel covers at that depth
        return float((hgt * footprint).sum())


def _box_overlay(summary: str) -> Overlay:
    return Overlay(text=[summary])
