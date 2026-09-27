"""The unload loop (stage B): every sock from the cargo box into the laundry bin of its color.

Only the wrist camera tells the loop where things are (D-036). The rover is
parked at the station, but not exactly where the layout says, and the bins are put down by
hand, so the loop first finds each bin (a look at its layout place, a second look centered on
the estimate if the bin was cut by the image edge). Then, per sock: look into the cargo box,
pick the sock on top of the pile, look into the box again (this look is also the next cycle's),
show the gripper to the camera: the held sock's color from how it hangs, else from the one sock
gone from the box; unknown → back into the box. Then drop into the bin found for that color, and
look into that bin to see it landed: only a seen drop is counted.

    LOOK_CARGO (the bins, once) → SENSE_CARGO → PICK_FROM_CARGO → DROP_TO_LAUNDRY → LOOK_CARGO …
    the box empty × N → DONE
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import numpy as np

from sorter.box_detector.cargo import CargoView, SockSeen, SockTarget, find_sock, taken
from sorter.box_detector.geometry import points
from sorter.box_detector.held import SHOW_POSE, HeldView, find_held, show_pose
from sorter.box_detector.station import BinFit, find_bin
from sorter.core.errors import ArmError, SorterError, TargetRejected
from sorter.core.types import ArmPoint, ColorClass, Observation, Overlay, Phase, Zone
from sorter.orchestrator.state_machine import Loop

log = logging.getLogger(__name__)

FINGER_TRAVEL_MM = 50.0  # each finger from the middle, the gripper all open (rebot_b601)
SHAKE_MM = 12.0  # sideways, twice, right after a pick
MAX_PUT_BACKS = 6  # in a row before each more counts as a failure
RELOCATE_MM = 20.0  # a bin found this far from where the look was centered: look again there
DROP_IN_MM = 20.0  # the TCP this far below the bin's rim to let go (the bin is 140 mm inside)
DROP_ABOVE_MM = 90.0  # over the rim before going down to the drop height (the sock hangs)
LANDED_MM3 = 3000.0  # cloth "volume" (mm·px) a sock adds to a bin, at least
BIN_FLOOR_MM = 5.0  # the bin's inside floor above the floor


@dataclass
class Bin:
    center: tuple[float, float]  # mm, arm frame
    yaw: float
    look: np.ndarray  # joints of the look pose centered on it
    found: bool  # seen (else: the layout's place)
    cloth: float = 0.0  # cloth seen in it at the last look (mm·px)


class UnloadLoop(Loop):
    first = Phase.LOOK_CARGO

    def reset(self) -> None:
        self.avoid: list[ArmPoint] = []
        self.empty_streak = 0
        self.view: CargoView | None = None
        self.target: SockTarget | None = None
        self.held: HeldView | None = None
        self.gone: list[SockSeen] = []  # what the last pick took out of the box, as seen
        self.put_backs = 0  # in a row, since the last drop
        self.color: ColorClass | None = None  # of the sock in the gripper
        self.next_obs: Observation | None = None  # a look into the box still true now
        self.bins: dict[ColorClass, Bin] | None = None
        self._segment = None

    # --- helpers ---

    def segment(self):
        """The segmentation for the socks: the render's in the sim, else the SAM3 service.
        Remembered for the last few frames: a look into the box is used again later."""
        if self._segment is None:
            cam = self.s.camera
            if hasattr(cam, "segment") and not self.s.cfg.sim.use_sam3:
                seg = cam.segment
            else:
                from sorter.color_classifier.segmenter import SamSegmenter

                seg = SamSegmenter(self.s.cfg.color_classifier.sam).segment
            cache: list[tuple[np.ndarray, list]] = []

            def remembered(bgr: np.ndarray) -> list:
                for img, out in cache:
                    if img is bgr:
                        return out
                out = seg(bgr)
                cache.append((bgr, out))
                del cache[:-4]
                return out

            self._segment = remembered
        return self._segment

    def _look_pose(self, center: tuple[float, float]) -> np.ndarray:
        from sorter.sim.layout import look_pose

        cfg = self.s.cfg
        return look_pose(cfg.sim, cfg.arm, center, cfg.sim.layout.floor_z_mm)

    def _show_pose(self) -> np.ndarray:
        """Where the camera sees what the gripper holds: the rig's `show_held`, else found now."""
        cfg = self.s.cfg
        if SHOW_POSE not in cfg.poses:
            q = show_pose(
                np.asarray(cfg.poses["home"]),
                np.asarray(self.s.calibration.cam_pose(np.eye(4))),
                cfg.arm.keep_out_mm,
                cfg.arm.keep_out_margin_mm,
                cfg.arm.z_min_mm,
                cfg.sim.layout.body.bounds()[1],
                cfg.sim.layout.floor_z_mm,
                cfg.sim.focal_px,
                (cfg.sim.width, cfg.sim.height),
            )
            if q is None:
                raise SorterError("no pose shows the camera what the gripper holds")
            cfg.poses[SHOW_POSE] = [float(v) for v in q]
        return np.asarray(cfg.poses[SHOW_POSE])

    def _move(self, q: np.ndarray) -> None:
        """A straight joint move to `q`, via home if that one would hit something."""
        try:
            self.s.arm.move_joints(q)  # type: ignore[attr-defined]
        except ArmError:
            self.s.arm.home()
            self.s.arm.move_joints(q)  # type: ignore[attr-defined]

    def _observe(self, q: np.ndarray, zone: Zone) -> Observation:
        self._move(q)
        return self._observe_here(zone)

    def _observe_here(self, zone: Zone) -> Observation:
        s = self.s
        frame = s.camera.fresh()
        return Observation(frame, zone, s.calibration.cam_pose(s.arm.ee_pose()), s.arm.joints())

    def _publish(self, obs: Observation, result, overlay: Overlay, summary: str) -> None:
        self.sm.obs = obs
        self.sm.decide(result, overlay, summary, self.sm.phase)

    # --- the station ---

    def _find_bin(
        self, color: ColorClass, guess: tuple[float, float]
    ) -> tuple[Bin, BinFit, Observation]:
        """Look at the bin's layout place; if the bin was cut by the image edge or is off the
        middle, look again centered on it. The best of the looks counts: a later one may see
        less (a look pose the arm can't center)."""
        bins = self.s.cfg.sim.layout.laundry
        floor = self.s.cfg.sim.layout.floor_z_mm
        center, look = guess, self._look_pose(guess)
        best: tuple[float, BinFit, Observation, np.ndarray] | None = None
        for _ in range(3):
            obs = self._observe(look, Zone.LAUNDRY)
            fit = find_bin(obs, center, floor, bins.size_mm, bins.height_mm, bins.wall_t_mm)
            self._publish(obs, fit, fit.overlay, f"{color} bin: " + fit.overlay.text[-1])
            if fit.center is None:
                break
            quality = fit.score * fit.seen + (1.0 if fit.complete else 0.0)
            if best is None or quality > best[0]:
                best = (quality, fit, obs, look)
            if fit.complete and math.dist(fit.center, center) < RELOCATE_MM:
                break
            center, look = fit.center, self._look_pose(fit.center)  # look again, centered
        if best is None:
            log.warning("%s bin not found: using its place in the layout", color)
            return Bin(guess, 0.0, self._look_pose(guess), False), fit, obs
        _, fit, obs, look = best
        assert fit.center is not None
        return Bin(fit.center, fit.yaw, look, True), fit, obs

    def _locate_bins(self) -> dict[ColorClass, Bin]:
        out = {}
        for color, guess in self.s.cfg.sim.layout.laundry.centers_mm.items():
            b, _, obs = self._find_bin(color, guess)
            b.cloth = self._cloth_in(obs, b)
            out[color] = b
        self.s.arm.home()
        return out

    def _cloth_in(self, obs: Observation, b: Bin) -> float:
        """Cloth inside bin `b` (the sum of its height over the bin floor, mm·px)."""
        lay = self.s.cfg.sim.layout
        bins = lay.laundry
        p = points(obs)
        c, s = math.cos(-b.yaw), math.sin(-b.yaw)
        dx, dy = p[..., 0] - b.center[0], p[..., 1] - b.center[1]
        u, v = c * dx - s * dy, s * dx + c * dy  # in the bin's own axes
        half = bins.size_mm / 2 - bins.wall_t_mm - 8.0  # off the walls
        h = p[..., 2] - (lay.floor_z_mm + BIN_FLOOR_MM)
        with np.errstate(invalid="ignore"):
            inside = (np.abs(u) < half) & (np.abs(v) < half) & (h > 3) & (h < bins.height_mm + 60)
        return float(np.nansum(np.where(inside, h, 0.0)))

    # --- the phases ---

    def _look_cargo(self) -> Phase:
        if self.bins is None:
            self.bins = self._locate_bins()
        self.sm.new_cycle()
        if self.next_obs is not None:  # the look after the last pick: nothing touched the box since
            self.sm.obs, self.next_obs = self.next_obs, None
        else:
            self.sm.obs = self.s.observer.observe(Zone.CARGO)
        return Phase.SENSE_CARGO

    def _cargo_view(self, obs: Observation) -> CargoView:
        cfg = self.s.cfg
        cargo = cfg.sim.layout.cargo
        return find_sock(
            obs,
            cargo,
            cargo.floor_z_mm,
            cargo.rim_z_mm,
            cfg.zones[Zone.CARGO].workspace_mm,
            cfg.color_classifier,
            self.segment(),
            self.avoid,
            finger_mm=FINGER_TRAVEL_MM * cfg.arm.gripper.open,
        )

    def _sense_cargo(self) -> Phase:
        sm = self.sm
        obs = sm.obs
        assert obs is not None
        view = self._cargo_view(obs)
        self.view, self.target = view, view.target
        if view.target is not None:
            t = view.target
            self.empty_streak = 0
            summary = (
                f"{t.color} sock ({t.confidence:.2f}), grasp at ({t.point.x:.0f}, "
                f"{t.point.y:.0f}), {t.height_mm:.0f} mm up; {view.socks} sock(s) in view"
            )
            if t.confidence < sm.cfg.low_confidence:
                log.warning("low color confidence: %s", summary)
            return sm.decide(view, view.overlay, summary, Phase.PICK_FROM_CARGO)
        if not view.empty:
            if self.avoid:
                log.warning("no grasp left, clearing %d avoided points", len(self.avoid))
                self.avoid.clear()
                summary = "no grasp, clearing avoided"
                return sm.decide(view, view.overlay, summary, Phase.SENSE_CARGO)
            sm.failures += 1
            return sm.decide(view, view.overlay, "cloth, but no grasp point", Phase.LOOK_CARGO)
        self.empty_streak += 1
        n = sm.cfg.empty_confirmations
        nxt = Phase.DONE if self.empty_streak >= n else Phase.LOOK_CARGO
        return sm.decide(view, view.overlay, f"cargo empty ({self.empty_streak}/{n})", nxt)

    def _pick_from_cargo(self) -> Phase:
        sm = self.sm
        t = self.target
        assert sm.obs is not None and t is not None
        target = self.s.calibration.to_arm(sm.obs, t.grasp)
        # from over the box: the look may be an earlier one, taken before a drop
        self.s.arm.look(Zone.CARGO)
        errors = []
        for yaw in t.yaws:  # near a wall only some finger directions fit
            try:
                result = self.s.arm.pick(target, Zone.CARGO, yaw)
                break
            except TargetRejected as e:
                errors.append(str(e))
        else:
            log.warning("grasp at (%.0f, %.0f) rejected: %s", target.x, target.y, errors[0])
            self.avoid.append(target)
            sm.failures += 1
            return Phase.SENSE_CARGO  # same observation
        cargo = self.s.cfg.sim.layout.cargo
        lift = self.s.cfg.zones[Zone.CARGO].lift_z_mm
        # a shake over the box: a neighbor only stuck to a finger falls back in; what is
        # pinched stays
        x, y = target.x, target.y
        for dx in (SHAKE_MM, -SHAKE_MM, 0.0):
            try:
                self.s.arm.move_tcp((x + dx, y, lift), linear=True)  # type: ignore[attr-defined]
            except TargetRejected:
                break
        # out over the middle of the box first: what hangs from the fingers (and a neighbor
        # stuck to them) would drag over the near wall on the way out
        self.s.arm.move_tcp((*cargo.center_mm, lift), linear=True)  # type: ignore[attr-defined]
        # which sock left the box? The box lit from above shows colors best; this look is
        # also the next cycle's
        after_obs = self.s.observer.observe(Zone.CARGO)
        after = self._cargo_view(after_obs)
        gone = taken(self.view, after) if self.view is not None else []
        self.gone = gone
        # is anything in the gripper? Shown to the camera on the way to the bins
        self.s.arm.home()
        obs = self._observe(self._show_pose(), Zone.LAUNDRY)
        cfg = self.s.cfg
        held = find_held(obs, self.segment(), cfg.color_classifier, cfg.sim.layout.floor_z_mm)
        self.held = held
        if held.color is None and not gone and result.likely_empty:
            summary = f"nothing in the gripper (opening {result.gripper_opening:.2f})"
            log.warning(summary)
            self._publish(obs, held, held.overlay, summary)
            self.avoid.append(target)
            sm.failures += 1
            return Phase.LOOK_CARGO
        color, why = self._held_color(held, gone)
        if color is None:
            summary = f"{why}: back into the box"
            log.warning(summary)
            self._publish(obs, held, held.overlay, summary)
            self._put_back()
            self.put_backs += 1
            if self.put_backs > MAX_PUT_BACKS:  # not getting anywhere with this pile
                sm.failures += 1
            return Phase.LOOK_CARGO
        self.color = color
        self.next_obs = after_obs
        summary = f"holding a {color} sock ({why})"
        if color is not t.color:
            log.info("%s; the target was a %s one", summary, t.color)
        self._publish(obs, held, held.overlay, summary)
        return Phase.DROP_TO_LAUNDRY

    @staticmethod
    def _held_color(held: HeldView, gone: list[SockSeen]) -> tuple[ColorClass | None, str]:
        """The held sock's color, and why. First what hangs from the fingers, seen from the
        show pose (side-lit: its own thresholds, `side_class`); socks of different classes
        hanging there → none. Else the one sock gone from the box (lit from above; a pile
        that settles anew once a sock is pulled out of it can hide others, so only one
        counts). Else none: the color is unknown."""
        classes = {c for c in held.classes or [] if c is not None}
        if len(classes) > 1:
            return None, f"{held.count} socks of different colors hang from the fingers"
        if classes:
            return classes.pop(), "seen hanging"
        if len(gone) == 1:
            return gone[0].color, "gone from the box"
        return None, "holding a sock of a color nobody saw"

    def _put_back(self) -> None:
        """Everything in the gripper back into the middle of the cargo box."""
        arm, cfg = self.s.arm, self.s.cfg
        cargo = cfg.sim.layout.cargo
        arm.home()
        arm.move_tcp((*cargo.center_mm, cfg.zones[Zone.CARGO].lift_z_mm))  # type: ignore[attr-defined]
        # into the box, as into a bin: let go over the rim and the sock may land on it
        down = (*cargo.center_mm, cargo.rim_z_mm - DROP_IN_MM)
        arm.move_tcp(down, linear=True)  # type: ignore[attr-defined]
        arm.set_gripper(cfg.arm.gripper.open)  # type: ignore[attr-defined]
        arm.move_tcp((*cargo.center_mm, cfg.zones[Zone.CARGO].lift_z_mm), linear=True)  # type: ignore[attr-defined]
        arm.look(Zone.CARGO)

    def _drop_to_laundry(self) -> Phase:
        sm, arm = self.sm, self.s.arm
        assert self.color is not None and self.bins is not None
        color = self.color
        # the look into the box after the pick stays true only if this drop goes as planned
        box_obs, self.next_obs = self.next_obs, None
        b = self.bins[color]
        lay = self.s.cfg.sim.layout
        rim = lay.floor_z_mm + lay.laundry.height_mm
        x, y = b.center
        arm.home()  # from the show pose: a straight move from there swings the wrist into things
        arm.move_tcp((x, y, rim + DROP_ABOVE_MM))  # type: ignore[attr-defined]
        # the fingers go into the bin: the sock hangs from them off to a side, released over the
        # rim it could land on it
        arm.move_tcp((x, y, rim - DROP_IN_MM), linear=True)  # type: ignore[attr-defined]
        arm.set_gripper(self.s.cfg.arm.gripper.open)  # type: ignore[attr-defined]
        arm.move_tcp((x, y, rim + DROP_ABOVE_MM), linear=True)  # type: ignore[attr-defined]
        self.avoid.clear()
        # did it land? look into the bin: its cloth must have grown
        obs = self._observe(b.look, Zone.LAUNDRY)
        cloth = self._cloth_in(obs, b)
        grew = cloth - b.cloth
        b.cloth = cloth
        self.put_backs = 0
        if grew >= LANDED_MM3:
            sm.counters[color] += 1
            sm.failures = 0
            summary = f"{color} sock landed in its bin (+{grew:.0f})"
        else:
            sm.failures += 1
            summary = f"{color} sock not seen in its bin (+{grew:.0f})"
            log.warning(summary)
        self._publish(obs, None, Overlay(text=[summary]), summary)
        self.next_obs = box_obs
        return Phase.LOOK_CARGO
