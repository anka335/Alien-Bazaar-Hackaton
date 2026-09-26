"""The camera calibration page (`/calibrate`, setup mode), without a printed board (D-021).

1. **Marks.** The arm points its tip at spots around the mat center (`calibration.marks`), a
   few mm above the mat; a small dark tape mark goes right under the tip.
2. **Camera mount.** From a few views, each mark is clicked in the live image; click + depth give
   the camera-frame point, and the fit of all clicks gives T_link5_cam. Saved to
   `config/hand_eye.yaml` and used at once (the 3D view, the overlay).
3. **Look poses.** `look_box` / `look_bg` recomputed for that mount: the camera straight over
   the zone, as low as it can still see the whole zone with depth. Saved to `config/rig.yaml`
   with the zone ROIs.

The overlay (the marks and the zone outlines projected into the live image from where the arm
is) shows at every step whether the camera, the mount and the tape agree. Motions go through
`ManualControl`, one at a time, like on the manual page.
"""

from __future__ import annotations

import dataclasses
import datetime as dt
import json
import logging
import re
import threading
from pathlib import Path
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np
import yaml

from sorter.arm import kinematics as kin
from sorter.arm.controller import in_polygon
from sorter.calibration.marks import (
    Click,
    MountFit,
    deproject,
    find_squares,
    fit_mount,
    hand_eye_result,
    identify,
    marks,
    mirrored,
    mount_change,
    plausible,
    project,
    snap,
    view_rmse,
)
from sorter.core.errors import TargetRejected
from sorter.core.types import Intrinsics, Pose, Zone
from sorter.dashboard.manual import Busy, ManualControl, write_pose
from sorter.sim.layout import axis_hit, camera_over
from sorter.sim.world import camera_mount

if TYPE_CHECKING:
    from sorter.core.config import Config
    from sorter.core.protocols import Calibration, Camera
    from sorter.sim.config import RectConfig

log = logging.getLogger(__name__)

HOVER_MM = 5.0  # the tip stops this far above a mark
MIN_DEPTH_MM = 200.0  # the D435i has no depth closer than ~18 cm at 640x480
LOOK_TCP_Z_MM = (60, 200)  # look poses: the TCP heights tried, low to high, 10 mm steps
# views for the clicks: (dx, dy) of the camera center from the marks' center (mm). No wrist turn:
# the camera is on link5 and joint 6 doesn't turn it (D-022)
VIEWS = ((0.0, 0.0), (35.0, 25.0), (-35.0, -25.0))
# measured joints of one pose wander by a few mrad while the arm holds it: the same pose (view)
SAME_POSE_RAD = 0.01
VIEW_TCP_Z_MM = (160, 80)  # the highest TCP height tried for a view, then lower
LOOKS = {
    "look_box": Zone.BOX,
    "look_bg": Zone.BACKGROUND,
}


def _same(a, b) -> bool:
    """Joints `a` and `b` are one pose (view)."""
    return bool(np.allclose(a, b, atol=SAME_POSE_RAD))


def _corners(r: RectConfig, z: float, margin: float = 0.0) -> list[tuple[float, float, float]]:
    (cx, cy), (w, h) = r.center_mm, r.size_mm
    x0, x1 = cx - w / 2 + margin, cx + w / 2 - margin
    y0, y1 = cy - h / 2 + margin, cy + h / 2 - margin
    return [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]


def coverage(px: list[tuple[float, float] | None], width: int, height: int) -> float:
    """Share of the (convex) pixel polygon inside the image; 0 if a corner is behind the camera."""
    if any(p is None for p in px):
        return 0.0
    poly = np.array(px, dtype=np.float32)
    area = abs(cv2.contourArea(poly))
    if area < 1.0:
        return 0.0
    img = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32)
    if cv2.contourArea(poly, oriented=True) < 0:
        poly = poly[::-1].copy()
    inside, _ = cv2.intersectConvexConvex(poly, img)
    return float(min(inside / area, 1.0))


def hidden_rows(depth_mm: np.ndarray) -> int:
    """Rows at the top of the image hidden by the gripper (no depth: closer than the minimum)."""
    no_depth = (depth_mm == 0).mean(axis=1) > 0.3
    return int(np.argmin(no_depth)) + 10 if no_depth[0] else 0


def write_views(rig: Path, rois: dict[str, list[list[int]]]) -> None:
    """Replace `views.<zone>.roi` in rig.yaml, keeping the rest of the file."""
    text = rig.read_text() if rig.is_file() else ""
    views = (yaml.safe_load(text) or {}).get("views") or {}
    for zone, roi in rois.items():
        views.setdefault(zone, {})["roi"] = roi
    block = yaml.safe_dump({"views": views}, sort_keys=False, default_flow_style=None, width=100)
    m = re.search(r"^views:\n(?:[ -].*\n)*", text, re.MULTILINE)
    if m is None:
        text = text + ("" if text.endswith("\n") or not text else "\n") + block
    else:
        text = text[: m.start()] + block + text[m.end() :]
    rig.write_text(text)


class CalibrateControl:
    def __init__(
        self,
        manual: ManualControl,
        camera: Camera,
        calibration: Calibration,
        cfg: Config,
        hand_eye_file: Path,
        true_mount: Pose | None = None,
        fixed_marks: bool = False,
        marks_file: Path | None = None,
        dump_dir: Path | None = None,
    ):
        """`true_mount`: the sim's camera mount, to report the fit's error. `fixed_marks`: the
        marks lie at their nominal spots already (the sim draws them), not where the tip stops.
        `marks_file`: where the tip stopped over each mark, kept across restarts. `dump_dir`:
        every frame clicked or detected in and the clicks go to a folder in it, to debug a fit."""
        self.manual, self.arm, self.camera = manual, manual.arm, camera
        self.calibration, self.cfg = calibration, cfg
        self.hand_eye_file = Path(hand_eye_file)
        self.true_mount = None if true_mount is None else np.asarray(true_mount, dtype=float)
        lay = cfg.sim.layout
        ms = marks(lay.background.center_mm, cfg.calibration.marks_z_mm)
        self.marks = {m.name: m for m in ms}
        self.fixed_marks = fixed_marks
        # where the tip really stopped over each mark (FK of the measured joints): the arm sags
        # a few mm at full reach, and the tape goes under the real tip
        self._placed: dict[str, tuple[float, float, float]] = {}
        self.marks_file = None if fixed_marks or marks_file is None else Path(marks_file)
        if self.marks_file is not None and self.marks_file.is_file():
            saved = (yaml.safe_load(self.marks_file.read_text()) or {}).get("marks") or {}
            self._placed = {n: tuple(v) for n, v in saved.items() if n in self.marks}
        he = cfg.calibration.hand_eye
        self.method = he.method if he is not None else "sim"
        self._lock = threading.Lock()
        self._clicks: list[tuple[Click, tuple[float, ...]]] = []  # and the joints at the click
        self._fit: MountFit | None = None
        self._fixed: list[int] = []  # views whose mirrored labels the last fit fixed (1-based)
        self._detected: dict[str, Any] | None = None  # the last detection: squares, marks
        # the mount a fit is checked against: the nominal one, not a (maybe wrong) saved one
        self.prior = camera_mount(cfg.sim)
        self._saved: str | None = None
        self._look: dict[str, dict[str, Any]] = {}
        self._look_saved: str | None = None
        self._dump = (
            None
            if dump_dir is None
            else Path(dump_dir) / dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        )

    # --- the mount ---

    @property
    def mount(self) -> Pose:
        """T_link5_cam in use: what the 3D view and the pipeline use now."""
        return np.asarray(self.calibration.cam_pose(np.eye(4)), dtype=float)

    def _plausible(self, T: Pose) -> bool:
        """Near the nominal mount (any turn about the optical axis): not a wrong fit or file."""
        return plausible(self.prior, T)

    def _preview(self) -> Pose:
        """The fitted mount if there is one, else the one in use: what the overlay shows."""
        with self._lock:
            return self._fit.T_link5_cam if self._fit is not None else self.mount

    def _intrinsics(self) -> Intrinsics | None:
        frame = self.camera.latest()
        return frame.intrinsics if frame is not None else getattr(self.camera, "intrinsics", None)

    def _mark(self, name: str) -> tuple[float, float, float]:
        """Where mark `name` is, mm, arm frame."""
        if name not in self.marks:
            raise ValueError(f"unknown mark {name!r}")
        return self._placed.get(name, self.marks[name].xyz)

    def _check_idle(self) -> None:
        if self.manual.busy:
            raise Busy("busy: wait until the arm stops")

    # --- step 1: marks ---

    def goto_mark(self, name: str) -> None:
        """The tip `HOVER_MM` above the mark, gripper closed: the tape goes right under it."""
        if name not in self.marks:
            raise ValueError(f"unknown mark {name!r}")
        x, y, z = self.marks[name].xyz
        ws = self.arm.zones[Zone.BACKGROUND].workspace_mm
        if not in_polygon(x, y, ws):
            raise TargetRejected(f"mark {name} ({x:.0f}, {y:.0f}) is outside the mat workspace")
        # from above, as high as the arm can still hold the gripper vertical there
        zmin = self.cfg.arm.z_min_mm
        above = next(
            (
                h
                for h in (self.cfg.arm.safe_z_mm, 80.0, 60.0, 40.0)
                if kin.solve((x, y, z + h), "down", None, zmin) is not None
            ),
            None,
        )
        if above is None or kin.solve((x, y, z + HOVER_MM), "down", None, zmin) is None:
            raise TargetRejected(f"the tip can't reach mark {name} pointing down")
        above += z

        def move() -> None:
            self.arm.lift()
            self.arm.set_gripper(0.0)
            self.arm.move_tcp((x, y, above))
            self.arm.move_tcp((x, y, z + HOVER_MM), linear=True)
            if not self.fixed_marks:
                tip = kin.fk_tcp(self.arm.joints())[:3, 3]
                xyz = (round(float(tip[0]), 1), round(float(tip[1]), 1), z)
                with self._lock:
                    self._placed[name] = xyz
                    placed = {n: list(v) for n, v in self._placed.items()}
                    # clicks of a re-placed mark refer to where it is now
                    self._clicks = [
                        (dataclasses.replace(c, p_base=xyz) if c.mark == name else c, q)
                        for c, q in self._clicks
                    ]
                    self._refit()
                if self.marks_file is not None:
                    self.marks_file.parent.mkdir(parents=True, exist_ok=True)
                    self.marks_file.write_text(yaml.safe_dump({"marks": placed}))

        self.manual.run(f"to mark {name}", move)

    # --- step 2: the camera mount ---

    def view_pose(self, i: int) -> np.ndarray:
        """Joints of view `i` over the marks."""
        if not 0 <= i < len(VIEWS):
            raise ValueError(f"view {i} out of range 0..{len(VIEWS) - 1}")
        dx, dy = VIEWS[i]
        pts = np.array([self._mark(n) for n in self.marks])
        cx, cy, z = pts[:, 0].mean() + dx, pts[:, 1].mean() + dy, pts[0, 2]
        # aimed with the saved mount, else the nominal one (not with a fit in progress: a fit
        # from one view can be off enough to lose the marks); as close as the arm can
        X = self.prior if self.method == "nominal" else self.mount
        best = None  # (how far off the marks' center, joints)
        for tcp_z in range(VIEW_TCP_Z_MM[0], VIEW_TCP_Z_MM[1] - 1, -10):
            q = camera_over(self.cfg.arm, (cx, cy), tcp_z, X, z, exact=False)
            if q is None:
                continue
            hit = axis_hit(kin.fk_link5(q) @ X, z)
            off = np.inf if hit is None else float(np.hypot(hit[0] - cx, hit[1] - cy))
            if off < 2.0:
                return q  # the highest centered one
            if best is None or off < best[0]:
                best = (off, q)
        if best is not None:
            return best[1]
        raise TargetRejected(f"no pose puts the camera over the marks for view {i + 1}")

    def goto_view(self, i: int) -> None:
        q = self.view_pose(i)

        def move() -> None:
            self.arm.lift()
            self.arm.move_joints(q)
            self._detect()

        self.manual.run(f"view {i + 1}", move)

    def detect(self) -> list[str]:
        """Find and name the marks in the image from where the arm is now (see `_detect`)."""
        self._check_idle()
        return self._detect()

    def _detect(self) -> list[str]:
        """Find the tape squares in a fresh frame, name them from the distances between them
        (`identify`), and make them the clicks of this pose. Returns the names found."""
        frame = self.camera.fresh()
        self._dump_frame(frame, "detect")
        pts = [(u, v, deproject(frame, u, v)) for u, v in find_squares(frame)]
        pts = [(u, v, p) for u, v, p in pts if p is not None]
        positions = {n: self._mark(n) for n in self.marks}
        found = identify(np.array([p for _, _, p in pts]), positions) if len(pts) >= 3 else None
        q, F = self.arm.joints(), self.arm.ee_pose()
        names = [] if found is None else sorted(found[0].values())
        with self._lock:
            self._detected = {"squares": len(pts), "marks": names}
            if found is None:
                return []
            self._clicks = [(c, cq) for c, cq in self._clicks if not _same(cq, q)]
            for i, name in sorted(found[0].items(), key=lambda t: t[1]):
                u, v, p = pts[i]
                self._clicks.append((Click(name, positions[name], F, p, (u, v)), q))
            self._refit()
        log.info("found marks %s (%d dark squares)", names, len(pts))
        return names

    def click(self, name: str, u: float, v: float) -> None:
        """Mark `name` is at pixel (u, v) of the live image, seen from where the arm is now."""
        xyz = self._mark(name)
        self._check_idle()
        frame = self.camera.fresh()
        k = frame.intrinsics
        if not (0 <= u < k.width and 0 <= v < k.height):
            raise ValueError(f"({u:.0f}, {v:.0f}) is outside the image")
        self._dump_frame(frame, f"click-{name}")
        snapped = snap(frame.color, u, v)
        if snapped is not None:
            u, v = snapped
        p = deproject(frame, u, v)
        if p is None:
            raise ValueError(
                "no depth at the click: the camera is too close (the D435i needs ~18 cm) "
                "or the mark is on an edge; raise the arm or click the mark center"
            )
        q = self.arm.joints()
        c = Click(name, xyz, self.arm.ee_pose(), p, (u, v))
        with self._lock:
            # one click per mark and pose: a second one corrects the first
            self._clicks = [
                (o, oq) for o, oq in self._clicks if not (o.mark == name and _same(oq, q))
            ]
            self._clicks.append((c, q))
            self._refit()
        log.info("mark %s clicked at (%.0f, %.0f), %.0f mm deep", name, u, v, p[2])

    def delete_click(self, i: int) -> None:
        with self._lock:
            if not 0 <= i < len(self._clicks):
                raise ValueError(f"no click {i}")
            del self._clicks[i]
            self._refit()

    def clear_clicks(self) -> None:
        with self._lock:
            self._clicks = []
            self._fit = None
            self._fixed = []

    def _views(self) -> list[int]:
        """The view (distinct pose, in click order) of every click."""
        poses: list[tuple[float, ...]] = []
        out = []
        for _, q in self._clicks:
            i = next((j for j, p in enumerate(poses) if _same(q, p)), None)
            if i is None:
                poses.append(q)
                i = len(poses) - 1
            out.append(i)
        return out

    def _refit(self) -> None:
        """Fit the clicks; a view clicked mirrored (M2↔M3, M4↔M5) gets its labels fixed."""
        views = self._views()
        positions = {n: self._mark(n) for n in self.marks}
        clicks = [c for c, _ in self._clicks]
        self._fit, flipped = fit_mount(clicks, views, positions)
        if flipped:
            self._clicks = [
                (mirrored(c, positions) if v in flipped else c, q)
                for (c, q), v in zip(self._clicks, views, strict=True)
            ]
            self._fixed = [v + 1 for v in flipped]
            log.warning("marks of view(s) %s were mirrored (M2/M3, M4/M5): fixed", self._fixed)
        if self._fit is not None:
            per_view = ", ".join(
                f"view {v['view']}: {'-' if v['rmse_mm'] is None else v['rmse_mm']} mm"
                for v in self._view_fits()
            )
            log.info(
                "fit rmse %.1f mm; per view, without the arm FK: %s", self._fit.rmse_mm, per_view
            )
        self._dump_clicks()

    def _view_fits(self) -> list[dict[str, Any]]:
        """Per view: its marks and `view_rmse` (the lock held)."""
        views = self._views()
        out = []
        for v in sorted(set(views)):
            cs = [c for (c, _), cv in zip(self._clicks, views, strict=True) if cv == v]
            r = view_rmse(cs)
            out.append(
                {
                    "view": v + 1,
                    "marks": sorted({c.mark for c in cs}),
                    "rmse_mm": None if r is None else round(r, 1),
                }
            )
        return out

    def _dump_frame(self, frame, what: str) -> None:
        if self._dump is None:
            return
        try:
            self._dump.mkdir(parents=True, exist_ok=True)
            stem = self._dump / f"{dt.datetime.now().strftime('%H%M%S-%f')}-{what}"
            cv2.imwrite(str(stem.with_suffix(".png")), frame.color)
            np.save(stem.with_suffix(".npy"), frame.depth_mm)
            meta = {
                "joints": list(self.arm.joints()),
                "intrinsics": dataclasses.asdict(frame.intrinsics),
            }
            stem.with_suffix(".json").write_text(json.dumps(meta))
        except Exception as e:  # debugging aid only: never break a click
            log.warning("calibrate dump failed: %s", e)

    def _dump_clicks(self) -> None:
        """The clicks and the fit as they are now (the lock held)."""
        if self._dump is None:
            return
        fit = self._fit
        data = {
            "clicks": [
                {
                    "mark": c.mark,
                    "p_base": list(c.p_base),
                    "p_cam": list(c.p_cam),
                    "px": list(c.px),
                    "joints": list(q),
                    "T_base_link5": np.asarray(c.T_base_link5).tolist(),
                }
                for c, q in self._clicks
            ],
            "placed": {n: list(v) for n, v in self._placed.items()},
            "fit": None
            if fit is None
            else {
                "T_link5_cam": np.asarray(fit.T_link5_cam).tolist(),
                "rmse_mm": fit.rmse_mm,
                "residuals_mm": fit.residuals_mm,
            },
            "views": self._view_fits(),
        }
        try:
            self._dump.mkdir(parents=True, exist_ok=True)
            (self._dump / "clicks.json").write_text(json.dumps(data, indent=1))
        except Exception as e:
            log.warning("calibrate dump failed: %s", e)

    def save_mount(self) -> str:
        """Write the fit to hand_eye.yaml and use it from now on. Returns the file."""
        with self._lock:
            fit = self._fit
            n = len({c.mark for c, _ in self._clicks})
        if fit is None:
            raise ValueError(f"click at least 3 marks (not in a line) first; have {n}")
        if not self._plausible(fit.T_link5_cam):
            dist, _ = mount_change(self.prior, fit.T_link5_cam)
            raise ValueError(
                f"the fit puts the camera {dist:.0f} mm from the nominal mount or looking away "
                "from the gripper: a click is on the wrong mark; delete the biggest errors"
            )
        serial = getattr(self.camera, "serial", "") or ""
        result = hand_eye_result(fit, n, serial)
        self.hand_eye_file.parent.mkdir(parents=True, exist_ok=True)
        text = yaml.safe_dump({"calibration": {"hand_eye": result}}, sort_keys=False)
        self.hand_eye_file.write_text(text)
        if hasattr(self.calibration, "T_link5_cam"):
            self.calibration.T_link5_cam = np.array(fit.T_link5_cam)
        self.method = result["method"]
        self._saved = str(self.hand_eye_file)
        with self._lock:
            self._look = {}  # computed for the old mount
        log.info("camera mount saved to %s (rmse %.1f mm)", self.hand_eye_file, fit.rmse_mm)
        return self._saved

    # --- step 3: look poses ---

    def _zone(self, zone: Zone) -> tuple[RectConfig, float, float]:
        """The zone's rectangle, its surface height, the ROI margin (mm)."""
        lay = self.cfg.sim.layout
        if zone is Zone.BOX:
            return lay.box, lay.box.floor_z_mm, 0.0
        return lay.background, self.cfg.calibration.marks_z_mm, 5.0

    def compute_look(self) -> dict[str, dict[str, Any]]:
        """`look_box` / `look_bg` for the mount in use: over the zone center (or as close as the
        arm can put the camera), the lowest TCP height at which the camera sees the whole zone
        with depth, else the best one. Heights above the highest reachable one aren't tried: the
        IK search runs in the arm's process and a long one starves its control loop."""
        k = self._intrinsics()
        if k is None:
            raise ValueError("no camera frame yet")
        X = self.mount
        out = {}
        for name, zone in LOOKS.items():
            rect, surface, margin = self._zone(zone)
            corners = _corners(rect, surface, margin)
            best = None
            for tcp_z in range(LOOK_TCP_Z_MM[0], LOOK_TCP_Z_MM[1] + 1, 10):
                q = camera_over(self.cfg.arm, rect.center_mm, tcp_z, X, surface, exact=False)
                if q is None:
                    if best is not None:
                        break  # too high from here on
                    continue
                T_cam = kin.fk_link5(q) @ X
                px = project(T_cam, k, corners)
                cand = {
                    "q": [round(float(v), 4) for v in q],
                    "tcp_z": tcp_z,
                    "camera_mm": round(float(T_cam[2, 3] - surface)),
                    "coverage": round(coverage(px, k.width, k.height), 3),
                    "px": px,
                    "top": 0,
                }
                cand["depth_ok"] = cand["camera_mm"] >= MIN_DEPTH_MM
                cand["fits"] = cand["depth_ok"] and cand["coverage"] >= 0.999
                if cand["fits"]:
                    best = cand
                    break
                if best is None or (cand["depth_ok"], cand["coverage"]) > (
                    best["depth_ok"],
                    best["coverage"],
                ):
                    best = cand
                elif best["depth_ok"] and cand["coverage"] < best["coverage"] - 0.02:
                    break  # past the best: higher, the arm pulls the camera off the zone
            if best is None:
                raise TargetRejected(f"no pose puts the camera straight over the {zone} zone")
            h = best["camera_mm"]
            best["sees_mm"] = [round(k.width * h / k.fx), round(k.height * h / k.fy)]
            out[name] = best
        with self._lock:
            self._look = out
            self._look_saved = None
        return out

    def goto_look(self, name: str) -> None:
        """Preview a computed look pose; the rows the gripper hides are measured there."""
        with self._lock:
            cand = self._look.get(name)
        if cand is None:
            raise ValueError(f"compute the look poses first ({name})")

        def move() -> None:
            self.arm.lift()
            self.arm.move_joints(cand["q"])
            cand["top"] = hidden_rows(self.camera.fresh().depth_mm)
            if self._look_saved is not None:  # saved already: the ROIs now skip the gripper
                self._write_look()

        self.manual.run(f"preview {name}", move)

    def _roi(self, cand: dict[str, Any], k: Intrinsics) -> list[list[int]]:
        return [
            [
                int(np.clip(round(u), 0, k.width - 1)),
                int(np.clip(round(v), cand["top"], k.height - 1)),
            ]
            for u, v in cand["px"]
        ]

    def save_look(self) -> list[str]:
        """Write the computed look poses and the zone ROIs to rig.yaml; use them at once."""
        self._check_idle()
        rig_file = self.manual.rig_file
        with self._lock:
            look = dict(self._look)
        k = self._intrinsics()
        if not look or k is None:
            raise ValueError("compute the look poses first")
        lines = self._write_look()
        for name, c in look.items():
            self.arm.set_pose(name, c["q"])
        with self._lock:
            self._look_saved = str(rig_file)
        log.info("look poses and views saved to %s", rig_file)
        return lines

    def _write_look(self) -> list[str]:
        rig_file = self.manual.rig_file
        with self._lock:
            look = dict(self._look)
        k = self._intrinsics()
        lines = [write_pose(rig_file, name, c["q"]) for name, c in look.items()]
        write_views(rig_file, {LOOKS[n].value: self._roi(c, k) for n, c in look.items()})
        return lines

    # --- all at once: the page's Calibrate button ---

    def calibrate(self) -> dict[str, Any]:
        """Save the fitted mount, recompute the look poses for it, save them with the ROIs."""
        self._check_idle()
        file = self.save_mount()
        self.compute_look()
        lines = self.save_look()
        return {"file": file, "lines": lines}

    # --- status ---

    def _overlay(self, k: Intrinsics | None) -> dict[str, Any]:
        if k is None:
            return {"marks": [], "zones": {}}
        T_cam = self.arm.ee_pose() @ self._preview()
        names = list(self.marks)
        px = project(T_cam, k, [self._mark(n) for n in names])
        zones = {}
        for zone in Zone:
            rect, surface, _ = self._zone(zone)
            corners = project(T_cam, k, _corners(rect, surface))
            if all(p is not None for p in corners):
                zones[zone.value] = [[round(u, 1), round(v, 1)] for u, v in corners]
        return {
            "marks": [
                {"name": n, "px": None if p is None else [round(p[0], 1), round(p[1], 1)]}
                for n, p in zip(names, px, strict=True)
            ],
            "zones": zones,
        }

    def state(self) -> dict[str, Any]:
        k = self._intrinsics()
        with self._lock:
            clicks = list(self._clicks)
            fit = self._fit
            look = {
                n: {key: v for key, v in c.items() if key not in ("q", "px")}
                for n, c in self._look.items()
            }
            look_saved = self._look_saved
            view_fits = self._view_fits() if fit is not None else []
        q_now = self.arm.joints()
        poses: list[tuple[float, ...]] = []  # the distinct poses clicked from, in order
        for _, q in clicks:
            if not any(_same(q, p) for p in poses):
                poses.append(q)
        fit_json = None
        if fit is not None:
            dist, angle = mount_change(self.mount, fit.T_link5_cam)
            fit_json = {
                "rmse_mm": round(fit.rmse_mm, 2),
                "change_mm": round(dist, 1),
                "change_deg": round(angle, 2),
                "marks": len({c.mark for c, _ in clicks}),
                "fixed_views": self._fixed,
                "views": view_fits,
                "plausible": self._plausible(fit.T_link5_cam),
            }
            if self.true_mount is not None:
                err, err_deg = mount_change(self.true_mount, fit.T_link5_cam)
                fit_json["true_error_mm"] = round(err, 2)
                fit_json["true_error_deg"] = round(err_deg, 2)
        return {
            "arm": self.manual.state(),
            "image": None if k is None else {"width": k.width, "height": k.height},
            "overlay": self._overlay(k),
            "overlay_mount": "fit" if fit is not None else "in use",
            "marks": [
                {
                    "name": n,
                    "xyz": list(self._mark(n)),
                    "placed": self.fixed_marks or n in self._placed,
                }
                for n in self.marks
            ],
            "hover_mm": HOVER_MM,
            "detected": self._detected,
            "views": len(VIEWS),
            "clicks": [
                {
                    "mark": c.mark,
                    "px": [round(c.px[0], 1), round(c.px[1], 1)],
                    "here": bool(_same(q, q_now)),
                    "pose": next(j for j, p in enumerate(poses) if _same(q, p)),
                    "residual_mm": (None if fit is None else round(fit.residuals_mm[i], 1)),
                }
                for i, (c, q) in enumerate(clicks)
            ],
            "fit": fit_json,
            "mount": {"method": self.method, "saved": self._saved},
            "hand_eye_file": str(self.hand_eye_file),
            "look": look or None,
            "look_saved": look_saved,
            "rig_file": str(self.manual.rig_file),
        }
