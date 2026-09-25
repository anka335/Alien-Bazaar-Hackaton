"""Zone setup: calibrate a zone's look view and define where the arm may pick (D-014).

    uv run python -m sorter.calibration.setup <box|background> snap     # look pose → image
    uv run python -m sorter.calibration.setup <box|background>          # markers + corners
    uv run python -m sorter.calibration.setup <box|background> verify   # touch test

Needs the `home` and look poses (`python -m sorter.arm.teach`) and ≥ 4 printed ArUco markers
(`python -m sorter.calibration.markers`) lying flat in the zone. Writes the homography to
`config/calibration.yaml` and the zone's ROI, workspace and floor to `config/rig.yaml`.
"""

from __future__ import annotations

import argparse
import contextlib
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from sorter.arm.backend import create as create_arm
from sorter.arm.config import ZoneConfig
from sorter.arm.controller import So101Arm
from sorter.calibration.markers import detect
from sorter.calibration.plane import apply, fit_homography
from sorter.camera.uvc import UvcCamera
from sorter.core.config import DEFAULT_CONFIG_DIR, Config, load_config, update_yaml
from sorter.core.errors import TargetRejected
from sorter.core.types import Zone

OUT = Path("data/calib")
MARGIN_MM = {Zone.BOX: 35.0, Zone.BACKGROUND: 20.0}  # workspace inside the corners (walls, camera)
FLOOR_MM = {Zone.BOX: 5.0, Zone.BACKGROUND: 2.0}  # fingertips stay this far above the plane
APPROACH_MM = {Zone.BOX: 120.0, Zone.BACKGROUND: 60.0}


def shrink(poly: list[tuple[float, float]], margin: float) -> list[tuple[float, float]]:
    """Offset a convex polygon inward by `margin` (either winding)."""
    P = np.asarray(poly, dtype=float)
    n = len(P)
    area = 0.5 * sum(P[i, 0] * P[(i + 1) % n, 1] - P[(i + 1) % n, 0] * P[i, 1] for i in range(n))
    sign = 1.0 if area > 0 else -1.0
    lines = []
    for i in range(n):
        a, b = P[i], P[(i + 1) % n]
        d = (b - a) / np.linalg.norm(b - a)
        normal = sign * np.array([-d[1], d[0]])  # points inside
        lines.append((a + margin * normal, d))
    out = []
    for i in range(n):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        t = np.linalg.solve(np.c_[d1, -d2], p2 - p1)[0]
        out.append(tuple(float(round(v, 1)) for v in p1 + t * d1))
    return out


class Session:
    def __init__(self, cfg: Config, zone: Zone):
        self.cfg, self.zone = cfg, zone
        self.arm: So101Arm = create_arm(cfg)
        self.camera = UvcCamera(cfg.camera)

    def __enter__(self) -> Session:
        self.camera.start()
        self.arm.start()
        return self

    def __exit__(self, *exc) -> None:
        self.camera.close()
        if self.arm.bus is not None:
            self.arm.bus.close()  # motors stay on, holding

    def look(self) -> np.ndarray:
        self.arm.look(self.zone)
        return self.camera.fresh().color.copy()

    def lift_and_look(self) -> np.ndarray:
        q = self.arm.read_q()
        p = self.arm.tcp(q)
        with contextlib.suppress(TargetRejected):  # no straight lift from here: go home directly
            self.arm.follow(self.arm.plan_line(p, p + [0, 0, 60], q, 60.0), 60.0, strict=False)
        self.arm.goto("home", speed_scale=0.5)
        return self.look()

    def touch(self, what: str) -> np.ndarray | None:
        """Motors off, the person puts the fingertips on the point, motors on. None = skip."""
        ans = input(f"\n{what}\n  Hold the arm, then Enter (motors go off), or 's' to skip: ")
        if ans.strip().lower() == "s":
            return None
        self.arm.set_gripper(0.0)
        self.arm.disable()
        input("  Put the closed fingertips on the point, then Enter: ")
        p = self.arm.tcp(self.arm.read_q())
        self.arm.start()
        print(f"  → ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm")
        return p


def save_image(name: str, img: np.ndarray) -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / name
    cv2.imwrite(str(path), img)
    return path


def draw_markers(img: np.ndarray, markers: dict[int, tuple[float, float]]) -> np.ndarray:
    out = img.copy()
    for mid, (u, v) in markers.items():
        cv2.drawMarker(out, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
        cv2.putText(out, str(mid), (int(u) + 10, int(v) - 10), 0, 1.2, (0, 0, 255), 3)
    return out


def snap(s: Session) -> None:
    img = s.look()
    markers = detect(img, s.cfg.calibration.marker_dict)
    path = save_image(f"{s.zone}_look.jpg", draw_markers(img, markers))
    print(f"{path}: {img.shape[1]}x{img.shape[0]}, markers {sorted(markers)}")


def calibrate(s: Session, config_dir: Path) -> None:
    img = s.look()
    markers = detect(img, s.cfg.calibration.marker_dict)
    save_image(f"{s.zone}_markers.jpg", draw_markers(img, markers))
    print(f"markers seen: {sorted(markers)}")
    if len(markers) < 4:
        raise SystemExit("need ≥ 4 markers in view; move them or re-teach the look pose")

    pixels, points = [], []
    for mid in sorted(markers):
        p = s.touch(f"Marker {mid}: touch its center.")
        if p is not None:
            pixels.append(markers[mid])
            points.append(p)
    if len(points) < 4:
        raise SystemExit("need ≥ 4 touched markers")
    H, rmse = fit_homography(pixels, [tuple(p[:2]) for p in points])
    plane_z = float(np.median([p[2] for p in points]))
    print(f"\nhomography RMSE {rmse:.1f} mm over {len(points)} markers, plane z {plane_z:.1f} mm")

    print("\nNow the zone corners (mat corners / inner box corners at the floor), going around.")
    corners = []
    while True:
        p = s.touch(f"Corner {len(corners) + 1} (or 's' when done, ≥ 3).")
        if p is None:
            if len(corners) >= 3:
                break
            continue
        corners.append((float(p[0]), float(p[1])))

    img = s.lift_and_look()
    roi = apply(np.linalg.inv(H), np.asarray(corners))
    h, w = img.shape[:2]
    roi = [(int(np.clip(u, 0, w - 1)), int(np.clip(v, 0, h - 1))) for u, v in roi]
    preview = img.copy()
    cv2.polylines(preview, [np.asarray(roi, np.int32)], True, (0, 255, 0), 3)
    path = save_image(f"{s.zone}_roi.jpg", preview)

    z = s.zone
    zc = s.cfg.zones.get(z) or ZoneConfig(approach_mm=APPROACH_MM[z])
    workspace = shrink(corners, MARGIN_MM[z])
    update_yaml(
        config_dir / "calibration.yaml",
        {
            "calibration": {
                "zones": {
                    z.value: {
                        "H": [[float(v) for v in row] for row in H],
                        "plane_z_mm": round(plane_z, 1),
                        "rmse_mm": round(rmse, 2),
                        "n_points": len(points),
                        "created": datetime.now().isoformat(timespec="seconds"),
                    }
                }
            }
        },
    )
    update_yaml(
        config_dir / "rig.yaml",
        {
            "views": {z.value: {"roi": [list(p) for p in roi]}},
            "zones": {
                z.value: {
                    "workspace_mm": [list(p) for p in workspace],
                    "z_floor_mm": round(plane_z + FLOOR_MM[z], 1),
                    "grasp_depth_mm": zc.grasp_depth_mm,
                    "approach_mm": zc.approach_mm,
                }
            },
        },
    )
    print(f"\nsaved. ROI preview: {path}. Check that it excludes the gripper fingers;")
    print("edit views.<zone>.roi in config/rig.yaml if not. Then run the touch test (verify).")


def verify(s: Session) -> None:
    """Touch test: the tip goes 15 mm above each marker the camera sees; the person checks."""
    img = s.look()
    markers = detect(img, s.cfg.calibration.marker_dict)
    zc = s.cfg.calibration.zones.get(s.zone)
    if zc is None:
        raise SystemExit(f"{s.zone} not calibrated yet")
    print(f"markers seen: {sorted(markers)}")
    for mid, px in sorted(markers.items()):
        ((x, y),) = apply(np.asarray(zc.H), np.array([px]))
        above = np.array([x, y, zc.plane_z_mm + 80])
        low = np.array([x, y, zc.plane_z_mm + 15])
        if input(f"\nmarker {mid}: tip 15 mm above ({x:.0f}, {y:.0f})? Enter / s: ") == "s":
            continue
        r = s.arm.solve_down(above, s.arm.read_q(), 30.0)
        try:
            if r is None:
                raise TargetRejected("out of reach")
            line = s.arm.plan_line(above, low, r.q, 30.0)
        except TargetRejected as e:
            print(f"  {e}")
            continue
        s.arm.move_joints(r.q, speed_scale=0.5)
        s.arm.follow(line, 65.0)
        input("  Is the tip over the marker center? Note the error, Enter to go on: ")
        s.lift_and_look()


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="sorter.calibration.setup")
    p.add_argument("zone", choices=[z.value for z in Zone])
    p.add_argument("step", nargs="?", default="calibrate", choices=["snap", "calibrate", "verify"])
    p.add_argument("--config-dir", type=Path, default=DEFAULT_CONFIG_DIR)
    args = p.parse_args(argv)
    cfg = load_config(args.config_dir)
    with Session(cfg, Zone(args.zone)) as s:
        if args.step == "snap":
            snap(s)
        elif args.step == "verify":
            verify(s)
        else:
            calibrate(s, args.config_dir)


if __name__ == "__main__":
    main()
