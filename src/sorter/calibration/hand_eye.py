"""Hand-eye calibration, eye-in-hand (D-006): find T_link5_cam from views of a ChArUco board.

    uv run python -m sorter.calibration.hand_eye            # the real rig → config/hand_eye.yaml
    uv run python -m sorter.calibration.hand_eye --sim      # the physics simulator (a check)

Lay the printed board (`python -m sorter.calibration.board`) flat under `look_bg`. The tool
moves the arm from `look_bg` through `calibration.poses` views, each shifted and tilted a little,
detects the board in each, and solves AX = XB (Park's method). The views are only a little
tilted (the arm can't tilt the camera much over the mat), which leaves Park's translation off by
~1-2 cm; so that is only the start of a fit of every corner's reprojection over the mount and the
board's pose, with the board flat at its measured height `calibration.board_z_mm`. `rmse_mm` is
the spread of the board position computed through each view: small means consistent.
"""

from __future__ import annotations

import argparse
import datetime as dt
import logging
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from sorter.arm import kinematics as kin
from sorter.calibration.board import Detection, detect
from sorter.calibration.marks import mount_change, plausible
from sorter.core.config import DEFAULT_CONFIG_DIR, Config, load_config
from sorter.core.errors import SorterError
from sorter.core.types import Pose, Zone
from sorter.sim.world import camera_mount

log = logging.getLogger(__name__)


def view_poses(cfg: Config, rng: np.random.Generator) -> list[np.ndarray]:
    """Joint targets around `look_bg`: the TCP shifted, the gripper tilted. Not turned about its
    axis: joint 6 doesn't turn the camera (D-027)."""
    c = cfg.calibration
    q_look = np.asarray(cfg.poses["look_bg"], dtype=float)
    tcp = kin.fk_tcp(q_look)[:3, 3]
    out = []
    for _ in range(c.poses * 4):
        if len(out) == c.poses:
            break
        tilt = math.radians(rng.uniform(0.3, 1.0) * c.tilt_deg)
        az = rng.uniform(0, 2 * math.pi)
        approach = (math.sin(tilt) * math.cos(az), math.sin(tilt) * math.sin(az), -math.cos(tilt))
        target = tcp + [*rng.uniform(-c.shift_mm, c.shift_mm, 2), rng.uniform(-10, 20)]
        q = kin.solve(target, approach, q_look, cfg.arm.z_min_mm)
        if q is not None:
            out.append(q)
    return out


def collect(system, cfg: Config, seed: int = 0) -> list[tuple[Pose, Detection]]:
    """(T_base_link5, the board's detection) for every view where the board was found."""
    arm = system.arm
    arm.start()
    arm.look(Zone.BACKGROUND)
    pairs = []
    for i, q in enumerate(view_poses(cfg, np.random.default_rng(seed))):
        try:
            arm.driver.execute(
                kin.plan_joints(arm.driver.joints(), q, z_min_mm=cfg.arm.z_min_mm),
                cfg.arm.speed_scale,
            )
        except SorterError as e:
            log.warning("view %d skipped: %s", i, e)
            continue
        frame = system.camera.fresh()
        found = detect(frame)
        if found is None:
            log.warning("view %d: board not found", i)
            continue
        pairs.append((arm.ee_pose(), found))
        log.info("view %d: board found (%d corners)", i, len(found.obj))
    arm.home()
    return pairs


def park(pairs: list[tuple[Pose, Pose]]) -> Pose:
    """AX = XB by Park & Martin, over every pair of views. A = G_i⁻¹ G_j (link5 motion),
    B = C_i C_j⁻¹ (camera motion), since G_i X C_i is the same board for every i.
    (OpenCV 5 dropped `cv2.calibrateHandEye` from its Python package.)"""
    A, B = [], []
    for i in range(len(pairs)):
        for j in range(i + 1, len(pairs)):
            (g_i, c_i), (g_j, c_j) = pairs[i], pairs[j]
            A.append(np.linalg.inv(g_i) @ g_j)
            B.append(c_i @ np.linalg.inv(c_j))
    M = np.zeros((3, 3))
    for a, b in zip(A, B, strict=True):
        alpha = cv2.Rodrigues(a[:3, :3])[0].ravel()
        beta = cv2.Rodrigues(b[:3, :3])[0].ravel()
        M += np.outer(beta, alpha)
    w, V = np.linalg.eigh(M.T @ M)
    R = V @ np.diag(w**-0.5) @ V.T @ M.T
    C = np.vstack([a[:3, :3] - np.eye(3) for a in A])
    d = np.concatenate([R @ b[:3, 3] - a[:3, 3] for a, b in zip(A, B, strict=True)])
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, np.linalg.lstsq(C, d, rcond=None)[0]
    return X


def solve(pairs: list[tuple[Pose, Pose]]) -> tuple[Pose, float]:
    """T_link5_cam (mm) and the spread of the board position through the views (mm)."""
    if len(pairs) < 4:
        raise SorterError(f"only {len(pairs)} views of the board; need at least 4")
    X = park(pairs)
    return X, spread(pairs, X)


def spread(pairs: list[tuple[Pose, Pose]], X: Pose) -> float:
    """RMS spread of the board position computed through each view with mount X (mm)."""
    board = np.array([(ee @ X @ cb)[:3, 3] for ee, cb in pairs])
    return float(np.sqrt(((board - board.mean(axis=0)) ** 2).sum(axis=1).mean()))


def _pose(r: np.ndarray, t: np.ndarray) -> Pose:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(np.asarray(r, dtype=float))[0], t
    return T


def _flat(x: float, y: float, yaw: float, z: float, down: bool) -> Pose:
    """A board lying flat at height z, its x axis at `yaw`; `down`: its z axis points down."""
    c, s = math.cos(yaw), math.sin(yaw)
    B = np.eye(4)
    B[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ np.diag([1, -1, -1] if down else 1)
    B[:3, 3] = x, y, z
    return B


def _lm(f, p: np.ndarray, iters: int = 100) -> np.ndarray:
    """Levenberg-Marquardt on residuals f(p), central-difference Jacobian (no scipy)."""
    r = f(p)
    cost, lam = r @ r, 1e-3
    for _ in range(iters):
        J = np.empty((r.size, p.size))
        for j in range(p.size):
            dp = np.zeros_like(p)
            dp[j] = 1e-5 * max(1.0, abs(p[j]))
            J[:, j] = (f(p + dp) - f(p - dp)) / (2 * dp[j])
        A, g = J.T @ J, J.T @ r
        while True:
            step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            r_new = f(p + step)
            if (c_new := r_new @ r_new) < cost:
                break
            lam *= 5
            if lam > 1e10:
                return p
        p, r, lam = p + step, r_new, max(lam / 3, 1e-9)
        done = cost - c_new < 1e-10 * cost
        cost = c_new
        if done:
            break
    return p


def refine(
    pairs: list[tuple[Pose, Detection]], X0: Pose, board_z_mm: float
) -> tuple[Pose, Pose, float]:
    """T_link5_cam, T_base_board and the corners' reprojection RMSE (px): every corner of every
    view projected through the mount and a board lying flat at `board_z_mm`, from `X0`."""
    boards = [ee @ X0 @ d.T_cam_board for ee, d in pairs]
    down = float(np.mean([b[2, 2] for b in boards])) < 0
    x_axis = np.mean([b[:3, 0] for b in boards], axis=0)
    xy = np.mean([b[:2, 3] for b in boards], axis=0)
    p0 = np.concatenate(
        [cv2.Rodrigues(X0[:3, :3])[0].ravel(), X0[:3, 3], [*xy, math.atan2(x_axis[1], x_axis[0])]]
    )

    def residuals(p: np.ndarray) -> np.ndarray:
        X, B = _pose(p[:3], p[3:6]), _flat(*p[6:9], board_z_mm, down)
        out = []
        for ee, d in pairs:
            T = np.linalg.inv(ee @ X) @ B
            uv, _ = cv2.projectPoints(d.obj, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], d.K, d.dist)
            out.append((uv.reshape(-1, 2) - d.img).ravel())
        return np.concatenate(out)

    p = _lm(residuals, p0)
    r = residuals(p).reshape(-1, 2)
    rmse = float(np.sqrt((r**2).sum(axis=1).mean()))
    return _pose(p[:3], p[3:6]), _flat(*p[6:9], board_z_mm, down), rmse


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="python -m sorter.calibration.hand_eye", description=__doc__)
    p.add_argument("--sim", action="store_true", help="on the physics simulator, with a board")
    p.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR)
    p.add_argument(
        "--out", type=Path, help="default: config/hand_eye.yaml (sim: data/hand_eye_sim.yaml)"
    )
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)-7s %(message)s")

    from sorter.app import build_system

    over = {"sim": {"engine": "physics", "board": True, "items": []}} if args.sim else {}
    cfg = load_config(args.config_dir, overrides=over)
    system = build_system(cfg, sim=args.sim)
    system.camera.start()
    try:
        views = collect(system, cfg)
        pairs = [(ee, d.T_cam_board) for ee, d in views]
        X_park, rmse_park = solve(pairs)
        X, B, rmse_px = refine(views, X_park, cfg.calibration.board_z_mm)
        rmse = spread(pairs, X)
    finally:
        system.arm.shutdown()
        system.camera.close()
        if hasattr(system.world, "stop"):
            system.world.stop()
    z_park = np.mean([(ee @ X_park @ cb)[2, 3] for ee, cb in pairs])
    moved, turned = mount_change(X_park, X)
    print(f"{len(views)} views; Park: spread {rmse_park:.2f} mm, board top at z {z_park:.1f} mm")
    print(
        f"refined (board flat at z {cfg.calibration.board_z_mm:g} mm): {moved:.1f} mm, "
        f"{turned:.2f}° from Park; reprojection {rmse_px:.2f} px, spread {rmse:.2f} mm"
    )
    print(f"board's first corner at ({B[0, 3]:.0f}, {B[1, 3]:.0f}) mm")
    print(f"T_link5_cam (mm):\n{np.round(X, 3)}")
    if not plausible(camera_mount(cfg.sim), X):
        log.warning("the result is far from the nominal camera mount: check the board and views")
    if rmse_px > 2.0:
        log.warning("reprojection %.1f px is high: board height or size, intrinsics?", rmse_px)
    if args.sim:
        from sorter.sim.physics.backend import hand_eye

        err, err_deg = mount_change(hand_eye(cfg), X)
        err_park, _ = mount_change(hand_eye(cfg), X_park)
        print(f"sim: {err:.2f} mm, {err_deg:.2f}° from the true mount (Park: {err_park:.1f} mm)")
    out = args.out or (
        Path("data/hand_eye_sim.yaml") if args.sim else Path(args.config_dir) / "hand_eye.yaml"
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    result = {
        "T_link5_cam": np.round(X, 4).tolist(),
        "rmse_mm": round(rmse, 3),
        "method": f"charuco ({len(views)} views, {rmse_px:.2f} px) + Park-Martin, refined",
        "camera_serial": getattr(system.camera, "serial", "") or ("sim" if args.sim else ""),
        "created": dt.datetime.now().isoformat(timespec="seconds"),
    }
    out.write_text(yaml.safe_dump({"calibration": {"hand_eye": result}}, sort_keys=False))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
