"""Hand-eye calibration, eye-in-hand (D-006): find T_link5_cam from views of a ChArUco board.

    uv run python -m sorter.calibration.hand_eye            # the real rig → config/hand_eye.yaml
    uv run python -m sorter.calibration.hand_eye --sim      # the physics simulator (a check)

Lay the printed board (`python -m sorter.calibration.board`) flat on the mat. The tool moves
the arm from `look_bg` through `calibration.poses` views, each shifted, tilted and turned a
little, detects the board in each, and solves AX = XB (Park's method). `rmse_mm` is the spread
of the board position computed through each view: small means consistent.
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
from sorter.calibration.board import detect
from sorter.core.config import DEFAULT_CONFIG_DIR, Config, load_config
from sorter.core.errors import SorterError
from sorter.core.types import Pose, Zone

log = logging.getLogger(__name__)


def view_poses(cfg: Config, rng: np.random.Generator) -> list[np.ndarray]:
    """Joint targets around `look_bg`: the TCP shifted, the gripper tilted and turned."""
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
        target = tcp + [*rng.uniform(-c.shift_mm, c.shift_mm, 2), rng.uniform(-15, 0)]
        q = kin.solve(target, approach, q_look, cfg.arm.z_min_mm)
        if q is None:
            continue
        q[5] += math.radians(rng.uniform(-20, 20))  # turn about the approach axis
        lo, hi = kin.JOINT_LIMITS[5]
        if lo <= q[5] <= hi:
            out.append(q)
    return out


def collect(system, cfg: Config, seed: int = 0) -> list[tuple[Pose, Pose]]:
    """(T_base_link5, T_cam_board) for every view where the board was found."""
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
        T_cam_board, n = found
        pairs.append((arm.ee_pose(), T_cam_board))
        log.info("view %d: board found (%d corners)", i, n)
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
    board = np.array([(ee @ X @ cb)[:3, 3] for ee, cb in pairs])
    rmse = float(np.sqrt(((board - board.mean(axis=0)) ** 2).sum(axis=1).mean()))
    return X, rmse


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
        X, rmse = solve(collect(system, cfg))
    finally:
        system.arm.shutdown()
        system.camera.close()
        if hasattr(system.world, "stop"):
            system.world.stop()
    print(f"T_link5_cam (mm):\n{np.round(X, 3)}\nboard spread {rmse:.2f} mm")
    if args.sim:
        from sorter.sim.physics.backend import hand_eye

        err = np.linalg.norm(X[:3, 3] - hand_eye(cfg)[:3, 3])
        print(f"sim: {err:.2f} mm from the true camera mount")
    out = args.out or (
        Path("data/hand_eye_sim.yaml") if args.sim else Path(args.config_dir) / "hand_eye.yaml"
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    result = {
        "T_link5_cam": np.round(X, 4).tolist(),
        "rmse_mm": round(rmse, 3),
        "method": "charuco + Park-Martin",
        "camera_serial": getattr(system.camera, "serial", "") or ("sim" if args.sim else ""),
        "created": dt.datetime.now().isoformat(timespec="seconds"),
    }
    out.write_text(yaml.safe_dump({"calibration": {"hand_eye": result}}, sort_keys=False))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
