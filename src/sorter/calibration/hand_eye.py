"""Hand-eye calibration, eye-in-hand (D-006): find T_flange_cam from views of a ChArUco board.

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
    """(T_base_flange, T_cam_board) for every view where the board was found."""
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


def solve(pairs: list[tuple[Pose, Pose]]) -> tuple[Pose, float]:
    """T_flange_cam (mm) and the spread of the board position through the views (mm)."""
    if len(pairs) < 4:
        raise SorterError(f"only {len(pairs)} views of the board; need at least 4")
    R_g = [ee[:3, :3] for ee, _ in pairs]
    t_g = [ee[:3, 3] for ee, _ in pairs]
    R_t = [cb[:3, :3] for _, cb in pairs]
    t_t = [cb[:3, 3] for _, cb in pairs]
    R, t = cv2.calibrateHandEye(R_g, t_g, R_t, t_t, method=cv2.CALIB_HAND_EYE_PARK)
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, t.ravel()
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
    print(f"T_flange_cam (mm):\n{np.round(X, 3)}\nboard spread {rmse:.2f} mm")
    if args.sim:
        from sorter.sim.physics.backend import hand_eye

        err = np.linalg.norm(X[:3, 3] - hand_eye(cfg)[:3, 3])
        print(f"sim: {err:.2f} mm from the true camera mount")
    out = args.out or (
        Path("data/hand_eye_sim.yaml") if args.sim else Path(args.config_dir) / "hand_eye.yaml"
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    result = {
        "T_flange_cam": np.round(X, 4).tolist(),
        "rmse_mm": round(rmse, 3),
        "method": "charuco + cv2.calibrateHandEye(PARK)",
        "camera_serial": getattr(system.camera, "serial", "") or ("sim" if args.sim else ""),
        "created": dt.datetime.now().isoformat(timespec="seconds"),
    }
    out.write_text(yaml.safe_dump({"calibration": {"hand_eye": result}}, sort_keys=False))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
