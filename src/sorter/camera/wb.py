"""Find the white balance for the room: put something white (a white sock, paper) so it fills the
green frame in the middle of the wrist camera's view, in the light the floor gets, and run (as
root on macOS, like the camera)

    sudo .venv/bin/python -m sorter.camera.wb

It steps the color sensor's white balance over its range, measures the middle of the image,
prints the value where it is the most neutral as a `config/local.yaml` line, and saves every
step side by side to `data/wb_sweep.jpg`. The auto white balance goes wrong on the orange
parquet (a white sock came out pink: "colored"), so the camera needs a fixed one.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import cv2
import numpy as np

from sorter.core.config import DEFAULT_CONFIG_DIR, load_config


def neutral_error(bgr: np.ndarray) -> float:
    """Lab chroma of the mean color of the brightest third of `bgr` (the white thing, not a
    hand or a shadow in front of it): 0 for a neutral gray or white."""
    px = bgr.reshape(-1, 3).astype(float)
    luma = px @ (0.114, 0.587, 0.299)
    px = px[luma >= np.percentile(luma, 67)]
    mean = px.mean(axis=0).reshape(1, 1, 3).astype(np.uint8)
    lab = cv2.cvtColor(mean, cv2.COLOR_BGR2LAB)
    _, a, b = lab.reshape(3).astype(float) - (0, 128, 128)
    return float(np.hypot(a, b))


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="python -m sorter.camera.wb", description=__doc__)
    p.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR)
    p.add_argument("--step", type=float, default=300.0, help="K between steps")
    p.add_argument("--out", type=Path, default=Path("data/wb_sweep.jpg"))
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    from sorter.camera.realsense import RealSenseCamera

    cfg = load_config(args.config_dir)
    cam = RealSenseCamera(cfg.camera.model_copy(update={"lock_exposure": False}))
    cam.start()
    try:
        rs = cam.rs
        lo, hi = cam._range(rs.option.white_balance)
        cam._set(rs.option.enable_auto_white_balance, 0)
        results, tiles = [], []
        for k in np.arange(lo, hi + 1, args.step):
            cam._set(rs.option.white_balance, float(k))
            for _ in range(cfg.camera.settle_frames):
                frame = cam.fresh()
            img = frame.color
            h, w = img.shape[:2]
            mid = img[h // 3 : 2 * h // 3, w // 3 : 2 * w // 3]
            err = neutral_error(mid)
            results.append((err, float(k)))
            tile = cv2.resize(img, (w // 3, h // 3))
            cv2.rectangle(tile, (w // 9, h // 9), (2 * w // 9, 2 * h // 9), (0, 255, 0), 1)
            cv2.putText(tile, f"{k:.0f} K  {err:.0f}", (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)  # fmt: skip
            tiles.append(tile)
            print(f"{k:6.0f} K: chroma in the middle {err:5.1f}", flush=True)
    finally:
        cam.close()
    rows = [np.hstack(tiles[i : i + 4] + [np.zeros_like(tiles[0])] * (4 - len(tiles[i : i + 4])))
            for i in range(0, len(tiles), 4)]  # fmt: skip
    args.out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.out), np.vstack(rows))
    err, best = min(results)
    print(f"\n# the middle is the most neutral at {best:.0f} K (chroma {err:.1f}); {args.out}")
    print(f"# into config/local.yaml:\ncamera:\n  white_balance_k: {best:.0f}")


if __name__ == "__main__":
    main()
