"""Save one snapshot per OpenCV camera index, to find the wrist camera's `camera.index`.

uv run python -m sorter.camera.probe [--out data/probe] [--max-index 4]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="sorter.camera.probe")
    p.add_argument("--out", type=Path, default=Path("data/probe"))
    p.add_argument("--max-index", type=int, default=4)
    args = p.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    api = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY
    for i in range(args.max_index):
        cap = cv2.VideoCapture(i, api)
        if not cap.isOpened():
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        img = None
        t_end = time.monotonic() + 1.5  # let auto exposure settle
        while time.monotonic() < t_end:
            ok, f = cap.read()
            img = f if ok else img
        cap.release()
        if img is None:
            print(f"{i}: opened, no frame")
            continue
        path = args.out / f"camera_{i}.jpg"
        cv2.imwrite(str(path), img)
        print(f"{i}: {img.shape[1]}x{img.shape[0]} → {path}")
    print(
        "The wrist camera shows the gripper fingers at the bottom. Put its index in camera.index."
    )


if __name__ == "__main__":
    main()
