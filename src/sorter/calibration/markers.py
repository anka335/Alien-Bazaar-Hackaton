"""ArUco markers for the calibration: detection, and a printable sheet.

    uv run python -m sorter.calibration.markers [--out data/markers.png] [--size-mm 40]

Print at 100 % scale. Each marker's center is what the fingertips touch.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def detector(dict_name: str = "DICT_4X4_50") -> cv2.aruco.ArucoDetector:
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    return cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())


def detect(color: np.ndarray, dict_name: str = "DICT_4X4_50") -> dict[int, tuple[float, float]]:
    """Marker id → pixel center."""
    corners, ids, _ = detector(dict_name).detectMarkers(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return {}
    return {
        int(i): tuple(float(x) for x in c.reshape(4, 2).mean(axis=0))
        for i, c in zip(ids.ravel(), corners, strict=True)
    }


def sheet(ids: list[int], size_mm: float, dpi: int = 300, dict_name: str = "DICT_4X4_50"):
    """A4 page (portrait) with the markers in a 2-column grid, a cross at each center."""
    px_mm = dpi / 25.4
    W, H = int(210 * px_mm), int(297 * px_mm)
    page = np.full((H, W), 255, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    s = int(size_mm * px_mm)
    cols, gap = 2, int(20 * px_mm)
    for k, mid in enumerate(ids):
        r, c = divmod(k, cols)
        x0 = int(W / 2 - s - gap / 2 + c * (s + gap))
        y0 = int(25 * px_mm + r * (s + gap + 10 * px_mm))
        if y0 + s > H:
            break
        page[y0 : y0 + s, x0 : x0 + s] = cv2.aruco.generateImageMarker(d, mid, s)
        cv2.putText(
            page, f"id {mid}", (x0, y0 + s + int(6 * px_mm)), cv2.FONT_HERSHEY_SIMPLEX, 2, 0, 3
        )
    return page


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="sorter.calibration.markers")
    p.add_argument("--out", type=Path, default=Path("data/markers.png"))
    p.add_argument("--size-mm", type=float, default=40.0)
    p.add_argument("--ids", type=int, nargs="+", default=list(range(6)))
    args = p.parse_args(argv)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.out), sheet(args.ids, args.size_mm))
    print(f"{args.out}: print on A4 at 100 % scale, cut the markers apart")


if __name__ == "__main__":
    main()
