"""The ChArUco board for the hand-eye calibration: its definition, detection, and a printable
image (also the texture of the board in the physics simulator).

Print `python -m sorter.calibration.board board.png` at 100% scale: 7 x 5 squares of 45 mm
(315 x 225 mm), 32 mm ArUco markers (6x6), and lay it flat in front of the arm under `look_bg`
(`calibration.board_z_mm`: its top above the table).
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from sorter.core.types import Frame, Pose

CACHE = Path(__file__).resolve().parents[3] / "data" / "cache"


@dataclass(frozen=True)
class BoardSpec:
    squares: tuple[int, int] = (7, 5)
    square_mm: float = 45.0
    marker_mm: float = 32.0
    dictionary: int = cv2.aruco.DICT_6X6_250
    px_per_mm: int = 10  # of the rendered image

    @property
    def size_m(self) -> tuple[float, float]:
        return self.squares[0] * self.square_mm / 1000, self.squares[1] * self.square_mm / 1000

    def board(self) -> cv2.aruco.CharucoBoard:
        d = cv2.aruco.getPredefinedDictionary(self.dictionary)
        return cv2.aruco.CharucoBoard(self.squares, self.square_mm, self.marker_mm, d)


BOARD = BoardSpec()


def board_image(spec: BoardSpec = BOARD) -> np.ndarray:
    w = round(spec.squares[0] * spec.square_mm * spec.px_per_mm)
    h = round(spec.squares[1] * spec.square_mm * spec.px_per_mm)
    return spec.board().generateImage((w, h), marginSize=0)


def board_texture(spec: BoardSpec = BOARD) -> Path:
    """A PNG of the board for the simulator's texture (cached)."""
    (w, h), d = spec.squares, spec.dictionary
    name = f"{w}x{h}_{spec.square_mm:g}_{spec.marker_mm:g}_{d}"
    path = CACHE / f"charuco_{name}.png"
    if not path.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), board_image(spec))
    return path


@dataclass(frozen=True)
class Detection:
    T_cam_board: Pose  # mm; board origin at its first corner, z into the board
    obj: np.ndarray  # (N, 3) the corners found, board frame, mm
    img: np.ndarray  # (N, 2) where they are in the image, px
    K: np.ndarray  # camera matrix
    dist: np.ndarray  # distortion coefficients


def detect(frame: Frame, spec: BoardSpec = BOARD, min_corners: int = 6) -> Detection | None:
    """The board's pose (PnP) and its corners in the frame, or None if the board isn't seen
    well enough. Six corners are enough for a flat board: the 45 mm squares are big for the
    camera's view from the look poses (~5 x 4 squares)."""
    board = spec.board()
    detector = cv2.aruco.CharucoDetector(board)
    gray = cv2.cvtColor(frame.color, cv2.COLOR_BGR2GRAY)
    corners, ids, _, _ = detector.detectBoard(gray)
    if ids is None or len(ids) < min_corners:
        return None
    obj, img = board.matchImagePoints(corners, ids)
    k = frame.intrinsics
    K = np.array([[k.fx, 0, k.cx], [0, k.fy, k.cy], [0, 0, 1]])
    dist = np.array(k.coeffs or [0.0] * 5, dtype=float)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist)
    if not ok:
        return None
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return Detection(T, obj.reshape(-1, 3).astype(float), img.reshape(-1, 2).astype(float), K, dist)


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "charuco_board.png"
    cv2.imwrite(out, board_image())
    print(
        f"wrote {out}: print at {BOARD.px_per_mm * 25.4:.0f} dpi for {BOARD.square_mm:g} mm squares"
    )
