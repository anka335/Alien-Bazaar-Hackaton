"""The ChArUco board for the hand-eye calibration: its definition, detection, and a printable
image (also the texture of the board in the physics simulator).

Print `python -m sorter.calibration.board board.png` at 100% scale: 7 x 5 squares of 30 mm
(210 x 150 mm), and lay it on the mat, squares' x axis along the arm's +x.
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
    square_mm: float = 30.0
    marker_mm: float = 22.0
    dictionary: int = cv2.aruco.DICT_4X4_50
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
    path = CACHE / f"charuco_{spec.squares[0]}x{spec.squares[1]}_{spec.square_mm:g}.png"
    if not path.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), board_image(spec))
    return path


def detect(frame: Frame, spec: BoardSpec = BOARD, min_corners: int = 8) -> tuple[Pose, int] | None:
    """T_cam_board (mm; board origin at its first corner, z out of the board) and the number
    of corners used, or None if the board isn't seen well enough."""
    board = spec.board()
    detector = cv2.aruco.CharucoDetector(board)
    gray = cv2.cvtColor(frame.color, cv2.COLOR_BGR2GRAY)
    corners, ids, _, _ = detector.detectBoard(gray)
    if ids is None or len(ids) < min_corners:
        return None
    obj, img = board.matchImagePoints(corners, ids)
    k = frame.intrinsics
    K = np.array([[k.fx, 0, k.cx], [0, k.fy, k.cy], [0, 0, 1]])
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, np.array(k.coeffs or [0.0] * 5))
    if not ok:
        return None
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T, len(ids)


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "charuco_board.png"
    cv2.imwrite(out, board_image())
    print(
        f"wrote {out}: print at {BOARD.px_per_mm * 25.4:.0f} dpi for {BOARD.square_mm:g} mm squares"
    )
