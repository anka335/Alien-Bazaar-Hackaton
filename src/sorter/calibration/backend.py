"""Real backend of block 2: pinhole camera + hand-eye result from `config/hand_eye.yaml`."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from sorter.calibration.calibration import HandEyeCalibration
from sorter.core.errors import CalibrationError

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config) -> HandEyeCalibration:
    he = cfg.calibration.hand_eye
    if he is None:
        raise CalibrationError(
            "no hand-eye result (config/hand_eye.yaml): run python -m sorter.calibration.hand_eye"
        )
    T = np.array(he.T_flange_cam, dtype=np.float64)
    if T.shape != (4, 4):
        raise CalibrationError("calibration.hand_eye.T_flange_cam must be 4x4")
    return HandEyeCalibration(T)
