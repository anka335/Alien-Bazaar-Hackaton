"""Config models for block 2 (calibration)."""

from pydantic import BaseModel, ConfigDict, Field


class HandEyeResult(BaseModel):
    """`calibration.hand_eye`, written by `python -m sorter.calibration.hand_eye` into
    `config/hand_eye.yaml` (committed, D-007)."""

    model_config = ConfigDict(extra="forbid")

    T_link5_cam: list[list[float]]  # 4x4, mm; camera = optical frame (x right, y down, z forward)
    rmse_mm: float | None = None  # spread of the board position over the calibration poses
    method: str = ""
    camera_serial: str = ""
    created: str = ""


class CalibrationConfig(BaseModel):
    """`calibration`: the hand-eye result and the hand-eye tool's settings."""

    model_config = ConfigDict(extra="forbid")

    hand_eye: HandEyeResult | None = None
    poses: int = Field(default=14, ge=4)  # views of the board the tool collects
    tilt_deg: float = 12.0  # how far the tool tilts the camera between views
    shift_mm: float = 30.0  # how far it moves the camera sideways
    board_z_mm: float = 16.0  # the board tool: the printed board's top above the table (a check)
    marks_z_mm: float = 1.0  # the /calibrate page: the mat top, where the tape marks lie
