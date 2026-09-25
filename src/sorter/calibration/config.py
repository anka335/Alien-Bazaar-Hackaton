"""Config models for block 2 (calibration). Placeholder from block 0: block 2 fills it in."""

from pydantic import BaseModel, ConfigDict


class CalibrationConfig(BaseModel):
    """`calibration`: `hand_eye` (the transform, from config/hand_eye.yaml)."""

    model_config = ConfigDict(extra="allow")
