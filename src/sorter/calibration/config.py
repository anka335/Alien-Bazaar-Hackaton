"""Config models for block 2 (calibration): one plane homography per zone (D-014)."""

from datetime import datetime

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import Zone


class ZoneCalibration(BaseModel):
    """Pixel (at the zone's look pose) ↔ arm XY on the zone's plane. `config/calibration.yaml`."""

    model_config = ConfigDict(extra="forbid")

    H: list[list[float]]  # 3×3, pixel (u, v, 1) → arm (x, y, 1) in mm
    plane_z_mm: float  # height of the plane the markers lay on (box floor / mat), arm frame
    surface_offset_mm: float = 0.0  # to_arm returns plane_z + this (e.g. typical pile height)
    rmse_mm: float = 0.0
    n_points: int = 0
    created: datetime | None = None


class CalibrationConfig(BaseModel):
    """`calibration`."""

    model_config = ConfigDict(extra="forbid")

    zones: dict[Zone, ZoneCalibration] = Field(default_factory=dict)
    marker_dict: str = "DICT_4X4_50"  # ArUco dictionary of the printed calibration markers
