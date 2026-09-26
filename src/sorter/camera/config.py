"""Config models for block 1 (camera, views)."""

from pydantic import BaseModel, ConfigDict


class CameraConfig(BaseModel):
    """`camera`: the RealSense D435i on the wrist."""

    model_config = ConfigDict(extra="forbid")

    serial: str = ""  # empty: the first RealSense found
    width: int = 640  # color and depth (depth is aligned to color)
    height: int = 480
    fps: int = 30
    warmup_frames: int = 30  # auto exposure settles, then it is locked
    lock_exposure: bool = True  # lock auto exposure and white balance after warm-up
    exposure_us: float | None = None  # a fixed exposure instead of the settled one
    white_balance_k: float | None = None  # a fixed white balance instead of the settled one
    timeout_s: float = 2.0  # no frame for this long: CameraError


class ViewConfig(BaseModel):
    """`views.<zone>`: the zone as seen from its look pose."""

    model_config = ConfigDict(extra="allow")

    roi: list[tuple[int, int]] = []  # pixel polygon, excluding the gripper fingers
