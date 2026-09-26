"""Config models for block 1 (camera, views)."""

from pydantic import BaseModel, ConfigDict


class CameraConfig(BaseModel):
    """`camera`: the RealSense D435i on the wrist."""

    model_config = ConfigDict(extra="forbid")

    serial: str = ""  # empty: the first RealSense found
    width: int = 640  # color and depth (depth is aligned to color)
    height: int = 480
    fps: int = 30
    warmup_frames: int = 60  # auto exposure and white balance settle, then they are locked
    lock_exposure: bool = True  # lock auto exposure and white balance after warm-up
    exposure: float | None = None  # a fixed exposure (x100 µs) instead of the settled one
    gain: float | None = None  # a fixed gain (0-128) instead of the settled one
    white_balance_k: float | None = None  # a fixed white balance instead of the settled one
    settle_frames: int = 5  # frames after an option change before the probe frame
    lock_steps: int = 8  # bisection steps per locked option
    timeout_s: float = 2.0  # no frame for this long: CameraError
    start_attempts: int = 5  # macOS: the system camera driver sometimes wins the device, so retry


class ViewConfig(BaseModel):
    """`views.<zone>`: the zone as seen from its look pose."""

    model_config = ConfigDict(extra="allow")

    roi: list[tuple[int, int]] = []  # pixel polygon, excluding the gripper fingers
