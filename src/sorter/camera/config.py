"""Config models for block 1 (camera, views)."""

from pydantic import BaseModel, ConfigDict, Field


class CameraConfig(BaseModel):
    """`camera`: the SO-101 wrist camera, a UVC webcam read with OpenCV (D-014). RGB only."""

    model_config = ConfigDict(extra="forbid")

    index: int = 0  # OpenCV device index; `python -m sorter.camera.probe` shows which is which
    name: str = "HD Camera"  # macOS: refuse to open unless a camera with this name is present
    width: int = 1280
    height: int = 720
    fps: float = 30.0
    hfov_deg: float = 70.0  # horizontal field of view, for the approximate intrinsics
    fresh_skip_frames: int = Field(2, ge=0)  # fresh(): drop this many frames after the call
    warmup_s: float = 1.0  # auto exposure settles before start() returns
    reconnect_s: float = 1.0  # wait between reopen attempts after the device is lost


class ViewConfig(BaseModel):
    """`views.<zone>`: the zone as seen from its look pose."""

    model_config = ConfigDict(extra="allow")

    roi: list[tuple[int, int]] = []  # pixel polygon, excluding the gripper fingers
