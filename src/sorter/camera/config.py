"""Config models for block 1 (camera, views). Placeholder from block 0: block 1 fills it in."""

from pydantic import BaseModel, ConfigDict


class CameraConfig(BaseModel):
    """`camera`: device type, serial, resolution, fps, exposure / white balance."""

    model_config = ConfigDict(extra="allow")


class ViewConfig(BaseModel):
    """`views.<zone>`: the zone as seen from its look pose."""

    model_config = ConfigDict(extra="allow")

    roi: list[tuple[int, int]] = []  # pixel polygon, excluding the gripper fingers
