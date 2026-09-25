"""Config models for block 3 (box detector). Placeholder from block 0: block 3 fills it in."""

from pydantic import BaseModel, ConfigDict


class BoxDetectorConfig(BaseModel):
    """`box_detector`: thresholds, `avoid_radius_px`, wall margin."""

    model_config = ConfigDict(extra="allow")

    avoid_radius_px: int = 30  # also used by the simulator
