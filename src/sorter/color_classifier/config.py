"""Config models for block 4 (color classifier). Placeholder from block 0: block 4 fills it in."""

from pydantic import BaseModel, ConfigDict


class ColorClassifierConfig(BaseModel):
    """`color_classifier`: class thresholds, erosion, min / max item area."""

    model_config = ConfigDict(extra="allow")
