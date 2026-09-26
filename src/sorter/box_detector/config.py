"""Config model for block 3 (box detector)."""

from pydantic import BaseModel, ConfigDict


class BoxDetectorConfig(BaseModel):
    """`box_detector`: depth thresholds, margins, `avoid_radius_px`."""

    model_config = ConfigDict(extra="forbid")

    avoid_radius_px: int = 30  # skip candidates this close to a failed grasp (also the simulator)
    wall_margin_px: int = 60  # the grasp stays this far inside the box ROI (fingers + camera)
    floor_percentile: float = 98.0  # the box floor depth: this percentile of the ROI depth
    floor_depth_mm: float | None = None  # or a fixed box floor depth at the look pose
    cloth_height_mm: float = 8.0  # higher above the floor counts as cloth
    min_cloth_px: int = 1500  # less cloth than this: the box is empty
    inset_px: int = 10  # the grasp stays this far inside a cloth region, off its edges
    smooth_px: float = 4.0  # blur of the height map before picking its top
    depth_window_px: int = 5  # window for the robust depth of the grasp point
