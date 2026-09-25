"""Config models for block 3 (box detector)."""

from pydantic import BaseModel, ConfigDict


class BoxDetectorConfig(BaseModel):
    """`box_detector`: cloth masks from SAM3 (D-015), grasp point selection, empty threshold."""

    model_config = ConfigDict(extra="forbid")

    avoid_radius_px: int = 30  # skip candidates this close to a failed grasp (also the simulator)
    min_area_px: int = 800  # smaller instances are noise
    empty_coverage: float = 0.02  # cloth covering less of the ROI → EMPTY
    wall_margin_px: int = 40  # grasp points stay this far inside the ROI (walls, camera)
    min_inset_px: int = 12  # and at least this far inside the cloth
