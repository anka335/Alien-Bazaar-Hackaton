"""Config model of the floor detector (stage A)."""

from pydantic import BaseModel, ConfigDict


class FloorDetectorConfig(BaseModel):
    """`floor_detector`: which blobs count as socks. Sizes in mm from the depth, not pixels, so
    they hold from any camera height."""

    model_config = ConfigDict(extra="forbid")

    min_area_mm2: float = 2500.0  # smaller: a scrap, a crumb of glare
    max_area_mm2: float = 45000.0  # bigger: a towel, a shirt, the floor itself
    max_length_mm: float = 380.0  # a sock is at most this long lying flat
    edge_px: int = 4  # a mask this close to the frame's edge or to no-depth pixels is cut off
    avoid_radius_px: float = 30.0  # a sock whose grasp is this close to an avoided point is left
    local_axis_mm: float = 60.0  # the sock's direction at the grasp: from its mask this close
