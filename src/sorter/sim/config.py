"""Config models for the simulator (`sim`, block 0)."""

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import ColorClass, Zone


class SimZoneConfig(BaseModel):
    """The part of the table the camera sees from a zone's look pose (arm frame, mm)."""

    model_config = ConfigDict(extra="forbid")

    center_mm: tuple[float, float]
    width_mm: float  # along the image u axis; the height follows from the image aspect
    surface_z_mm: float  # box floor / background surface


def _default_zones() -> dict[Zone, SimZoneConfig]:
    return {
        Zone.BOX: SimZoneConfig(center_mm=(350, -200), width_mm=300, surface_z_mm=40),
        Zone.BACKGROUND: SimZoneConfig(center_mm=(350, 150), width_mm=300, surface_z_mm=0),
    }


class SimConfig(BaseModel):
    """`sim`: simulator world."""

    model_config = ConfigDict(extra="forbid")

    seed: int = 0
    items: list[ColorClass] = Field(
        default_factory=lambda: [
            ColorClass.LIGHT,
            ColorClass.DARK,
            ColorClass.COLORED,
            ColorClass.COLORED,
            ColorClass.LIGHT,
            ColorClass.DARK,
        ]
    )  # colors of the items in the box at start
    miss_prob: float = 0.2  # a pick grabs nothing
    double_prob: float = 0.1  # a pick from the box grabs two items
    motion_s: float = 0.0  # duration of every arm motion; > 0 to watch the loop live
    width: int = 640
    height: int = 480
    cam_height_mm: float = 400  # camera above the zone surface at the look pose
    item_radius_mm: float = 35
    zones: dict[Zone, SimZoneConfig] = Field(default_factory=_default_zones)
