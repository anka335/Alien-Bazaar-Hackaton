"""Config models for the simulator (`sim`, block 0)."""

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import ColorClass


class RectConfig(BaseModel):
    """An axis-aligned rectangle on the table, arm frame, mm."""

    model_config = ConfigDict(extra="forbid")

    center_mm: tuple[float, float]
    size_mm: tuple[float, float]  # along x, along y


class BoxLayout(RectConfig):
    """The mixed box: `size_mm` is the inside."""

    floor_z_mm: float = 5.0
    wall_mm: float = 60.0  # wall height above the floor


class BinsLayout(BaseModel):
    model_config = ConfigDict(extra="forbid")

    centers_mm: dict[ColorClass, tuple[float, float]]
    size_mm: float = 180.0  # square, outside
    wall_mm: float = 150.0
    floor_z_mm: float = 5.0


def _default_bins() -> BinsLayout:
    return BinsLayout(
        centers_mm={
            ColorClass.LIGHT: (257.0, 306.0),
            ColorClass.DARK: (0.0, 400.0),
            ColorClass.COLORED: (-257.0, 306.0),
        }
    )


class LayoutConfig(BaseModel):
    """The table around the arm (arm base at the origin, +x forward, +y left), mm.

    The committed `poses` and `zones` in `rig.yaml` are computed from this layout
    (`python -m sorter.sim.layout`); build the real table the same way.
    """

    model_config = ConfigDict(extra="forbid")

    box: BoxLayout = Field(
        default_factory=lambda: BoxLayout(center_mm=(0.0, -270.0), size_mm=(240.0, 180.0))
    )
    background: RectConfig = Field(
        default_factory=lambda: RectConfig(center_mm=(270.0, 0.0), size_mm=(240.0, 180.0))
    )
    bins: BinsLayout = Field(default_factory=_default_bins)


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
    miss_prob: float = 0.2  # a closed gripper holds nothing although it reached the cloth
    double_prob: float = 0.1  # a pick from the box grabs two items
    time_scale: float = 0.0  # arm motion time × this; 1 = the real arm's speed, 0 = instant
    vision_s: float = 0.0  # how long sim vision "thinks" per frame
    width: int = 640
    height: int = 480
    focal_px: float = 615.0  # RealSense D435i color at 640x480
    # the wrist camera in the TCP frame (x = approach); it looks along the approach axis
    camera_mount_mm: tuple[float, float, float] = (-140.0, 0.0, 55.0)
    item_radius_mm: float = 30
    layout: LayoutConfig = Field(default_factory=LayoutConfig)
