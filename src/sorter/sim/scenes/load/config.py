"""`sim.load`: the load scene (stage A)."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import ColorClass


class PlacedSock(BaseModel):
    """A sock at a given spot (arm base frame), not a random one: the mission hands over the
    socks the rover stopped next to."""

    model_config = ConfigDict(extra="forbid")

    color: ColorClass
    x_mm: float
    y_mm: float
    yaw_rad: float = 0.0
    rgb: tuple[float, float, float] | None = None  # None: a random color of the class
    bunched: bool = False


class LoadSceneConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    # socks lying on the floor around the rover at start
    socks: list[ColorClass] = Field(
        default_factory=lambda: [ColorClass.LIGHT, ColorClass.DARK, ColorClass.COLORED]
    )
    # where they lie: anywhere the arm reaches on the floor (the ring below, or `zone_mm`: the
    # floor zone from rig.yaml, which `watch` and `bench` pass), or only in the floor view
    area: Literal["reach", "zone", "view"] = "reach"
    zone_mm: list[tuple[float, float]] = Field(default_factory=list)  # "zone": XY polygon
    # "reach": the ring around the arm's base where it picks from the floor (the floor zone in
    # rig.yaml, less a sock's half width)
    reach_mm: tuple[float, float] = (265.0, 410.0)
    reach_deg: float = 125.0  # "reach": up to this far from straight ahead, both sides
    margin_mm: float = 40.0  # "view": a sock's center stays this far inside the floor view
    sock_mm: tuple[float, float] = (200.0, 90.0)  # a sock lying flat: length, width
    bunched_prob: float = 0.3  # a sock lies bunched up instead of flat
    placed: list[PlacedSock] = Field(default_factory=list)  # given: these instead of `socks`
