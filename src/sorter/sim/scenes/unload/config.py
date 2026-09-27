"""`sim.unload`: the unload scene (stage B)."""

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import ColorClass


class UnloadSceneConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    # socks of each color in the cargo box at start: in the compartment of their color, or
    # anywhere in the box if it has no compartment for it (an undivided box)
    cargo: dict[ColorClass, int] = Field(default_factory=lambda: dict.fromkeys(ColorClass, 1))
    sock_mm: tuple[float, float] = (120.0, 120.0)  # the flat cloth sheet, before it is gathered
    # the station is never quite where the layout says: the rover parks within a tolerance
    # (the whole row of bins shifts and turns), and each bin stands a bit off its own place;
    # uniform in ± these, by seed
    station_mm: float = 0.0
    station_deg: float = 0.0
    bin_mm: float = 0.0
    bin_deg: float = 0.0
    # the real rover's parts the base doesn't draw (electronics, the cargo box's bracket, the
    # box's cardboard): on with its geometry (`sorter.sim.scenes.unload.rover`)
    rover_parts: bool = False
