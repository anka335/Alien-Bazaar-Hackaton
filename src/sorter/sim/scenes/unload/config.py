"""`sim.unload`: the unload scene (stage B)."""

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import ColorClass


class UnloadSceneConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    # socks of each color in the cargo box at start: in the compartment of their color, or
    # anywhere in the box if it has no compartment for it (an undivided box)
    cargo: dict[ColorClass, int] = Field(default_factory=lambda: dict.fromkeys(ColorClass, 1))
    sock_mm: tuple[float, float] = (120.0, 120.0)  # the flat cloth sheet, before it is gathered
    # limp like a real sock: it hangs from the gripper far enough for the camera to see it
    # (the base's stiffer cloth stays bunched at the fingers, out of view from `show_held`)
    sock_young: float = 1e4  # Pa
    sock_thickness_mm: float = 1.0
    # the station is never quite where the layout says: the rover parks within a tolerance
    # (the whole row of bins shifts and turns), and each bin stands a bit off its own place;
    # uniform in ± these, by seed
    station_mm: float = 0.0
    station_deg: float = 0.0
    bin_mm: float = 0.0
    bin_deg: float = 0.0
