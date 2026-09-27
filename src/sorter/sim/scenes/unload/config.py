"""`sim.unload`: the unload scene (stage B)."""

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import ColorClass


class UnloadSceneConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    # socks of each color in the cargo box at start: in the compartment of their color, or
    # anywhere in the box if it has no compartment for it (an undivided box)
    cargo: dict[ColorClass, int] = Field(default_factory=lambda: dict.fromkeys(ColorClass, 1))
    sock_mm: tuple[float, float] = (120.0, 120.0)  # the flat cloth sheet, before it is gathered
