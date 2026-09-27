"""`sim.unload`: the unload scene (stage B)."""

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import ColorClass


class UnloadSceneConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    # socks in each cargo compartment at start
    cargo: dict[ColorClass, int] = Field(default_factory=lambda: dict.fromkeys(ColorClass, 1))
