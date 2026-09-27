"""`sim.load`: the load scene (stage A)."""

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import ColorClass


class LoadSceneConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    # socks lying on the floor at start, scattered over the floor view
    socks: list[ColorClass] = Field(
        default_factory=lambda: [ColorClass.LIGHT, ColorClass.DARK, ColorClass.COLORED]
    )
    margin_mm: float = 40.0  # a sock's center stays this far inside the floor view
