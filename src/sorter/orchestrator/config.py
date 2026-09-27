"""Config models for block 6 (state machine)."""

from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field


class StateMachineConfig(BaseModel):
    """`state_machine`."""

    model_config = ConfigDict(extra="forbid")

    empty_confirmations: int = 2  # EMPTY box results in a row before DONE
    max_consecutive_failures: int = 5  # then ERROR, paused
    low_confidence: float = 0.5  # color confidence below this logs a warning
    save_runs: bool = True  # write run logs
    runs_dir: Path = Path("data/runs")  # relative to the working directory


class LoadConfig(BaseModel):
    """`load`: the load loop (stage A)."""

    model_config = ConfigDict(extra="forbid")

    max_attempts: int = 3  # picks of one sock before it is left where it lies
    empty_rounds: int = 1  # rounds of all the scan views without a sock before DONE
    aim_off_center: float = 0.45  # a sock this far off the image center gets a closer look
    aim_heights_mm: list[float] = Field(default_factory=lambda: [300.0, 270.0, 240.0, 210.0])
    # two sightings are one sock if either one's grasp point lies this close to the other's
    # cloth (its mask on the floor): the grasp point moves between views, the cloth doesn't
    same_sock_mm: float = 30.0
    max_sock_height_mm: float = 80.0  # a grasp point higher above the floor is not on the floor
    raised_mm: float = 4.0  # the box's surface rose by this much where a dropped sock lies
    min_raised_mm2: float = 1500.0  # … over at least this area (a sock is ~180 cm² flat)
    cargo_margin_mm: float = 8.0  # the box's walls: measured this far inside them
