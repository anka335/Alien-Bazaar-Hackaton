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
    same_sock_mm: float = 70.0  # grasp points this close are the same sock
    max_sock_height_mm: float = 80.0  # a grasp point higher above the floor is not on the floor
    min_sock_volume_mm3: float = 15000.0  # a drop adds at least this much cloth to the box
    cloth_min_mm: float = 4.0  # the box's floor: lower is depth noise, not cloth
    cargo_margin_mm: float = 8.0  # the box's walls: measured this far inside them
