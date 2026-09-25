"""Config models for block 6 (state machine)."""

from pathlib import Path

from pydantic import BaseModel, ConfigDict


class StateMachineConfig(BaseModel):
    """`state_machine`."""

    model_config = ConfigDict(extra="forbid")

    empty_confirmations: int = 2  # EMPTY box results in a row before DONE
    max_consecutive_failures: int = 5  # then ERROR, paused
    low_confidence: float = 0.5  # color confidence below this logs a warning
    save_runs: bool = True  # write run logs
    runs_dir: Path = Path("data/runs")  # relative to the working directory
