"""Config models for block 6 (state machine). Block 0 set the fields its placeholder loop uses."""

from pydantic import BaseModel, ConfigDict


class StateMachineConfig(BaseModel):
    """`state_machine`."""

    model_config = ConfigDict(extra="allow")

    empty_confirmations: int = 2
    max_consecutive_failures: int = 5
