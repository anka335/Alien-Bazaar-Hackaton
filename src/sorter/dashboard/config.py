"""Config models for block 7 (dashboard)."""

from pydantic import BaseModel, ConfigDict, Field


class DashboardConfig(BaseModel):
    """`dashboard`."""

    model_config = ConfigDict(extra="forbid")

    host: str = "127.0.0.1"
    port: int = 8000
    stream_fps: float = Field(10.0, gt=0)  # live wrist feed; the decision frame is sent on change
    jpeg_quality: int = Field(80, ge=10, le=100)
    status_hz: float = Field(5.0, gt=0)  # max rate of status pushes over /ws
