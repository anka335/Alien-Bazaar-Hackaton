"""Config models for block 7 (dashboard). Block 0 set the fields its placeholder server uses."""

from pydantic import BaseModel, ConfigDict


class DashboardConfig(BaseModel):
    """`dashboard`: host, port, stream fps, JPEG quality, optional scene_camera."""

    model_config = ConfigDict(extra="allow")

    host: str = "127.0.0.1"
    port: int = 8000
