"""Config models for block 5 (arm, poses, zones). Placeholder from block 0: block 5 fills it in."""

from pydantic import BaseModel, ConfigDict


class ArmConfig(BaseModel):
    """`arm`: SDK config path, speeds, tcp_offset_mm, grasp_rpy_deg, safe_z_mm, gripper, ..."""

    model_config = ConfigDict(extra="allow")


class ZoneConfig(BaseModel):
    """`zones.<zone>`: workspace_mm, z_floor_mm, grasp_depth_mm, approach_mm."""

    model_config = ConfigDict(extra="allow")
