"""Config models for block 5 (arm, poses, zones)."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

POSE_NAMES = (
    "rest",
    "home",
    "look_box",
    "look_bg",
    "place_bg",
    "bin_light",
    "bin_dark",
    "bin_colored",
)


class GripperConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    open: float = 1.0  # opening before a grasp and at a release, 0 = closed .. 1 = open
    empty_below: float = 0.01  # a closed gripper opening below this → `likely_empty`


class ArmConfig(BaseModel):
    """`arm`: driver, speed, pick geometry, gripper."""

    model_config = ConfigDict(extra="forbid")

    dry_run: bool = False  # real backend: rebot_b601's simulated motors instead of the CAN bus
    speed_scale: float = Field(1.0, gt=0)  # of rebot_b601's joint speeds, at start
    # the dashboard's speed control goes up to this, at most speed_ceiling() (the motors' limit)
    max_speed_scale: float = Field(1.4, gt=0)
    approach: Literal["down"] = "down"  # tool orientation for a pick
    safe_z_mm: float = 100.0  # recover() / shutdown() lift the TCP to this height first
    z_min_mm: float = 3.0  # table clearance: no point of the arm goes lower
    place_release_height_mm: float = 90.0  # TCP above the background at `place_bg`
    bin_release_height_mm: float = 90.0  # TCP above the bin rim at `bin_<color>`
    gripper: GripperConfig = Field(default_factory=GripperConfig)


class ZoneConfig(BaseModel):
    """`zones.<zone>`: where a pick is allowed and how deep it goes."""

    model_config = ConfigDict(extra="forbid")

    workspace_mm: list[tuple[float, float]]  # XY polygon, arm frame
    z_floor_mm: float  # the grasp never goes lower (box floor / mat)
    grasp_depth_mm: float = 15.0  # below the cloth surface
    approach_mm: float = 40.0  # above the cloth surface before descending
    lift_z_mm: float = 100.0  # TCP height after the pick (the box needs to clear its wall)
