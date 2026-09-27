"""Config models for block 5 (arm, poses, zones)."""

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import ColorClass, Zone

# where the wrist camera looks at a zone from; the zones with a look pose are the pick zones
LOOK_POSES = {Zone.FLOOR: "look_floor", Zone.CARGO: "look_cargo"}
POSE_NAMES = (
    "rest",
    "home",
    *LOOK_POSES.values(),
    *(f"cargo_{c.value}" for c in ColorClass),  # above a cargo compartment: drop there
    *(f"laundry_{c.value}" for c in ColorClass),  # above a laundry bin: drop there
)


class GripperConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    open: float = 1.0  # opening before a grasp and at a release, 0 = closed .. 1 = open
    empty_below: float = 0.01  # a closed gripper opening below this → `likely_empty`


class ArmConfig(BaseModel):
    """`arm`: driver, speed, pick geometry, gripper, what the arm must not hit."""

    model_config = ConfigDict(extra="forbid")

    dry_run: bool = False  # real backend: rebot_b601's simulated motors instead of the CAN bus
    speed_scale: float = 0.5  # of rebot_b601's joint speeds; the driver caps it at 0.6
    approach: Literal["down"] = "down"  # tool orientation for a pick
    safe_z_mm: float = 100.0  # recover() / shutdown() lift the TCP to this height first
    z_min_mm: float = 3.0  # floor clearance: no point of the arm goes lower
    drop_height_mm: float = 30.0  # TCP above the rim at `cargo_<color>` / `laundry_<color>`
    # boxes no point of the arm may enter, [x0, x1, y0, y1, z0, z1] in mm, arm frame: the rover
    # body, the cargo box walls (rig.yaml, from `python -m sorter.sim.layout --write`)
    keep_out_mm: list[tuple[float, float, float, float, float, float]] = Field(default_factory=list)
    keep_out_margin_mm: float = 15.0  # the checked points are on the links' axes: their radius
    gripper: GripperConfig = Field(default_factory=GripperConfig)


class ZoneConfig(BaseModel):
    """`zones.<zone>`: where a pick is allowed and how deep it goes."""

    model_config = ConfigDict(extra="forbid")

    workspace_mm: list[tuple[float, float]]  # XY polygon, arm frame
    z_floor_mm: float  # the grasp never goes lower (the floor, a compartment's floor)
    grasp_depth_mm: float = 15.0  # below the cloth surface
    approach_mm: float = 40.0  # above the cloth surface before descending
    lift_z_mm: float = 100.0  # TCP height after the pick (clear of walls and the rover)
