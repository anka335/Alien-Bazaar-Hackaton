"""Config models for block 5 (arm, zones): the SO-101 on a Feetech STS3215 bus (D-014)."""

from pydantic import BaseModel, ConfigDict, Field


class GripperConfig(BaseModel):
    """`arm.gripper`. Opening is 0 (closed end of the calibrated range) .. 1 (open end)."""

    model_config = ConfigDict(extra="forbid")

    open: float = Field(0.7, ge=0, le=1)  # opening before a grasp
    closed: float = Field(0.0, ge=0, le=1)  # target when closing on cloth (stalls on the cloth)
    look: float = Field(0.0, ge=0, le=1)  # at the look poses: fixed finger pixels for the ROI
    torque_limit: int = Field(500, ge=50, le=1000)  # RAM Torque_Limit, 1000 = 100 %
    close_s: float = 1.0  # time to close / open
    empty_below: float = 0.03  # opening after closing below this → likely empty
    reversed: bool = False  # True if the low end of the raw range is the open one


class ArmConfig(BaseModel):
    """`arm`: bus, joint mapping, speeds, gripper, pick geometry, timeouts."""

    model_config = ConfigDict(extra="forbid")

    port: str = "auto"  # serial device; auto = the only /dev/cu.usbmodem* (or /dev/ttyACM*)
    baudrate: int = 1_000_000
    ids: list[int] = [1, 2, 3, 4, 5]  # shoulder_pan .. wrist_roll
    gripper_id: int = 6
    # joint = sign · (raw − middle of the EEPROM range) · 2π/4096 + offset (LeRobot degree mode)
    signs: list[float] = [1.0, 1.0, 1.0, 1.0, 1.0]
    offsets_deg: list[float] = [0.0, 0.0, 0.0, 0.0, 0.0]
    limit_margin_deg: float = 2.0  # stay this far inside the EEPROM range

    control_hz: float = 50.0  # goal streaming rate during a motion
    joint_speed_deg_s: float = 60.0  # peak joint speed of joint moves
    linear_speed_mm_s: float = 80.0  # mean TCP speed of straight moves
    step_mm: float = 5.0  # IK waypoint spacing of straight moves
    settle_s: float = 3.0  # after a motion, wait at most this long for the arm to stop
    still_ticks: int = 2  # position unchanged (±2 ticks) for this many reads → still
    max_error_deg: float = 10.0  # joint error after settling above this → ArmError
    sag_passes: int = 2  # re-command the remaining error this many times (P-only servos sag)
    sag_tol_deg: float = 1.0  # no correction below this error

    tcp_extend_mm: float = 0.0  # fingertip point beyond the URDF gripper_frame, along the approach
    grasp_max_tilt_deg: float = 25.0  # grasp approach may tilt this far from straight down
    roll_deg: float = 0.0  # wrist roll during picks (finger line orientation)
    recover_lift_mm: float = 60.0  # recover(): straight up this far first, if reachable
    table_z_mm: float = -5.0  # no straight-move waypoint goes below this, in any zone

    gripper: GripperConfig = Field(default_factory=GripperConfig)


class ZoneConfig(BaseModel):
    """`zones.<zone>` (rig.yaml): where picks are allowed, arm frame, mm."""

    model_config = ConfigDict(extra="forbid")

    workspace_mm: list[tuple[float, float]] = []  # XY polygon the grasp point must be inside
    z_floor_mm: float = 0.0  # the fingertips never go lower
    grasp_depth_mm: float = 30.0  # go this far below the cloth surface before closing
    approach_mm: float = 80.0  # start the straight descent this far above the surface
