"""Config models of the rover navigation sim (`nav`): the Leo Rover 1.9, its OAK-D, the world."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


class LeoConfig(BaseModel):
    """`nav.leo`: the Leo Rover 1.9 and its firmware (docs.fictionlab.pl, leo_description)."""

    model_config = ConfigDict(extra="forbid")

    wheel_radius_m: float = 0.0625
    wheel_separation_m: float = 0.358
    # the firmware scales the commanded turn rate by this before it goes to the wheels, because
    # the tyres slip sideways in a skid-steer turn (firmware param, default 1.76)
    angular_velocity_multiplier: float = 1.76
    max_linear_mps: float = 0.4
    max_angular_rps: float = 1.0
    max_wheel_rps: float = 6.4  # 0.4 m/s at the wheel radius
    wheel_torque_nm: float = 3.0  # the URDF says 2; it climbs 45° with 5 kg, so a bit more
    accel_mps2: float = 1.0  # ramp of the commanded linear speed
    angular_accel_rps2: float = 3.0
    cmd_timeout_s: float = 0.5  # no cmd_vel for this long → the firmware stops the wheels
    # rubber on wood / laminate; the floor gets the same (MuJoCo uses the larger of the two).
    # The sim's skid-steer turn comes out at ~55 % of the commanded rate (soft contacts slip
    # more than real tyres); the commands close the loop on the gyro, so only the time differs
    tyre_friction: float = 0.7
    gyro_noise_rps: float = 0.002  # per-sample noise of the IMU's yaw rate


class OakDConfig(BaseModel):
    """`nav.camera`: the Luxonis OAK-D. RGB (IMX378) with the stereo depth aligned to it.

    Specs (docs.luxonis.com): RGB HFOV 69°, stereo pair OV9282 HFOV 72° / VFOV 49°, 7.5 cm
    baseline, depth MinZ ~0.8 m at 800P or ~0.2 m with extended disparity at 400P, < 2 % error
    below 4 m. The OAK-D (not Pro) has no IR projector: plain surfaces give no depth.
    """

    model_config = ConfigDict(extra="forbid")

    width: int = 640
    height: int = 480
    rgb_hfov_deg: float = 69.0
    stereo_vfov_deg: float = 49.0  # rows outside it have no depth in the aligned image
    baseline_m: float = 0.075
    stereo_width: int = 640  # the stereo pair runs at 640x400 (400P) or 1280x800 (800P)
    depth_mode: Literal["extended", "normal"] = "extended"  # extended disparity: MinZ ~0.2 m
    subpixel_bits: int = 3  # disparity steps of 1/8 px; 0 = integer disparity
    disparity_noise_px: float = 0.08
    max_range_m: float = 12.0
    texture_min: float = 1.0  # gray-level std (7x7) below which stereo finds no match
    dropout: float = 0.002  # share of random invalid pixels
    rgb_noise: float = 2.0  # gray levels
    # the mount on the rover's top plate, base_link frame: x forward, z up (m); pitch down (deg)
    mount_xyz_m: tuple[float, float, float] = (0.10, 0.0, 0.075)
    pitch_deg: float = 25.0


class GoalConfig(BaseModel):
    """`nav.goal`: where the sock must end up, rover frame (x forward from the rover's center).

    The default is the floor patch in front of the rover where the arm picks (stage 0's
    `floor_view`), centered: the sock center 0.33-0.53 m ahead, within ±0.10 m sideways.
    """

    model_config = ConfigDict(extra="forbid")

    center_m: tuple[float, float] = (0.43, 0.0)
    half_size_m: tuple[float, float] = (0.10, 0.10)
    max_push_m: float = 0.05  # the rover may not shove the sock further than this


class RealConfig(BaseModel):
    """`nav.real`: the real Leo Rover (rosbridge) and the real OAK-D (depthai)."""

    model_config = ConfigDict(extra="forbid")

    # on the rover: ws://127.0.0.1:9090; from a laptop on the rover's Wi-Fi: ws://10.0.0.1:9090
    rosbridge_url: str = "ws://127.0.0.1:9090"
    cmd_vel_topic: str = "/cmd_vel"
    odom_topic: str = "/merged_odom"
    connect_timeout_s: float = 5.0
    # below the rover's 0.4 m/s and 1 rad/s until the commands are trusted on the real floor
    max_linear_mps: float = 0.25
    max_angular_rps: float = 0.8
    oakd_device: str = ""  # MxID or IP of the OAK-D; empty = the first one found
    oakd_fps: float = 15.0


class NavConfig(BaseModel):
    """`nav`: the rover navigation simulator (rover stage N)."""

    model_config = ConfigDict(extra="forbid")

    leo: LeoConfig = Field(default_factory=LeoConfig)
    camera: OakDConfig = Field(default_factory=OakDConfig)
    goal: GoalConfig = Field(default_factory=GoalConfig)
    timestep_s: float = 0.002
    control_hz: float = 50.0  # rate of the firmware loop and the commands' feedback
    realtime: float = 1.0  # the live panel: simulated s per wall s (the CLI runs flat out)
    stream_fps: float = 10.0  # frames the live panel renders while a command runs
    max_command_s: float = 30.0  # a command that runs longer is stopped
    detector: Literal["color", "sam3", "seg"] = "color"  # the algorithm's sock detector
    sam_prompt: str = "sock"
    real: RealConfig = Field(default_factory=RealConfig)
