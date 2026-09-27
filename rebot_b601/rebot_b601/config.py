"""Hardware and safety configuration for the reBot Arm B601-RS.

Motor layout, models and PID values follow Seeed's reference code
(Seeed-Projects/reBotArm_control_py ``config/rebotarm_rs.yaml`` and
Seeed-Projects/lerobot-robot-seeed-b601).  Everything that limits motion is
deliberately conservative and can be overridden with ``REBOT_*`` environment
variables.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


def _env_bool(name: str, default: bool = False) -> bool:
    val = os.environ.get(name)
    if val is None:
        return default
    return val.strip().lower() in ("1", "true", "yes", "on")


JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")

# ---- CAN / motors ---------------------------------------------------------
CAN_CHANNEL = os.environ.get("REBOT_CAN_CHANNEL", "can0")
ARM_MOTOR_IDS = (1, 2, 3, 4, 5, 6)
GRIPPER_MOTOR_ID = 7
FEEDBACK_ID = 0xFD
ARM_MOTOR_MODELS = ("rs-06", "rs-06", "rs-06", "rs-00", "rs-00", "rs-00")
GRIPPER_MOTOR_MODEL = "rs-00"

# RobStride POS_VEL parameters written before enabling (SDK values):
# vlim [rad/s] (0x7017), vel_kp (0x701F), vel_ki (0x7020), pos_kp (0x701E)
PV_PARAMS = (
    dict(vel_kp=12.0, vel_ki=0.1, pos_kp=13.0),
    dict(vel_kp=14.0, vel_ki=0.1, pos_kp=16.0),
    dict(vel_kp=14.0, vel_ki=0.1, pos_kp=14.0),
    dict(vel_kp=5.0, vel_ki=0.1, pos_kp=20.0),
    dict(vel_kp=4.0, vel_ki=0.1, pos_kp=10.0),
    dict(vel_kp=4.0, vel_ki=0.1, pos_kp=10.0),
)
# hard velocity limit programmed into the motors [rad/s]; the trajectory
# generator keeps the commanded speed far below this.
MOTOR_VLIM_RAD_S = _env_float("REBOT_MOTOR_VLIM", 1.5)

# ---- Joint limits (degrees == motor degrees after zero calibration) -------
# Intersection of the URDF limits and the soft limits of Seeed's follower
# driver.  Home (all zeros) is inside, at the lower bound of joints 2 and 3.
JOINT_LIMITS_DEG = np.array(
    [
        (-145.0, 145.0),   # joint1 shoulder pan
        (0.0, 170.0),      # joint2 shoulder lift
        (0.0, 180.0),      # joint3 elbow
        (-80.0, 90.0),     # joint4 wrist flex
        (-90.0, 90.0),     # joint5 wrist yaw
        (-130.0, 130.0),   # joint6 wrist roll
    ]
)
JOINT_LIMITS_RAD = np.radians(JOINT_LIMITS_DEG)
HOME_DEG = np.zeros(6)

# Peak joint speed at speed_scale=1 [deg/s]; the actual speed is this times
# the requested scale (which is itself capped by MAX_SPEED_SCALE).
# joint6 at 40, not 90: on the rig it lagged ~20 deg behind at 45 deg/s (the wrist camera cable)
JOINT_SPEED_DPS = np.array([40.0, 30.0, 30.0, 60.0, 60.0, 40.0])
DEFAULT_SPEED_SCALE = _env_float("REBOT_DEFAULT_SPEED", 0.3)
MAX_SPEED_SCALE = _env_float("REBOT_MAX_SPEED", 0.6)
MIN_MOVE_TIME_S = 0.4
# Peak acceleration of a move, as JOINT_SPEED_DPS per second (4: joint2 120 deg/s^2, joint4
# 240 deg/s^2). The ramp to speed_scale s takes pi*s/(2*JOINT_ACCEL) s: 0.55 s at 1.4.
JOINT_ACCEL = _env_float("REBOT_JOINT_ACCEL", 4.0)

# ---- Control loop ---------------------------------------------------------
CONTROL_HZ = 50.0

# ---- Workspace / safety ---------------------------------------------------
# TCP box in the base frame [m].  Z=0 is the base plate; the table is assumed
# to be at z=0.  Adjust with REBOT_Z_MIN if you mounted the arm on something
# else or have objects on the table.
Z_MIN = _env_float("REBOT_Z_MIN", 0.03)
WORKSPACE_X = (-0.60, 0.60)
WORKSPACE_Y = (-0.60, 0.60)
WORKSPACE_Z = (Z_MIN, 0.70)

# Abort a move when a joint is this far from where it was commanded for
# TRACKING_ERR_TIME_S (arm blocked, collided, or motor fault).
TRACKING_ERR_DEG = _env_float("REBOT_TRACKING_ERR_DEG", 12.0)
TRACKING_ERR_TIME_S = 0.4
# Plus this many seconds of the commanded joint speed: a joint that follows a moving setpoint
# with a delay (a late control loop over USB-CAN, slow feedback: ~0.3 s on the macOS rig) lags
# by delay x speed without being blocked. A blocked joint's error keeps growing past it.
TRACKING_LAG_S = _env_float("REBOT_TRACKING_LAG_S", 0.4)
# Motor feedback that doesn't change at all for this long (bit for bit: position, velocity,
# torque of every motor) is stale: the adapter stopped receiving while the commands still go
# out. On the rig it stays the same ≤ 0.3 s normally; a dead receive froze it for minutes and
# the arm jerked back to the frozen pose on every fault. Hardware only.
STALE_FEEDBACK_MOVING_S = 0.5
STALE_FEEDBACK_IDLE_S = 1.5
# Refuse to enable the motors when the measured pose is outside the limits by
# more than this (usually means the zero calibration is missing / wrong).
POSE_SANITY_MARGIN_DEG = 15.0
# MOSFET temperature thresholds [deg C] (Seeed follower defaults)
TEMP_WARN_C = 115.0
TEMP_STOP_C = 125.0
TEMP_DISABLE_C = 135.0

# ---- Gripper (motor 7, MIT torque control like Seeed's follower) ----------
# motor angle [deg] at "fully open"; 0 deg = closed.  Seeed's follower allows
# 0..270 deg; stay below that until you have checked your gripper.
GRIPPER_OPEN_DEG = _env_float("REBOT_GRIPPER_OPEN_DEG", 240.0)
GRIPPER_KP = 12.0
GRIPPER_KD = 0.05
GRIPPER_TORQUE_LIMIT = _env_float("REBOT_GRIPPER_TORQUE", 4.5)   # Nm while moving (3: socks slipped out)
GRIPPER_HOLD_TORQUE = _env_float("REBOT_GRIPPER_HOLD_TORQUE", 2.5)  # Nm when stalled/holding (RS00: 5 rated)
GRIPPER_ENABLED = not _env_bool("REBOT_DISABLE_GRIPPER", False)

# ---- Dry run --------------------------------------------------------------
# REBOT_DRY_RUN=1 makes ``Arm.connect()`` use a simulated arm (no CAN access).
DRY_RUN = _env_bool("REBOT_DRY_RUN", False)
