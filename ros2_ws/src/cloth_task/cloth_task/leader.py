"""StarArm102 / reBot Arm 102 leader ("Spark") → reBot B601-RS joint targets.

The leader is 7 FashionStar UART servos (ids 0..6, 1 Mbaud, `motorbridge-smart-servo`). The
conventions are LeRobot's (teleoperators/rebot_102_leader, robots/rebot_b601_follower), so a
leader calibrated with LeRobot works here unchanged:

  leader raw angle ─(unwrap, × joint_direction, clamp to range)→ public angle (LeRobot)
  public angle ─(× -1, the RS family's motor direction)→ B601-RS motor angle = our joint angle

The gripper's direction carries a ×6 scale (leader travel → follower gripper motor travel);
our driver's gripper opening is motor angle / GRIPPER_OPEN_DEG.
ROS-free, so it can be unit-tested.
"""

from __future__ import annotations

JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
)
SERVO_IDS = {name: i for i, name in enumerate(JOINTS)}
BAUDRATE = 1_000_000
# LeRobot RebotArm102LeaderConfig defaults
DIRECTIONS = {
    "shoulder_pan": -1,
    "shoulder_lift": -1,
    "elbow_flex": 1,
    "wrist_flex": 1,
    "wrist_yaw": 1,
    "wrist_roll": -1,
    "gripper": -6,
}
RANGES_DEG = {
    "shoulder_pan": (-150.0, 150.0),
    "shoulder_lift": (-200.0, 1.0),
    "elbow_flex": (-200.0, 1.0),
    "wrist_flex": (-80.0, 90.0),
    "wrist_yaw": (-90.0, 90.0),
    "wrist_roll": (-90.0, 90.0),
    "gripper": (-270.0, 0.0),
}
RS_DIRECTION = -1.0  # LeRobot RS_PROFILE.joint_directions: raw motor = -public


def unwrap(value: float, lo: float, hi: float) -> float:
    """Remove whole turns from a multi-turn servo angle: into [center-180, center+180]."""
    center = (lo + hi) / 2.0
    return value - round((value - center) / 360.0) * 360.0


def public_angle(name: str, raw_deg: float) -> float:
    """LeRobot's leader action for one joint, degrees (RebotArm102Leader.get_action)."""
    lo, hi = RANGES_DEG[name]
    direction = DIRECTIONS[name]
    sign = 1.0 if direction >= 0 else -1.0
    value = unwrap(raw_deg, lo * sign, hi * sign) * direction
    return max(lo, min(hi, value))


def to_follower(
    raw_deg: dict[str, float], gripper_open_deg: float, flip: frozenset[str] = frozenset()
) -> tuple[list[float], float]:
    """Leader raw servo angles → (joint1..joint6 in degrees, gripper opening 0..1) for our
    driver (motor angle = URDF joint angle). `flip`: arm joints (leader names) whose direction
    is reversed on this particular leader."""
    q = [
        (-1.0 if name in flip else 1.0) * RS_DIRECTION * public_angle(name, raw_deg[name])
        for name in JOINTS[:6]
    ]
    gripper_motor_deg = RS_DIRECTION * public_angle("gripper", raw_deg["gripper"])
    opening = max(0.0, min(1.0, gripper_motor_deg / gripper_open_deg))
    return q, opening
