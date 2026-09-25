# Block 5: Arm Control

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** `src/sorter/arm/`, `tests/arm/`, `config/default.yaml` → `arm`, `config/rig.yaml` → `poses`, `zones`

## Goal

Safe, blocking, high-level arm operations for the state machine, on the Seeed reBot Arm B601-DM.

## Scope

- [ ] `ArmDriver`: a thin wrapper over `reBotArm_control_py` (`RebotArm` + `RebotArmEndPose`), internal to this block, plus a mock driver for unit tests. It converts mm ↔ m. Methods: blocking joint move (min-jerk, like the SDK's `safe_home`), blocking linear Cartesian move (`move_to_traj` + wait until still), gripper set/read, joints, FK, hold, disable, gravity compensation on/off.
- [ ] `ArmController` per [architecture.md](../architecture.md): `start`, `shutdown`, `home`, `look`, `pick`, `place_on_background`, `drop_to_bin`, `ee_pose`, `joints`, `hold`, `recover`.
- [ ] Pose teaching tool: gravity-compensation mode, move the arm by hand, save joint angles to `config/rig.yaml` → `poses` (`rest`, `home`, `look_box`, `look_bg`, `place_bg`, `bin_light`, `bin_dark`, `bin_colored`).
- [ ] Zones in `config/rig.yaml` → `zones.<zone>`: `workspace_mm` (XY polygon), `z_floor_mm`, `grasp_depth_mm`, `approach_mm`.
- [ ] `pick`: reject XY outside the zone workspace or on IK failure (`TargetRejected`, no motion); clamp Z to `z_floor_mm`; approach from above with the top-down orientation, descend linearly, close, lift to `safe_z_mm`. Return `PickResult` from the gripper opening.
- [ ] TCP offset (`arm.tcp_offset_mm`): targets are fingertip positions; convert to the flange for IK.
- [ ] **Hold, not disable** ([D-009](../decisions.md)): `hold()` freezes joint targets at the measured position, is thread-safe, and makes every later motion raise `EStopped` until `recover()`. `recover()` lifts to safe Z, opens the gripper above the background, and goes home.
- [ ] `shutdown()`: go to `rest`, then disable. Never disable anywhere else.
- [ ] Tune grasp depth, approach, gripper force, and release height on real clothes. Record the working values in config and here.

## Depends on / Unblocks

- Depends on: 0 (interface). Hardware.
- Unblocks: 2 (FK, moving to calibration poses), look poses for 1, integration.

## Acceptance criteria

- All named poses reachable repeatedly. The pick / place / drop cycle works on real clothes with hard-coded points.
- A target outside the zone workspace is rejected without motion. Z never goes below `z_floor_mm`.
- `hold()` from another thread stops motion within a fraction of a second, and the arm stays up. `recover()` resumes.

## Notes & risks

- **Start with the manual grasp test** (see block 3). It is the earliest signal of whether the approach works. Include the re-grasp from the flat background: a parallel gripper on flat cloth is harder than on a pile.
- **The SDK's `estop()` disables the motors, and the arm falls.** Use it only in `shutdown()` after `rest`. The physical e-stop switch is the power cut.
- `move_to_traj` is non-blocking and returns `False` on IK failure. `move_to_ik` jumps to the target without a trajectory; don't use it for motion.
- SDK units: metres and radians. The SDK's "home" is all joints at 0; ours is the `home` pose from config.
- The camera on the wrist descends with the gripper. Check that it clears the box walls at the wall margin used by block 3.
- The Seeed docs recommend Ubuntu; check the USB-CAN adapter and Pinocchio on the team laptop early.

## Open questions

- Gripper force and `empty_below` threshold: can the gripper opening tell "holding cloth" from "empty" at all? If not, set it so `likely_empty` is never true.

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: arm fixed (reBot B601-DM); `park()` replaced by `look(zone)`; `pick(target, zone) -> PickResult`; `hold` / `recover` instead of e-stop (D-006, D-009).
