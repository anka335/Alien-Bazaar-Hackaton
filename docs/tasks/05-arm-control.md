# Block 5: Arm Control

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** `src/sorter/arm/`, `tests/arm/`, `config/default.yaml` → `arm`, `config/rig.yaml` → `poses`, `zones`

## Goal

Safe, blocking, high-level arm operations for the state machine, on the Seeed reBot Arm B601-RS (RobStride).

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
- The SDK requires Python < 3.12 and has no `[build-system]` (D-010): clone it next to the repo and add it as a path / editable dependency (e.g. `uv add --editable ../reBotArm_control_py`, may need a small `[build-system]` patch). Its `config/` and `urdf/` are loaded by relative path; check that they are found.

- Standalone clothing-follow preview is verified; hardware following is not. Its cached RGB-D pairs lack an age check, and fault recovery offers torque disable after physical-support confirmation, which needs review against the rest-pose-only rule.

## Open questions

- Gripper force and `empty_below` threshold: can the gripper opening tell "holding cloth" from "empty" at all? If not, set it so `likely_empty` is never true.

## Requests from other blocks

_None yet._
- **Hardware is the B601-RS (RobStride), not the DM** ([D-011](../decisions.md)). The SDK reads `config/rebotarm_rs.yaml` (selected by `hardware_yaml` in `config/rebotarm.yaml`): RS-06 on joints 1–3, RS-00 on joints 4–6 and the gripper (motor 7). On Linux bring `can0` up at 1 Mbit/s with the PCAN-USB adapter; there is no serial port (that is the DM path). Zero calibration is done once with Motorbridge Studio (`motorbridge-gateway`, which holds the bus: stop it before running the SDK). A standalone RS driver, IK and simulator that can seed `ArmDriver` lives in `rebot_b601/`.
- **Top-down reach is limited** (computed from the URDF with `rebot_b601/`, not verified on the hardware): with the tool pointing straight down the TCP only reaches z ≲ 0.14 m above the base plate (x ≈ 0.1–0.45 m); pointing forward only z ≳ 0.15 m. Choose `look_*` poses, `safe_z_mm` and the approach height with this in mind, or relax the top-down requirement for the approach.

## Log

- 2026-09-25: arm fixed (reBot B601-RS); `park()` replaced by `look(zone)`; `pick(target, zone) -> PickResult`; `hold` / `recover` instead of e-stop (D-006, D-009).
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/arm/config.py` (placeholder). Real backend: `sorter/arm/backend.py` → `create(cfg) -> ArmController`. Python is 3.11 and the SDK is not in `pyproject.toml` yet: this block adds it (D-010). See architecture.md → Wiring.
- 2026-09-25: corrected the arm model everywhere: it is the **B601-RS (RobStride)**, not the DM (Damiao) ([D-011](../decisions.md)); RS hardware notes and the top-down reach limit added to *Notes & risks*.
- 2026-09-26: added a separate guarded clothing-follow integration test around the standalone `rebot_b601` driver. It remains outside the block-5 `ArmController` acceptance criteria and defaults to camera-only preview (D-015).
- 2026-09-26: camera-only clothing preview passed on Windows/WSL with ROS 2 Jazzy in Docker and SAM3. Physical following remains unverified. Manual tests are on `block/05-manual-arm-tests`; the block backend remains todo.
