# Block 5: Arm Control

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `feat/sim-3d-arm`
**Owned paths:** `src/sorter/arm/`, `tests/arm/`, `rebot_b601/`, `config/default.yaml` → `arm`, `config/rig.yaml` → `poses`, `zones`

## Goal

Safe, blocking, high-level arm operations for the state machine, on the Seeed reBot Arm B601-RS (RobStride).

## Scope

- [x] `ArmDriver` (`sorter.arm.driver`, [D-014](../decisions.md)): `RebotDriver` wraps `rebot_b601.arm.Arm` (CAN bus, or its simulated motors with `arm.dry_run`); the simulator has `SimDriver`. Units: the controller works in mm + rad, `sorter.arm.kinematics` converts.
- [x] `ArmController` per [architecture.md](../architecture.md): `sorter.arm.controller.Controller`, the same code on both drivers.
- [x] Named poses and zones for the table layout: `python -m sorter.sim.layout --write` computes them with IK and checks every pick on a grid and every pose-to-pose move ([D-015](../decisions.md)).
- [ ] Pose teaching tool on the real rig: hold the arm limp or in gravity compensation, move it by hand, save joint angles to `config/rig.yaml` → `poses` (`rest`, `home`, `look_box`, `look_bg`, `place_bg`, `bin_light`, `bin_dark`, `bin_colored`). Check them with `python -m sorter.sim.layout`'s `check()`.
- [x] Zones in `config/rig.yaml` → `zones.<zone>`: `workspace_mm` (XY polygon), `z_floor_mm`, `grasp_depth_mm`, `approach_mm`, `lift_z_mm`.
- [x] `pick`: plan all three moves first (reject XY outside the zone workspace or any IK / path failure with `TargetRejected`, no motion); clamp Z to `z_floor_mm`; above the target with the gripper down, open, straight down, close, straight up to `lift_z_mm`. `PickResult` from the gripper opening.
- [x] TCP: the URDF `gripper_end` (targets are fingertip positions); the flange is `link6`. No `tcp_offset_mm` key.
- [x] **Hold, not disable** ([D-009](../decisions.md)): `hold()` aborts the motion and holds the joints, is thread-safe, and makes every later motion raise `EStopped` until `recover()`. `recover()` lifts to `safe_z_mm`, opens the gripper above the background, and goes home.
- [x] `shutdown()`: release a hold, lift, go to `rest`, then disable. Never disable anywhere else.
- [ ] First run on the hardware: `backends.arm: real` with `arm.dry_run: true`, then the bus (`uv sync --extra hardware`), slow `arm.speed_scale`, hand on the power switch.
- [ ] Tune grasp depth, approach, gripper opening threshold (`gripper.empty_below`), and release heights on real clothes. Record the working values in config and here.

## Depends on / Unblocks

- Depends on: 0 (interface). Hardware.
- Unblocks: 2 (FK, moving to calibration poses), look poses for 1, integration.

## Acceptance criteria

- All named poses reachable repeatedly. The pick / place / drop cycle works on real clothes with hard-coded points.
- A target outside the zone workspace is rejected without motion. Z never goes below `z_floor_mm`.
- `hold()` from another thread stops motion within a fraction of a second, and the arm stays up. `recover()` resumes.

## Notes & risks

- **Top-down reach is small** (from the URDF, not verified on the hardware): with the gripper pointing straight down the TCP reaches at most ~120 mm above the base plate, at 0.20–0.35 m from the base (0.07 m high at 0.40 m, nothing at 0.45 m). The look poses sit at that limit (joint 4 at −80°). The layout is built around it ([D-015](../decisions.md)): a low tray, the mat close in front, a lower lift on the mat (`lift_z_mm` 70) than over the tray wall (100).
- **No collision model.** Paths are checked only against the table (`arm.z_min_mm`) and the base column. A joint-space move between poses can sweep the gripper or the wrist camera through a box wall; the layout keeps walls low. Watch the first real moves.
- `rebot_b601`'s hardware backend follows Seeed's reference code but has not run on the arm yet (its README): enabling sequence, behavior when frames stop. Start with `arm.dry_run`, then read-only (`python -m rebot_b601 state`), then small moves.
- Disabling the motors drops the arm. Only `shutdown()` disables, after `rest`. The physical e-stop switch is the power cut.
- The wrist camera descends with the gripper. `sim.camera_mount_mm` (140 mm behind the fingertips, 55 mm off-axis) is an assumption: measure the real mount and update it, then recompute the poses.
- **Hardware is the B601-RS (RobStride), not the DM** ([D-011](../decisions.md)): RS-06 on joints 1–3, RS-00 on joints 4–6 and the gripper (motor 7). On Linux bring `can0` up at 1 Mbit/s with the PCAN-USB adapter. Zero calibration is done once with Motorbridge Studio (`motorbridge-gateway`, which holds the bus: stop it before running the sorter).
- Speeds: `arm.speed_scale` 0.5 of `rebot_b601`'s joint speeds (capped at 0.6). A full sim cycle at real speed takes about a minute per item.

## Open questions

- Gripper opening with cloth: can it tell "holding cloth" from "empty" at all? If not, set `gripper.empty_below` so `likely_empty` is never true.

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: arm fixed (reBot B601-RS); `park()` replaced by `look(zone)`; `pick(target, zone) -> PickResult`; `hold` / `recover` instead of e-stop (D-006, D-009).
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/arm/config.py` (placeholder). Real backend: `sorter/arm/backend.py` → `create(cfg) -> ArmController`. Python is 3.11 and the SDK is not in `pyproject.toml` yet: this block adds it (D-010). See architecture.md → Wiring.
- 2026-09-25: corrected the arm model everywhere: it is the **B601-RS (RobStride)**, not the DM (Damiao) ([D-011](../decisions.md)); RS hardware notes and the top-down reach limit added to *Notes & risks*.
- 2026-09-26: controller, drivers and kinematics on `rebot_b601` instead of the Seeed SDK (D-014); the simulator runs this controller. Table layout, poses and zones from `sim.layout` (D-015). Config: `arm` typed (`dry_run`, `speed_scale`, `approach`, `safe_z_mm`, `z_min_mm`, release heights, `gripper`), `zones.<zone>.lift_z_mm` added, `tcp_offset_mm` / `grasp_rpy_deg` dropped. `rebot_b601`: planning is module-level (`plan_path`, `check_path`, `check_limits`), `Arm.execute_path`, `Arm.joints`, `mcp` and `motorbridge` are extras.
- 2026-09-26 (Softjey + Claude): the physics simulator runs `rebot_b601.arm.Arm` unchanged on MuJoCo motors (D-016); its tracking-error check caught a joint-space move from a bin into the gripper sweeping through the bin wall, so `drop_to_bin` now returns via `home`. `arm.gripper.open` is 0.6 (a fully open finger hit the box wall). The gripper force per motor torque in the sim is an assumption (`GRIPPER_N_PER_NM`): measure it on the arm.
- 2026-09-26 (Softjey + Claude): for the dashboard's setup mode (D-018) `Controller` has `held`, `at`, `go_to`, `move_joints`, `set_gripper`, `release` (leave hold without moving), `set_pose`; not in the `ArmController` protocol.
