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

- The arm's 50 Hz control loop runs in the sorter's Python process, next to the camera and the web server: a late tick makes the setpoint jump ahead, so the arm jerks and the moving joint trips the tracking-error check. `rebot_b601` logs `control loop late N times …, worst gap … ms` when ticks come over 3 periods apart, with the time per tick part (CAN request / poll, blocking parameter reads per motor, sends). On the rig (macOS, 2026-09-26) every tick took ~200 ms, even idle: `send_pos_vel` is 2–3 CAN frames per motor and the USB-CAN adapter takes ~10–25 ms a frame, so the loop ran at ~5 Hz, hence the jerks. Now the mode and speed limit are set once at enable and a tick writes only the changed position references plus one motor in turn (`REBOT_SEND=pos_vel` for the old way); still to be checked on the rig.
- joint6 (wrist roll) lags ~20° behind its setpoint on the rig at 45 °/s and trips the tracking-error fault (4 times on 2026-09-26, during calibration moves; the look poses turn it 50–100°). Likely the wrist camera's USB cable dragging it: keep the cable slack with room to twist. Its peak speed is now 40 °/s (was 90) in `rebot_b601/config.py → JOINT_SPEED_DPS`.
- **Top-down reach is small** (from the URDF, not verified on the hardware): with the gripper pointing straight down the TCP reaches at most ~120 mm above the base plate, at 0.20–0.35 m from the base (0.07 m high at 0.40 m, nothing at 0.45 m). The look poses sit at that limit (joint 4 at −80°). The layout is built around it ([D-015](../decisions.md)): a low tray, the mat close in front, a lower lift on the mat (`lift_z_mm` 70) than over the tray wall (100).
- **No collision model.** Paths are checked only against the table (`arm.z_min_mm`) and the base column. A joint-space move between poses can sweep the gripper or the wrist camera through a box wall; the layout keeps walls low. Watch the first real moves.
- `rebot_b601`'s hardware backend follows Seeed's reference code but has not run on the arm yet (its README): enabling sequence, behavior when frames stop. Start with `arm.dry_run`, then read-only (`python -m rebot_b601 state`), then small moves.
- Disabling the motors drops the arm. Only `shutdown()` disables, after `rest`. The physical e-stop switch is the power cut.
- The wrist camera descends with the gripper, but joint 6 (wrist roll) doesn't turn it: it is on `link5` (D-022). `sim.camera_mount_mm` (140 mm behind the fingertips, 55 mm off-axis) is an assumption: measure the real mount and update it, then recompute the poses.
- **Hardware is the B601-RS (RobStride), not the DM** ([D-011](../decisions.md)): RS-06 on joints 1–3, RS-00 on joints 4–6 and the gripper (motor 7). On Linux bring `can0` up at 1 Mbit/s with the PCAN-USB adapter. Zero calibration is done once with Motorbridge Studio (`motorbridge-gateway`, which holds the bus: stop it before running the sorter).
- First real-arm run (2026-09-26): going to `look_box` faulted with joint6 13.6° behind its setpoint, though nothing blocked it. That pose (from the sim layout) turns joint6 to 102°, the other poses keep it near 0°. Cause not known yet (RS-00 torque with the camera's weight, zero offset of joint6?): jog J6 in steps to find where it faults, and re-teach `look_box`.
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
- 2026-09-26 (Softjey + Claude): driver faults can be cleared without a restart (D-020): `ArmDriver.fault()` / `clear_fault()`, `Controller.fault` / `clear_fault()`, `rebot_b601.arm.Arm.clear_fault()`; the tracking-error message has the commanded and measured angle.
- 2026-09-26 (Softjey + Claude): `Controller.move_tcp(xyz, linear=)` (gripper down) and `lift()` for the calibration page (D-021); not in the `ArmController` protocol.
- 2026-09-26 (Softjey + Claude): `ee_pose()` returns `T_base_link5` (`kinematics.fk_link5`), the link the camera is fixed to (D-022); `kinematics.T_LINK5_TCP0` is the TCP in link5 with joint 6 at 0.
- 2026-09-26 (Softjey + Claude): runtime speed (D-025): `Controller.speed_scale`, `max_speed_scale`, `set_speed_scale()` (clamped to [0.05, `rebot_b601`'s cap], from the next motion); `build_system` passes the arm as `Hub(speed=)`.
