# Block 5: Arm Control

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `block/so101-integration`
**Owned paths:** `src/sorter/arm/`, `tests/arm/`, `config/default.yaml` → `arm`, `config/rig.yaml` → `poses`, `zones`

## Goal

Safe, blocking, high-level arm operations for the state machine, on the SO-101 ([D-014](../decisions.md)).

## Scope

- [x] `FeetechBus` (`feetech-servo-sdk`, sync read / sync write, RAM registers only) and `MockBus` for tests.
- [x] Kinematics from `so101_new_calib.urdf`: FK, jacobian, `ik_down` (position first, "tool down" in the null space, joint limits as an active set).
- [x] `So101Arm` per the `ArmController` contract: joint zero = middle of the EEPROM range, min-jerk joint moves, straight top-down lines, gripper with a torque limit, hold / recover, shutdown via `rest`.
- [x] `pick`: workspace polygon check, the whole path planned (IK) before moving → `TargetRejected` without motion; descend (stopping on cloth is fine), close, measure the opening, go back up.
- [x] Pose teaching tool: `python -m sorter.arm.teach` (`free`, `lock`, `save`, `go`, `grip`, `show`).
- [x] Unit tests on the mock bus.
- [x] Joint directions verified on the hardware: all `signs` +1 (LeRobot convention); FK matched the camera view and the rig photo.
- [ ] Poses: `rest`, `home`, `look_box`, `look_bg`, `place_bg` set from IK on the rig (2026-09-26); bins still missing. Zones come from the zone setup tool (block 2).
- [ ] Tune grasp depth, approach, gripper force and `empty_below` on real clothes. Record the working values in config and here.

## Depends on / Unblocks

- Depends on: 0 (interface). Hardware.
- Unblocks: 2 (FK, look poses), integration.

## Acceptance criteria

- All named poses reachable repeatedly. The pick / place / drop cycle works on real clothes.
- A target outside the zone workspace is rejected without motion. Z never goes below `z_floor_mm`.
- `hold()` from another thread stops motion within one control tick, and the arm stays up. `recover()` resumes.

## Notes & risks

- **Reach:** tool straight down only ≈ 10–30 cm from the base and up to z ≈ 80 mm; above that the tool tilts (≤ `grasp_max_tilt_deg`). Keep `approach_mm` low enough.
- **Motors off = the arm falls.** Only at `rest` (shutdown) or in the setup tools after asking to hold the arm.
- The bus reads **5.2 V**. If the servos are the 7.4 V type, torque is reduced: heavy clothes may slip or the arm may sag.
- Servos are P-controlled: under gravity the arm sags, up to ≈ 15–20° at the shoulder with the arm stretched out and a T-shirt in the gripper at 5 V. After each strict motion the remaining error is added to the command (`sag_passes`, `sag_tol_deg`). Goals are streamed from the last command, so the sag doesn't accumulate. A correction can overshoot when a blocked motion suddenly frees (cloth pulled loose).
- **Lift height limits the clothes:** with the gripper down the fingertips reach ≈ 25 cm at most, so a hanging T-shirt (30–40 cm) can't clear a 13 cm box wall. The pile lies on the table (no box), or the items are small.
- No arm thread: a motion runs in the calling thread; `hold()` from another thread takes the bus lock, freezes the goals, and the motion raises `EStopped` at its next write.
- `rebot_b601/` belongs to the old B601 arm and is not used.

## Open questions

- Gripper force and `empty_below`: can the gripper opening tell "holding cloth" from "empty" for thin cloth? If not, set `empty_below: 0` so `likely_empty` is never true (the camera is the truth).

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: arm fixed (reBot B601-RS); `park()` replaced by `look(zone)`; `pick(target, zone) -> PickResult`; `hold` / `recover` instead of e-stop (D-006, D-009).
- 2026-09-25 (block 0): skeleton ready. See architecture.md → Wiring.
- 2026-09-25: arm model corrected to the B601-RS (D-011).
- 2026-09-25: arm replaced by the SO-101 (D-014): new driver, kinematics, controller and teaching tool; `joints()` returns 5 values; config keys changed (`tcp_offset_mm` → `tcp_extend_mm`, no SDK path).
- 2026-09-26: first real picks: a T-shirt taken from a pile on the table and put down 25 cm to the left. Sag correction added; `roll_deg` 93.9 and `table_z_mm` −20 (table ≈ 17 mm below the URDF base origin) in `default.yaml`.
