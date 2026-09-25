# Block 2: Calibration

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** `src/sorter/calibration/`, `tests/calibration/`, `config/hand_eye.yaml`, `config/default.yaml` → `calibration`

## Goal

Convert camera observations into arm coordinates accurately enough to grasp clothes anywhere in the box and on the background.

## Scope

- [ ] Hand-eye calibration, eye-in-hand ([D-006](../decisions.md)): get `T_flange_cam`. Start from the Seeed script (`collect_handeye_eih.py`: ArUco marker, automatic poses, Tsai) and port or wrap it. Its output is in metres; convert to mm.
- [ ] Make sure the result is relative to the same flange frame (`end_link`) that `ArmController.ee_pose()` returns.
- [ ] Save to `config/hand_eye.yaml`: 4×4 in mm, `rmse_mm`, `method`, `camera_serial`, `created`. The file is committed ([D-007](../decisions.md)).
- [ ] Implement the `Calibration` contract: `cam_pose`, `to_arm`, `to_pixel`.
- [ ] Verification tool (touch test): from a look pose, click a point in the image (or detect a marker), move the TCP there with the arm, and measure the error. Cover box corners, box center, and background.

## Depends on / Unblocks

- Depends on: 1 (camera, rigid wrist mount), 5 (FK, moving the arm, TCP offset), 0 (contract).
- Unblocks: integration.

## Acceptance criteria

- Touch-test error ≤ _N_ mm over the box and the background, from the look poses. Set N from the gripper tolerance, e.g. 10 mm.
- Recalibration takes ≤ 15 minutes, and the steps are written in this file.

## Notes & risks

- The error budget includes FK accuracy, hand-eye accuracy, and depth noise. Depth error grows with distance, so the look poses should be as low as the camera's minimum range allows.
- Only a change of the camera mount invalidates the hand-eye result. Moving the rig doesn't, as long as the zones stay at their positions.

## Open questions

- Does the Seeed script run with our camera model as is?

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: method changed to eye-in-hand hand-eye calibration (D-006); API now `cam_pose` / `to_arm(obs, point)` / `to_pixel`; result committed in `config/hand_eye.yaml` (D-007).
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/calibration/config.py` (placeholder). Real backend: `sorter/calibration/backend.py` → `create(cfg) -> Calibration`. See architecture.md → Wiring.
