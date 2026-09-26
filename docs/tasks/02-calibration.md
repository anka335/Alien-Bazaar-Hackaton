# Block 2: Calibration

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `feat/sim-3d-arm`
**Owned paths:** `src/sorter/calibration/`, `tests/calibration/`, `config/hand_eye.yaml`, `config/default.yaml` → `calibration`

## Goal

Convert camera observations into arm coordinates accurately enough to grasp clothes anywhere in the box and on the background.

## Scope

- [x] Hand-eye calibration, eye-in-hand ([D-006](../decisions.md)): `python -m sorter.calibration.hand_eye`: a ChArUco board on the mat, automatic views around `look_bg`, Park & Martin in numpy (OpenCV 5 has no `calibrateHandEye` in Python). In mm.
- [x] Make sure the result is relative to the same frame that `ArmController.ee_pose()` returns (`link5`, the camera link, D-027).
- [x] Save to `config/hand_eye.yaml`: 4×4 in mm, `rmse_mm`, `method`, `camera_serial`, `created`. The file is committed ([D-007](../decisions.md)).
- [x] Implement the `Calibration` contract: `cam_pose`, `to_arm`, `to_pixel` (`HandEyeCalibration`).
- [ ] Verification tool (touch test): from a look pose, click a point in the image (or detect a marker), move the TCP there with the arm, and measure the error. Cover box corners, box center, and background.

## Depends on / Unblocks

- Depends on: 1 (camera, rigid wrist mount), 5 (FK, moving the arm, TCP offset), 0 (contract).
- Unblocks: integration.

## Acceptance criteria

- Touch-test error ≤ _N_ mm over the box and the background, from the look poses. Set N from the gripper tolerance, e.g. 10 mm.
- Recalibration takes ≤ 15 minutes, and the steps are written in this file.

## Notes & risks

- The error budget includes FK accuracy, hand-eye accuracy, and depth noise. Depth error grows with distance, so the look poses should be as low as the camera's minimum range allows.
- The calibration page (`/calibrate`, D-026) is the way without a board: tape marks, clicks, a rigid fit (`sorter.calibration.marks`). The marks lie on the mat plane, so clicks from 2–3 different views are needed for a good rotation.
- A poor fit on `/calibrate`: the page also shows each view's RMSE without the arm FK (`view_rmse`). Large in one view: a wrong label, bad depth on the tape or a tape off its spot; small in every view while the fit is poor: the FK pose differs between views (joint offsets, backlash). On the rig every clicked/detected frame (png, depth .npy, joints) and `clicks.json` go to `data/calibrate/<start time>/`.
- The camera is fixed to `link5`: joint 6 (wrist roll) doesn't turn it (D-027). On the rig (2026-09-26) the model with the camera on `link6` fit the clicks to 15.8 mm RMSE; on `link5` the same clicks fit to 1.9 mm. The rig camera is ~100° about its optical axis from the sim's nominal mount; the calibration finds any turn.
- On the rig the camera is ~100 mm out from the TCP, away from the base (2026-09-26 fit, 4.7 mm RMSE). With the gripper vertical the arm can't lift it above ~230 mm over a zone, so the zones were shrunk to what it sees whole (D-029): box 140 × 200 at (300, 0), mat 150 × 120. Bigger zones need a tilted look pose or a camera mounted closer to the gripper. The mark pattern is ±50 × ±35 mm to stay on the smaller mat.
- Over the mat (`look_bg`, off to the side of the arm) the image is turned ~50° against the mat, and the arm can't lift the camera high enough to see all of it: ~91% on the sim.
- Only a change of the camera mount invalidates the hand-eye result. Moving the rig doesn't, as long as the zones stay at their positions.

## Open questions

- Does the board detection hold up under the rig's lamp (glare on a laminated print)?
- `calibration.board_z_mm` goes straight into the camera's height in the result: measure it (board + paper) and change it if the board moves to another support.

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: method changed to eye-in-hand hand-eye calibration (D-006); API now `cam_pose` / `to_arm(obs, point)` / `to_pixel`; result committed in `config/hand_eye.yaml` (D-007).
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/calibration/config.py` (placeholder). Real backend: `sorter/calibration/backend.py` → `create(cfg) -> Calibration`. See architecture.md → Wiring.
- 2026-09-26 (block 5): the flange that `ee_pose()` returns is the URDF `link6` frame; FK in mm is `sorter.arm.kinematics.fk_flange(q)` (on `rebot_b601`, D-019). Compute the hand-eye result against it. The sim calibration now applies the sim camera mount in `cam_pose`.
- 2026-09-26 (Softjey + Claude): real backend `HandEyeCalibration` and the hand-eye tool. `--sim` runs it on the physics simulator with a rendered board and reports the error against the true mount. On the physics sim the calibration uses the exact sim mount, not `hand_eye.yaml`. Not yet run on the rig.
- 2026-09-26 (Softjey + Claude): the board tool is for the printed board (7 × 5, 45 mm squares, 32 mm DICT_6X6 markers) and refines Park's result by corner reprojection with the board flat at `calibration.board_z_mm` (D-028): on the sim 0.1 mm / 0.1° from the true mount, against ~17 mm for Park alone. The `/calibrate` page is unchanged.
- 2026-09-26 (Softjey + Claude): hand-eye from tape marks on the `/calibrate` page (D-026): `sorter.calibration.marks` (marks, `deproject`, `fit_mount` Kabsch, `project`); config key `calibration.marks_z_mm`. Result format unchanged (`method: marks (...)`).
- 2026-09-26 (Softjey + Claude): the camera is on `link5`, not the flange (D-027): `ee_pose()` is `T_base_link5`, the hand-eye key is `T_link5_cam` (was `T_flange_cam`; old files fail to load, recalibrate). The views no longer turn the wrist.
