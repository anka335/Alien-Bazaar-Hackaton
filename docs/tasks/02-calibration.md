# Block 2: Calibration

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `block/so101-integration`
**Owned paths:** `src/sorter/calibration/`, `tests/calibration/`, `config/calibration.yaml`, `config/default.yaml` → `calibration`

## Goal

Convert camera observations into arm coordinates accurately enough to grasp clothes anywhere in the box and on the background.

## Scope

- [x] Plane calibration ([D-014](../decisions.md)): one homography per zone, pixel at the look pose → arm XY on the zone's plane; `PlaneCalibration` implements `cam_pose` (informational), `to_arm`, `to_pixel`.
- [x] Markers: `python -m sorter.calibration.markers` writes a printable ArUco sheet (`DICT_4X4_50`, 40 mm).
- [x] Zone setup tool, `python -m sorter.calibration.setup <zone>`: look pose → detect markers → touch each marker center with the fingertips (motors off, arm held) → fit H → touch the zone corners → write `config/calibration.yaml` and the zone's ROI, workspace (corners shrunk by 35 mm box / 20 mm background) and `z_floor_mm` to `config/rig.yaml`. `snap` step: look image only.
- [x] Touch test: `... setup <zone> verify` moves the tip 15 mm above every visible marker.
- [ ] Calibrate both zones on the rig; record the RMSE and touch-test error here.

## Depends on / Unblocks

- Depends on: 1 (camera), 5 (FK, look poses), 0 (contract).
- Unblocks: integration.

## Acceptance criteria

- Touch-test error ≤ 10 mm over the box and the background, from the look poses.
- Recalibration of a zone takes ≤ 10 minutes (setup tool), steps in README → Real hardware.

## Notes & risks

- The homography is valid only at the zone's look pose and on its plane. Re-run the setup after re-teaching a look pose or moving a zone.
- **Pile parallax in the box:** points on top of the pile map as if on the box floor. The XY error grows with pile height and distance from the image center. If it matters, calibrate the box on a raised sheet (markers on a book at the typical pile height) and set `surface_offset_mm` accordingly.
- Lens distortion is not modeled; use markers spread over the whole zone. A large RMSE (> 5 mm) at the edges means distortion matters.
- The error budget includes FK accuracy (servo backlash, sag under load), the touch itself, and look-pose repeatability.

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: method changed to eye-in-hand hand-eye calibration (D-006); API now `cam_pose` / `to_arm(obs, point)` / `to_pixel`; result committed (D-007).
- 2026-09-25 (block 0): skeleton ready. See architecture.md → Wiring.
- 2026-09-25: no depth on the SO-101 camera (D-014): hand-eye replaced by per-zone plane homographies in `config/calibration.yaml` (was `hand_eye.yaml`). `to_arm` no longer needs depth or `T_base_cam`.
