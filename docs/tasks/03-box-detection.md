# Block 3: Box Detection

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `block/so101-integration`
**Owned paths:** `src/sorter/box_detector/`, `tests/box_detector/`, `config/default.yaml` → `box_detector`

## Goal

Given a frame from the `look_box` pose, choose a grasp point in the box (pixels), or report that the box is empty.

## Scope

- [x] `SamBoxDetector.detect(frame, avoid) -> BoxResult` ([D-015](../decisions.md)), stateless ([D-008](../decisions.md)): SAM3 masks (`color_classifier.sam`) inside the box ROI, unioned.
- [x] `EMPTY`: coverage below `empty_coverage`. `coverage` is filled.
- [x] Grasp point: the cloth pixel farthest from any cloth edge, at least `wall_margin_px` inside the ROI and `min_inset_px` inside the cloth, not within `avoid_radius_px` of an `avoid` point. `depth_mm` is `None` (no depth, D-014). `NO_GRASP` when cloth is present but no candidate is left.
- [x] Overlay: cloth mask, avoided points, chosen point, coverage text.
- [x] Unit tests with synthetic masks.
- [ ] Check on the real box: prompts and threshold on a pile, empty box, grasp success rate.

## Out of scope

Pixel → arm conversion (block 2). Arm motion (block 5).

## Depends on / Unblocks

- Depends on: 0 (contract), the SAM3 client of block 4, the box ROI from block 2's setup tool.
- Unblocks: integration.

## Acceptance criteria

- On recorded frames: never returns a point outside the ROI or within the wall margin; detects an empty box with no false "empty" on a non-empty box.
- On the real rig: grasp success rate measured and noted here (target ≥ 70 % on the demo clothes).

## Notes & risks

- **Main project risk: grasping deformable cloth** with a small parallel gripper. The pick descends to the box floor (`zones.box.z_floor_mm`) and closes there.
- SAM3 may return one mask for the whole pile or one per item; both work, since only the union is used.
- The wall margin covers the camera too (it descends with the gripper).

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: contract fixed: stateless `detect(frame, avoid)` with `BoxStatus` GRASP / EMPTY / NO_GRASP (D-006, D-008).
- 2026-09-25 (block 0): skeleton ready. See architecture.md → Wiring.
- 2026-09-25 (block 6): run logs are written to `data/runs/<run_id>/`; usable as test data.
- 2026-09-25: no depth (D-014); `GraspPoint.depth_mm` may be `None`. Detector implemented on SAM3 masks (D-015).
