# Block 3: Box Detection

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** `src/sorter/box_detector/`, `tests/box_detector/`, `config/default.yaml` → `box_detector`

## Goal

Given a frame from the `look_box` pose, choose a grasp point in the box (pixels + surface depth), or report that the box is empty.

## Scope

- [ ] Implement `BoxDetector.detect(frame, avoid) -> BoxResult` per [architecture.md](../architecture.md). Stateless ([D-008](../decisions.md)).
- [ ] Segment cloth inside the box ROI (`views.box.roi`): depth above the box floor (a constant, since the look pose is fixed), and/or difference from an empty-box reference.
- [ ] Choose a grasp point. Baseline: the **highest point of the pile** (smallest depth), deep inside a cloth region rather than on an edge, at least the wall margin away from the box walls.
- [ ] `depth_mm`: robust surface depth at the point (median over a small window, ignoring zeros).
- [ ] Skip candidates within `avoid_radius_px` of any `avoid` point. Return `NO_GRASP` if cloth is present but no candidate is left.
- [ ] `EMPTY`: cloth coverage or pile height below a threshold. Also fill `coverage`.
- [ ] Overlay: mask, candidates, chosen point, avoided points.
- [ ] Tests on recorded observations: empty box, a few pile configurations, `avoid` handling.

## Out of scope

Pixel → arm conversion (block 2). Arm motion (block 5).

## Depends on / Unblocks

- Depends on: 0 (contract, stubs). Real frames from 1 (recorded from the look pose); start with handheld photos or recordings.
- Unblocks: integration.

## Acceptance criteria

- On recorded frames: never returns a point outside the ROI or within the wall margin; detects an empty box with no false "empty" on a non-empty box.
- On the real rig: grasp success rate measured and noted here (target ≥ 70% on the demo clothes).

## Notes & risks

- **Main project risk: grasping deformable cloth.** Do a manual grasp test early: hard-coded point, fixed depth. Iterate on gripper, depth, and approach before polishing detection.
- **Wall margin covers the camera too.** The camera sits on the wrist and descends with the gripper. Agree on the footprint with block 5.
- The gripper fingers are visible at fixed pixels. They are excluded by the ROI.

## Open questions

- Which grasp spot works best for the parallel gripper: the pile top, or a fold?

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: contract fixed: stateless `detect(frame, avoid)` with `BoxStatus` GRASP / EMPTY / NO_GRASP; frames come from the wrist camera at `look_box` (D-006, D-008).
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/box_detector/config.py` (placeholder). Real backend: `sorter/box_detector/backend.py` → `create(cfg) -> BoxDetector`. `avoid_radius_px` is already a field (the simulator uses it). See architecture.md → Wiring.
