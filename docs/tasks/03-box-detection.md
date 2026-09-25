# Block 3: Box Detection

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** _TBD in block 0_ (box detector module, its tests and test data)

## Goal

Given a color + depth frame, choose a grasp point in the mixed box, or report that the box is empty.

## Scope

- [ ] Segment cloth inside the box ROI (depth above the box floor, and/or difference from the empty-box reference).
- [ ] Choose a grasp point. Baseline: the **highest point of the pile** (closest to the camera), shifted away from the box walls, preferring spots deep inside a cloth region rather than edges.
- [ ] Provide grasp depth from the depth map (how far to descend), per the contract.
- [ ] Detect an empty box: cloth coverage or pile height below a threshold.
- [ ] Accept failed-grasp feedback so retries try a different spot.
- [ ] Debug output for the dashboard: mask, candidate points.
- [ ] Tests on recorded frames: empty box, a few pile configurations.

## Out of scope

Pixel→arm conversion (block 2). Arm motion (block 5).

## Depends on / Unblocks

- Depends on: 0 (contract, stubs). Real frames from 1; start with phone photos or recordings.
- Unblocks: integration.

## Acceptance criteria

- On recorded frames: never returns a point outside the box ROI or on the box walls; detects an empty box with no false "empty" on a non-empty box.
- On the real rig: grasp success rate measured and noted here (target ≥ 70% on the demo clothes).

## Notes & risks

- **Main project risk: grasping deformable cloth.** Do a manual grasp test early: hard-coded point, fixed depth. Iterate on gripper, depth, and approach before polishing detection.
- The arm may be in the frame. Detect only when the arm is at home or out of view.

## Open questions

- Gripper type (pinch / parallel / other)? It affects which grasp spot works best.

## Requests from other blocks

_None yet._

## Log

_Significant changes to this block's scope or contracts, one line each (date: what, why)._
