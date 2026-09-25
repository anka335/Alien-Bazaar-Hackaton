# Block 2: Calibration

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** _TBD in block 0_ (calibration module, calibration tool)

## Goal

Convert camera observations into arm coordinates accurately enough to grasp clothes anywhere in the box and on the background.

## Scope

- [ ] Choose the method and record it in `docs/decisions.md`:
  - **3D (recommended with depth camera):** pixel + depth → 3D camera point via intrinsics, then a rigid camera→arm transform fitted from ≥4 point pairs (SVD/Kabsch). Gives X, Y and Z.
  - **Fallback, 2D homography:** pixel → arm XY on the table plane. Exact for one plane only.
- [ ] Calibration tool: for each reference point, move the arm tip onto a marker, read the arm position, mark the same point in the image. Save the result to a file.
- [ ] Load calibration at startup; conversion API per the block 0 contract.
- [ ] Verify accuracy with held-out points across the whole working area (box corners, box center, background).

## Depends on / Unblocks

- Depends on: 1 (fixed rig, camera), 5 (move the arm to points and read positions), 0 (contract).
- Unblocks: integration.

## Acceptance criteria

- Error on held-out points ≤ _N_ mm across box and background. Set N from the gripper's tolerance, e.g. 10 mm.
- Recalibration after a rig bump takes ≤ 10 minutes, and the steps are written in this file.

## Notes & risks

- Any shift of the camera or arm invalidates the calibration. Fix the mounts well.
- Calibration results are machine/rig-specific and are not committed (see `AGENTS.md`).

## Open questions

- Can the arm SDK report the current tool position? If not, points must be commanded, not jogged.

## Requests from other blocks

_None yet._

## Log

_Significant changes to this block's scope or contracts, one line each (date: what, why)._
