# Block 1: Setup & Camera

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** `src/sorter/camera/`, `tests/camera/`, `config/default.yaml` → `camera`, `config/rig.yaml` → `views`

## Goal

A fixed, repeatable physical rig with the camera rigidly mounted on the wrist, and a camera module that returns fresh, aligned color + depth frames.

## Scope

### Physical

- [ ] Place the arm, mixed-clothes box, background area (mid-gray, so both white and black clothes contrast), and 3 bins, all within reach with the gripper pointing down. Positions are fixed ([D-007](../decisions.md)).
- [ ] Fix everything to the table. Mark positions with tape.
- [ ] **Rigid camera mount on the wrist**, behind the gripper, looking along the gripper axis. It must not shift at all relative to the gripper. Route the cable along the arm so it can't snag in any pose.
- [ ] Dedicated lamp for stable lighting, placed so the arm's shadow doesn't fall on a zone at its look pose.
- [ ] Photos of the rig and the mount for the docs, so it can be rebuilt after transport.

### Software

- [ ] Camera module implementing the `Camera` contract: capture thread, `latest()`, `fresh()`, depth aligned to color, intrinsics.
- [ ] `fresh()` returns a frame whose exposure started after the call, never a stale buffered one.
- [ ] Lock auto exposure and white balance after warm-up, if the SDK allows.
- [ ] ROI tool: from each look pose, draw the zone polygon (excluding the gripper fingers) and save it to `config/rig.yaml` → `views.<zone>.roi`.
- [ ] Record tool: save observations (`sorter.core.io`) from the look poses into `data/datasets/<name>/`, so blocks 3 and 4 can work offline.

## Depends on / Unblocks

- Depends on: 0 (contracts). Look poses from 5 for ROIs and datasets. The physical part can start right away.
- Unblocks: 2, real frames for 3 and 4.

## Acceptance criteria

- The rig can be disassembled and reassembled to the same positions. The camera mount is rigid.
- `fresh()` verified: move an object, call `fresh()` immediately, and the object is in its new place.
- From each look pose, the whole zone is in the frame and has valid depth.
- A recorded dataset from the look poses: empty box, box with clothes, empty background, background with single items of each class, background with two items.

## Notes & risks

- **Minimum depth range.** For the D435i, the specified minimum-Z depends on resolution: approximately 17.5 cm at 640x480 (the configured profile) and 28 cm at 1280x720. Keep the nearest point, including the top of a full pile, beyond the applicable minimum plus a measured safety margin.
- The gripper fingers are at fixed pixels in every frame. Keep them out of the ROIs.
- Datasets taken by hand from other viewpoints are fine to start, but only look-pose recordings match runtime.

## Open questions

- Where does the full dataset live (not in git if large)?

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: camera moved from an overhead stand to the wrist (D-006); zones fixed (D-007); ROI and record tools added to scope.
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/camera/config.py` (placeholder). The real backend still needs to be implemented at `sorter/camera/backend.py` with `create(cfg) -> Camera`. See architecture.md → Wiring.
