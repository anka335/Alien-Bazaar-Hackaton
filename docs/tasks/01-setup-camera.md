# Block 1: Setup & Camera

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `block/so101-integration`
**Owned paths:** `src/sorter/camera/`, `tests/camera/`, `config/default.yaml` → `camera`, `config/rig.yaml` → `views`

## Goal

A fixed, repeatable physical rig around the SO-101, and a camera module that returns fresh frames from the wrist camera.

## Scope

### Physical

- [ ] Place the arm, mixed-clothes box, background area (mid-gray, so both white and black clothes contrast), and 3 bins, all within the top-down reach (≈ 10–30 cm from the base, [D-014](../decisions.md)). Positions are fixed ([D-007](../decisions.md)).
- [ ] Fix everything to the table. Mark positions with tape.
- [x] Camera on the wrist: the SO-101's UVC camera is already mounted on the gripper. Route its cable so it can't snag in any pose.
- [ ] Dedicated lamp for stable lighting, placed so the arm's shadow doesn't fall on a zone at its look pose (exposure can't be locked, see below).
- [ ] Photos of the rig for the docs, so it can be rebuilt after transport.

### Software

- [x] `UvcCamera` (`uvc.py`, `backend.create`): OpenCV capture thread, `latest()`, `fresh()` (skips `fresh_skip_frames` frames after the call), reopens a lost device. RGB only: `depth_mm = None`, approximate intrinsics from `hfov_deg`.
- [x] Probe tool: `python -m sorter.camera.probe` saves one snapshot per device index.
- [x] ROI: written per zone by the zone setup tool (block 2, `python -m sorter.calibration.setup`) from the touched zone corners; check the preview and cut the gripper fingers out by hand if needed.
- [ ] Record tool for datasets from the look poses (run logs of block 6 already record every decision frame; enough for now).

## Depends on / Unblocks

- Depends on: 0 (contracts). Look poses from 5.
- Unblocks: 2, real frames for 3 and 4.

## Acceptance criteria

- The rig can be disassembled and reassembled to the same positions.
- `fresh()` verified: move an object, call `fresh()` immediately, and the object is in its new place.
- From each look pose, the whole zone is in the frame, and the ROI excludes the gripper fingers.

## Notes & risks

- **Device index.** On macOS the OpenCV index of the wrist camera changes when devices are re-plugged (it was 0 with the built-in camera at 1; with the wrist camera unplugged, 0 is the built-in one). `camera.name` guards against that: the camera refuses to open if no camera with that name is plugged in. Re-run the probe if the live feed shows the wrong camera.
- **USB drops on motion.** The wrist cable dropped the link (or fell to 12 Mb/s) when the arm moved, until it was re-routed with slack and plugged into a USB 3 port. In-process reopening didn't work on macOS, hence the capture process.
- **No exposure / white balance lock** through OpenCV's AVFoundation backend. The lamp has to dominate the lighting; color thresholds are tuned under it.
- The gripper fingers are at fixed pixels in every look frame because `look()` sets the gripper to `arm.gripper.look`.

## Open questions

- Where does the full dataset live (not in git if large)?

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: camera moved from an overhead stand to the wrist (D-006); zones fixed (D-007); ROI and record tools added to scope.
- 2026-09-25 (block 0): skeleton ready. See architecture.md → Wiring.
- 2026-09-26: capture moved to a child process (reopen after USB drops); `camera.name` guard.
- 2026-09-25: arm replaced by the SO-101 with its own RGB wrist camera (D-014). `Frame.depth_mm` may be `None` (contract change). `UvcCamera` and the probe tool implemented; ROIs come from the zone setup tool.
