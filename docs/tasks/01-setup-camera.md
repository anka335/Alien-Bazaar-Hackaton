# Block 1: Setup & Camera

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `feat/sim-3d-arm`
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

- [x] Camera module implementing the `Camera` contract: capture thread, `latest()`, `fresh()`, depth aligned to color, intrinsics (`RealSenseCamera`, `pyrealsense2`, the `camera` extra). Untested on the device.
- [x] `fresh()` returns a frame whose exposure started after the call, never a stale buffered one (by frame timestamp).
- [x] Lock auto exposure and white balance after warm-up, if the SDK allows (`lock_exposure`, or fixed `exposure_us` / `white_balance_k`).
- [ ] ROI tool: from each look pose (for the sim layout `python -m sorter.sim.layout --write` computes them from the physics camera), draw the zone polygon (excluding the gripper fingers) and save it to `config/rig.yaml` → `views.<zone>.roi`.
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
- **macOS.** `pyrealsense2` comes from the community `pyrealsense2-macosx` wheel, pinned to 2.54.2: as root, 2.56.5 segfaults while opening the IMU (HID) on macOS 26. The macOS UVC driver (UVCAssistant, a user process) holds the video interfaces and takes them back ~30 ms after libusb captures them, while librealsense opens and closes them during start: `failed to set power state`. Run as root; the camera pauses UVCAssistant while it runs ([D-019](../decisions.md)). A newer libusb (1.0.30) does not help. About 1 start in 5 still gets no frames, hence `camera.start_attempts`. If the process dies hard: `sudo killall -CONT UVCAssistant`.

## Open questions

- Where does the full dataset live (not in git if large)?

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: camera moved from an overhead stand to the wrist (D-006); zones fixed (D-007); ROI and record tools added to scope.
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/camera/config.py` (placeholder). The real backend still needs to be implemented at `sorter/camera/backend.py` with `create(cfg) -> Camera`. See architecture.md → Wiring.
- 2026-09-26 (block 5): the look poses are computed for the table layout (D-015): the camera straight down from ~260 mm, which at 640x480 (fx 615) sees 271 x 203 mm, enough for the 240 x 180 mm tray and mat. It assumes the camera 140 mm behind the fingertips and 55 mm off-axis, looking along the gripper (`sim.camera_mount_mm`): tell block 5 the real mount. The top of a pile in the tray stays ~200 mm from the camera, above the D435i minimum range.
- 2026-09-26 (Softjey + Claude): `RealSenseCamera` in `src/sorter/camera/realsense.py` (not yet run on a D435i). The physics simulator (D-016) renders the same camera (pinhole at fx 615, D435i-like depth noise, no depth under 175 mm), so the rest of the loop runs on camera-like frames. Sim ROIs in `rig.yaml` come from the layout tool.
