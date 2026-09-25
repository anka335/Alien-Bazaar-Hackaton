# Architecture

> **Status: draft.** Interfaces and repo layout are defined in [block 0](tasks/00-contracts.md). Until then, this file lists the components and the questions each contract must answer. Replace the _TBD_ sections as they get defined, and keep this file the single source of truth for contracts between blocks.

## Components

| Component | Block | Responsibility |
| --- | --- | --- |
| Camera | 1 | Capture aligned color + depth frames. Always returns a fresh frame, never a stale buffered one |
| Calibration | 2 | Convert pixel (+ depth) coordinates into arm coordinates. The only place where this conversion happens |
| Box detector | 3 | Grasp point in the mixed box, or "box empty" |
| Color classifier | 4 | Item on background: color class, re-grasp point, or "background empty" |
| Arm controller | 5 | Named poses, `pick` / `place_on_background` / `drop_to_bin`, workspace limits, emergency stop |
| State machine | 6 | Main loop, failure handling, publishes system status |
| Dashboard | 7 | Live feed with overlays, state, counters, controls (start / pause / step / stop) |

## Data flow

```text
Camera ──frame──► Box detector ──pixel grasp──► Calibration ──arm point──► Arm controller
   │                                                                          ▲
   └──frame──► Color classifier ──color + pixel center──► Calibration ────────┘

State machine orchestrates all of the above and publishes status ──► Dashboard
Camera frames + status ──► Dashboard (overlays)
```

## Coordinate frames

- **Pixel frame:** image coordinates of the color stream, `(u, v)`, origin top-left. The depth stream must be aligned to the color stream.
- **Camera frame:** 3D in metres/mm, from pixel + depth via the camera intrinsics.
- **Arm frame:** 3D in mm in the arm base frame. This is what the arm controller accepts.
- Rule: **vision outputs the pixel frame only**. Calibration owns pixel/camera → arm.

## Contracts (_TBD in block 0_)

Each contract must specify: types, units, coordinate frame, what "nothing found" looks like, blocking or non-blocking behavior, and error handling.

- [ ] **Frame:** color image format, depth format/units, timestamp.
- [ ] **Box detector → state machine:** grasp point (pixel + depth?), confidence, "empty box" signal, feedback about failed grasps.
- [ ] **Color classifier → state machine:** color class enum, re-grasp point, "empty background" signal, debug info for the dashboard.
- [ ] **Calibration:** pixel (+depth) → arm point. File format of saved calibration.
- [ ] **Arm controller:** method list, blocking semantics, units, poses config, gripper control, e-stop.
- [ ] **State machine → dashboard:** status snapshot (state, counters, last detections, errors, log), control commands.
- [ ] **Config:** format, location, which keys belong to which block.

## Repo layout (_TBD in block 0_)

The directory ⇄ block ownership map goes here, so parallel agents don't edit the same files.
