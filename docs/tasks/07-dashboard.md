# Block 7: Dashboard

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** `src/sorter/dashboard/`, `tests/dashboard/`, `config/default.yaml` → `dashboard`

## Goal

A live view that makes the demo understandable to the audience and gives the operator control.

## Scope

- [ ] FastAPI server and a single static page (no build step), with the HTTP API from [architecture.md](../architecture.md#dashboard-http-api-block-7). It depends only on the Hub.
- [ ] Main panel: the last **decision frame** with the zone ROI and the overlay (mask, grasp point, color label) drawn server-side.
- [ ] Small panel: live wrist camera feed. Optional: colorized depth.
- [ ] Current phase (big), next phase, mode, cycle time.
- [ ] Item counters per bin.
- [ ] Controls: start / pause / resume / step / stop / reset, and a big, always visible **HOLD** button.
- [ ] Recent events and the error, if any.
- [ ] Works on the simulator before hardware is ready.
- [ ] Optional: a fixed scene webcam for the audience (`dashboard.scene_camera`), since the wrist feed moves.

## Depends on / Unblocks

- Depends on: 0 (Hub, contracts, simulator).
- Unblocks: 8.

## Acceptance criteria

- Readable from 3 meters on a projector or laptop screen.
- The decision frame and its overlay always match.
- HOLD from the dashboard stops the arm, and it stays up (verified on the real rig).

## Notes & risks

- Keep the tech simple. MJPEG streams and a WebSocket for status are enough.

## Open questions

_None._

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: runs on the same laptop in the same process (D-005); main panel is the decision frame (wrist camera, D-006); e-stop is HOLD (D-009); HTTP API fixed.
