# Block 7: Dashboard

**Status:** review · **Owner:** Softjey + Claude · **Branch:** `block/07-dashboard`
**Owned paths:** `src/sorter/dashboard/`, `tests/dashboard/`, `config/default.yaml` → `dashboard`

## Goal

A live view that makes the demo understandable to the audience and gives the operator control.

## Scope

- [x] FastAPI server and a single static page (no build step), with the HTTP API from [architecture.md](../architecture.md#dashboard-http-api-block-7). It depends only on the Hub.
- [x] Main panel: the last **decision frame** with the zone ROI and the overlay (mask, grasp point, color label) drawn server-side.
- [x] Small panel: live wrist camera feed. Optional: colorized depth (not done).
- [x] Current phase (big), next phase, mode, cycle time.
- [x] Item counters per bin.
- [x] Controls: start / pause / resume / step / stop / reset, and a big, always visible **HOLD** button.
- [x] Recent events and the error, if any.
- [x] Works on the simulator before hardware is ready.
- [ ] Optional: a fixed scene webcam for the audience (`dashboard.scene_camera`), since the wrist feed moves.

## Depends on / Unblocks

- Depends on: 0 (Hub, contracts, simulator).
- Unblocks: 8.

## Acceptance criteria

- Readable from 3 meters on a projector or laptop screen.
- The decision frame and its overlay always match.
- HOLD from the dashboard stops the arm, and it stays up (verified on the real rig).

## Notes & risks

- Keep the tech simple. MJPEG streams and a WebSocket for status are enough (D-012).
- Code: `server.py` (endpoints, JPEG cache, MJPEG, `/ws`), `render.py` (overlay drawing, caption strip, `PHASE_LABELS` shared with the page), `static/` (page). `index.html` is read when the app is created: restart the process after editing it; `app.js` and `style.css` are served live.
- Space or Esc anywhere on the page sends HOLD. The HOLD button is still the primary control; the keys are a backup for the operator.
- The page enables each button by the same rules as `StateMachine._apply`. If block 6 changes when a command applies, update `ENABLED` in `static/app.js`.
- `/ws` pushes at most `status_hz`, so on the sim with instant motions some phases are skipped in the headline. The decision frame is unaffected.
- Design: the front of a washing machine. A dark glass control panel (phase, program lights for the 8 steps of one item, readouts, round Hold button), the decision frame as a screen, the live feed in a porthole, bins as stacks of folded clothes. The program lights and the cloth colors are in `static/app.js`.
- Layout targets a 16:9 screen without scrolling (checked at 1280×800 and 1600×900); below 900 px it stacks and scrolls.
- Not yet verified: HOLD from the dashboard on the real rig (acceptance criterion), readability on the actual projector.

## Open questions

_None._

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: runs on the same laptop in the same process (D-005); main panel is the decision frame (wrist camera, D-006); e-stop is HOLD (D-009); HTTP API fixed.
- 2026-09-25: dashboard implemented (D-012). Contract changes: `create_app(hub, cfg.dashboard, views=cfg.views)`; `/api/status` and `/ws` add `now`; new `/snapshot/{decision,live}.jpg`; config keys `stream_fps`, `jpeg_quality`, `status_hz`. Added the `websockets` dependency.
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/dashboard/config.py` (placeholder). A placeholder server is in `sorter/dashboard/server.py` (`create_app(hub, cfg)`: page, `/api/status`, `/api/command`); replace it, keep the entry point. See architecture.md → Wiring.
