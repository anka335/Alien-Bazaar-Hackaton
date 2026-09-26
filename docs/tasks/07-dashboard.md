# Block 7: Dashboard

**Status:** review · **Owner:** Softjey + Claude · **Branch:** `block/07-dashboard`
**Owned paths:** `src/sorter/dashboard/`, `frontend/`, `tests/dashboard/`, `config/default.yaml` → `dashboard`

## Goal

A live view that makes the demo understandable to the audience and gives the operator control.

## Scope

- [x] FastAPI server and one admin panel (React, D-026), with the HTTP API from [architecture.md](../architecture.md#dashboard-http-api-block-7). It depends only on the Hub.
- [x] Main panel: the last **decision frame** with the zone ROI and the overlay (mask, grasp point, color label) drawn server-side.
- [x] Small panel: live wrist camera feed. Optional: colorized depth (not done).
- [x] Current phase (big), next phase, mode, cycle time.
- [x] Item counters per bin.
- [x] Controls: start / pause / resume / step / stop / reset, and a big, always visible **HOLD** button.
- [x] Recent events and the error, if any.
- [x] Works on the simulator before hardware is ready.
- [x] Camera calibration tab (calibrate mode, D-021): tape marks, clicks on the live image with an overlay, the camera mount, look poses and ROIs into `rig.yaml`.
- [x] 3D view (3D view tab, three.js, the arm's CAD meshes from `rebot_b601`): the arm posed from FK, the table layout, pick workspaces, the clothes (sim), and what the wrist camera sees. The main screen switches between it and the decision frame.
- [x] One admin panel (D-026): tabs on top switched without a reload, the Auto / Manual / Calibrate tabs switch the operator mode (one process); arm speed slider; Hold on every tab.
- [ ] Optional: a fixed scene webcam for the audience (`dashboard.scene_camera`), since the wrist feed moves.

## Depends on / Unblocks

- Depends on: 0 (Hub, contracts, simulator).
- Unblocks: 8.

## Acceptance criteria

- Readable from 3 meters on a projector or laptop screen.
- The decision frame and its overlay always match.
- HOLD from the dashboard stops the arm, and it stays up (verified on the real rig).

## Notes & risks

- MJPEG streams and a WebSocket for status (D-012); the page is React + TypeScript built with Vite (D-026).
- Code: `server.py` (endpoints, JPEG cache, MJPEG, `/ws`, serves the build), `modes.py` (`ModeSwitch`), `manual.py`, `calibrate.py`, `twin.py`, `render.py` (overlay drawing, caption strip, `PHASE_LABELS`, sent to the page by `/api/meta`). The front end is `frontend/src/`: `sorter.tsx` (the shared status stream and actions), `components/TopBar.tsx` (tabs that switch the mode, speed, Hold), `pages/*` (one per tab), `twin/scene.ts` (the three.js scene), `styles/`.
- Front-end workflow: `cd frontend && npm install && npm run build` writes `src/sorter/dashboard/web/` (not in git; the server serves it, a plain reload shows a new build). `npm run dev` serves the panel with hot reload on :5173 and proxies the API to a sorter on :8000. `npm run typecheck` checks the types.
- The 3D view loads the arm's meshes (~36 MB) when first shown; the Auto tab shows it only when its 3D view is picked.
- Space or Esc anywhere sends HOLD. The HOLD button is still the primary control; the keys are a backup for the operator.
- The Auto tab enables each button by the same rules as `StateMachine._apply`. If block 6 changes when a command applies, update `COMMANDS` in `frontend/src/pages/AutoPage.tsx`.
- `/ws` pushes at most `status_hz`, so on the sim with instant motions some phases are skipped in the headline. The decision frame is unaffected.
- Design: the front of a washing machine. A dark glass bar (tabs / modes, speed, Hold) and control panel (phase, program lights for the 8 steps of one item, readouts), the decision frame as a screen, the live feed in a porthole, bins as stacks of folded clothes. The program lights and the cloth colors are in `frontend/src/pages/AutoPage.tsx`.
- Layout targets a 16:9 screen without scrolling (checked at 1280×800 and 1600×900); below 900 px it stacks and scrolls.
- Checked on the sim in headless Chrome at 1600×950, 1280×950 and 420 px wide. Not yet verified: HOLD from the dashboard on the real rig (acceptance criterion), readability on the actual projector.

## Open questions

_None._

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: runs on the same laptop in the same process (D-005); main panel is the decision frame (wrist camera, D-006); e-stop is HOLD (D-009); HTTP API fixed.
- 2026-09-25: dashboard implemented (D-012). Contract changes: `create_app(hub, cfg.dashboard, views=cfg.views)`; `/api/status` and `/ws` add `now`; new `/snapshot/{decision,live}.jpg`; config keys `stream_fps`, `jpeg_quality`, `status_hz`. Added the `websockets` dependency.
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/dashboard/config.py` (placeholder). A placeholder server is in `sorter/dashboard/server.py` (`create_app(hub, cfg)`: page, `/api/status`, `/api/command`); replace it, keep the entry point. See architecture.md → Wiring.
- 2026-09-26 (Softjey + Claude): setup mode (D-018): `/manual` page, `GET/POST /api/manual`, `create_app(..., manual=)`, `sorter/dashboard/manual.py` (`ManualControl`); `/` redirects to `/manual` in that mode.
- 2026-09-26 (Softjey + Claude): `/api/manual` state has `fault`, new action `clear_fault` (D-020); the page shows a fault banner with Clear fault. The page switcher greys out the page of the mode that isn't running.
- 2026-09-26 (block 5): 3D view added. Contract changes: `Hub(..., twin=)` / `hub.twin()` (`TwinSource`), new endpoints `/twin`, `/api/twin/layout`, `/api/twin/state`, `/twin-assets/...` (architecture.md → Dashboard HTTP API).
- 2026-09-26 (Softjey + Claude): `/calibrate` page and `GET/POST /api/calibrate` (D-021); `create_app(..., calibrate=CalibrateControl)`; `ManualControl.run()` / `.busy` let other pages run motions through it.
- 2026-09-26 (Softjey + Claude): speed control (D-025). Contract changes: `Hub(..., speed=)` / `hub.speed()` (`SpeedControl`), new `GET/POST /api/speed`, `speed` in `/api/status` and `/ws`.
- 2026-09-26 (Softjey + Claude): one React admin panel and runtime operator modes (D-026). Contract changes: `Hub(..., mode=)`, `hub.mode()` / `hub.set_mode()` (`OperatorMode`, `WrongMode`), run commands 409 outside auto; new `GET /api/meta`, `GET/POST /api/mode`, `operator` in `/api/status` and `/ws`; `/api/manual` and `/api/calibrate` 409 outside their modes; pages `/auto`, `/manual`, `/calibrate`, `/3d` serve the panel, `/twin` is gone; `create_app(..., modes=, web_dir=)`; `run --mode`, `manual` = `run --mode manual`, `run_manual` is gone. The speed slider is in the bar.
