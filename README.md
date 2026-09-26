# Alien Bazaar Hackathon — Robotic Laundry Sorter

An autonomous robotic system that takes clothes from a mixed pile and sorts them by color.

A robotic arm picks items one by one from a box of mixed clothing, using a depth camera mounted on its wrist to locate a grasp point. Each item is placed on a uniform background area, where a color classifier determines whether it is **light**, **dark**, or **colored**. The arm then picks the item up again and drops it into the matching bin, repeating the cycle until the box is empty. A live dashboard shows the camera feed, detected grasp points and colors, the current system state, and item counts per bin.

This demo addresses the core challenge of automated laundry handling — picking and sorting deformable clothing items from a cluttered pile — as the first step toward a fully automated wash–dry–sort pipeline.

## Hardware

| Component | Details |
| --- | --- |
| Robot arm | Seeed reBot Arm B601-RS (RobStride motors): 6 DoF + parallel gripper, driven through [`rebot_b601/`](rebot_b601/README.md) (`motorbridge` over SocketCAN, own IK) |
| Camera | Intel RealSense D435i RGB-D, mounted on the arm's wrist (eye-in-hand), USB 3 |
| Work area | at fixed positions (the layout below): mixed-clothes box, uniform background area (mid-gray), 3 bins (light / dark / colored) |
| Lighting | dedicated lamp for stable lighting |

## How it works

```text
 ┌──────────┐  pick   ┌────────────┐ classify ┌────────────┐  pick   ┌──────┐
 │ Mixed box├────────►│ Background ├─────────►│ color known├────────►│ Bin  │
 └──────────┘         └────────────┘          └────────────┘         └──┬───┘
      ▲                                                                 │
      └──────────────── verify drop, repeat until box is empty ─────────┘
```

1. **Pick from box.** The arm looks at the box from a fixed pose → depth + color frame → grasp point in pixels → arm coordinates → pick.
2. **Place on background.** An item on the background proves the grasp worked. An empty background means a missed grasp.
3. **Classify.** Segment the item, classify it as light, dark, or colored, and find a re-grasp point.
4. **Pick from background → drop into bin.**
5. **Verify.** The next look at the background must show one item fewer before the bin counter goes up.

Every cycle starts by looking at the background, so missed grasps, double grasps, and failed drops are all handled by what the camera sees. Vision works in **pixel coordinates**. Conversion to arm coordinates happens in one place (calibration). This lets vision be developed on recorded frames without the arm.

## Project plan

**The setup is moving to a rover** ([D-032](docs/decisions.md)): the arm collects socks from the floor into a cargo box on the rover (load) and sorts them into 3 laundry bins at a station (unload). The work is tracked in three stages: [0 Preparation](docs/rover/0-preparation.md), then [A Loading](docs/rover/a-loading.md) and [B Unloading](docs/rover/b-unloading.md) in parallel, all on the simulator first. The table blocks below are the code base they reuse.

The work is split into blocks 0–8 that can be developed in parallel by different people or agents:

| # | Block | Task file |
| --- | --- | --- |
| 0 | Contracts & skeleton | [docs/tasks/00-contracts.md](docs/tasks/00-contracts.md) |
| 1 | Setup & camera | [docs/tasks/01-setup-camera.md](docs/tasks/01-setup-camera.md) |
| 2 | Calibration | [docs/tasks/02-calibration.md](docs/tasks/02-calibration.md) |
| 3 | Box detection | [docs/tasks/03-box-detection.md](docs/tasks/03-box-detection.md) |
| 4 | Color classification | [docs/tasks/04-color-classification.md](docs/tasks/04-color-classification.md) |
| 5 | Arm control | [docs/tasks/05-arm-control.md](docs/tasks/05-arm-control.md) |
| 6 | State machine | [docs/tasks/06-state-machine.md](docs/tasks/06-state-machine.md) |
| 7 | Dashboard | [docs/tasks/07-dashboard.md](docs/tasks/07-dashboard.md) |
| 8 | Demo preparation | [docs/tasks/08-demo.md](docs/tasks/08-demo.md) |

The dependency graph and the live status board are in [docs/plan.md](docs/plan.md).

## Documentation map

| File | What's inside |
| --- | --- |
| [docs/plan.md](docs/plan.md) | Blocks, dependencies, status board (who is doing what) |
| [docs/architecture.md](docs/architecture.md) | Components, data flow, coordinate frames, interfaces |
| [docs/decisions.md](docs/decisions.md) | Log of design decisions and why they were made |
| [docs/rover/](docs/rover/) | Rover stages: tasks, status, acceptance criteria |
| [docs/tasks/](docs/tasks/) | One file per block: scope, acceptance criteria, open questions |
| [AGENTS.md](AGENTS.md) | Rules for AI coding agents (and humans) working in this repo |

## Getting started

Requires [uv](https://docs.astral.sh/uv/) (it installs Python 3.11 and the dependencies itself) and, to build the dashboard, Node 20+.

```bash
uv sync                                         # install
(cd frontend && npm install && npm run build)   # build the dashboard (again after a front-end change)
uv run python -m sorter run --sim               # the whole loop on the simulated hardware
uv run pytest                                   # tests
```

`run --sim` starts the loop and the dashboard, an admin panel at http://127.0.0.1:8000 ([D-031](docs/decisions.md)). Tabs on top: **Auto**, **Manual**, **Calibrate** (each switches the operator mode) and **3D view**; the bar also has the arm's **speed** and **Hold**. On the Auto tab press Start, or pass `--autostart`. The mode changes between runs (Stop first; a setup mode is left when the arm stops moving); `--mode manual|calibrate` starts in another one. The red Hold button (or Space / Esc on the page) freezes the arm; Reset continues. Ctrl+C holds the arm and shuts down. To open the dashboard from another device, set `dashboard.host: 0.0.0.0` in `config/local.yaml`. Other flags: `--no-dashboard`, `--config-dir`, `-v`. Front-end development: `cd frontend && npm run dev` serves the panel with hot reload at http://127.0.0.1:5173 against a sorter running on :8000; `npm run typecheck` checks the types. The build goes to `src/sorter/dashboard/web/` (not in git); without it the server shows how to build it. Every run is logged to `data/runs/<run_id>/` (turn off with `state_machine.save_runs: false`).

**Manual mode** (the Manual tab; the state machine gets no commands; `uv run python -m sorter manual` starts in it): the wrist camera, the 3D view, and a remote: named poses, a **tour** through them one press at a time (box view first, then the mat view, the mat, the three bins, home), joint jog (1° / 5° / 15°), gripper open / close, Hold (Space / Esc) and Release hold. When a joint lags its setpoint (blocked, collision, weak motor) the arm stops and holds; a red banner names the joint with its commanded and measured angle, and **Clear fault** (after checking the arm) lets it move again without turning the motors off. **Save current** writes the arm's joints as the chosen pose into `config/rig.yaml` (re-teaching the poses on the real rig; commit it). Moves to and from a bin go via `home`. It runs before the hand-eye calibration: without `config/hand_eye.yaml` the real rig uses the nominal camera mount (`sim.camera_mount_mm`), for an auto run too, so calibrate first. Ctrl+C holds, goes to rest and turns the motors off.

**Camera calibration** (the Calibrate tab, [D-026](docs/decisions.md)), no printed board, a 4-step wizard: (1) **Marks**: *Move tip to M1…M6* brings the gripper tip 5 mm above each spot around the mat center; stick a small dark tape square (~1 cm) right under it (already taped: the visit records where the tip really is; kept in `data/calibration_marks.yaml`). (2) **Click**: *Move camera to view N*: the page finds and names the tape marks by itself; check that every circle sits on its square, and click any it missed: click the highlighted mark's tape square in the live image (the click snaps to the square's center), mark after mark; until 3 marks are clicked the circles are only a guess, so find the square by the map (*Not visible, skip*), for two or three views; the fit updates with each click (RMSE of a few mm is good). (3) **Calibrate** writes the camera mount to `config/hand_eye.yaml`, recomputes `look_box` / `look_bg` for it and writes them with the zone ROIs to `config/rig.yaml`, all used at once (commit both). (4) **Check**: preview each look pose. The circles on the image are where the mount expects the marks, the dashed outlines the box and the mat: they should sit on the real ones. The camera needs ~18 cm to the mat for depth; raise the arm if a click says no depth. On `--sim` the marks lie on the simulated mat in the Calibrate mode only, and the fit reports its error against the true mount.

**The simulator** simulates the hardware only, the arm and the wrist camera, in a MuJoCo physics scene ([D-021](docs/decisions.md)); calibration, box detection and color classification run their real code on the rendered RGB-D frames, and the arm runs the real `rebot_b601` control loop on simulated motors. So `run` without `--sim` runs the same code on the rig. Clothes are cloth that falls, folds and hangs from the gripper. `sim.realtime` sets the speed (1 = real time, `0` = as fast as the CPU allows); `sim.engine: kinematic` is the quick physics-free world the tests use. The color classifier uses the render's segmentation instead of SAM3 (`sim.use_sam3: true` to call the service). The Auto tab's main screen switches between the last decision frame and a **3D view** (also the 3D view tab): the arm, the table, the clothes, and what the wrist camera sees.

**Table layout** ([D-020](docs/decisions.md)), arm base at the origin, +x forward, +y left, set in `sim.layout` (`config/default.yaml`), sizes x × y: a low tray (inside 140 × 200 mm, walls 60 mm) centered at (300, 0), the gray mat (150 × 120 mm) at (180, 210), bins (180 mm, walls 150 mm) at (150, −330) light, (380, −250) dark, (430, 220) colored. The zones are this small because the rig's wrist camera sees no more from its look poses ([D-029](docs/decisions.md)); clothes must fit the mat. The wrist camera is fixed to link5 and looks along the gripper ([D-027](docs/decisions.md)). The arm reaches the tray and the mat only with this geometry (the gripper pointing down tops out at ~120 mm TCP height), so build the real table the same way. After changing the layout, recompute the poses and zones: `uv run python -m sorter.sim.layout --write` (it checks that every pick plans).

**Config** is in `config/`: `default.yaml` (all sections), `rig.yaml` (poses, zones, ROIs of the rig; poses and zones computed for the layout), `hand_eye.yaml` (calibration result), and your own `local.yaml` (gitignored, machine overrides). `backends` chooses `real` or `sim` per component, for example in `config/local.yaml`:

```yaml
backends:
  camera: real
```

**Camera** (block 1): `uv sync --extra camera` adds `pyrealsense2` (Linux / Windows; on macOS the community `pyrealsense2-macosx` build, pinned to 2.54.2). On macOS the system camera driver (UVCAssistant) holds the camera, so run as root with the venv's Python: `sudo .venv/bin/python -m sorter run --mode manual`. The camera then pauses UVCAssistant while it runs and resumes it on exit ([D-024](docs/decisions.md)); after a crash resume it by hand: `sudo killall -CONT UVCAssistant`. The start is retried `camera.start_attempts` times (the first frames sometimes never come). `camera.serial` picks one D435i (empty = the first found); exposure and white balance lock after `camera.warmup_frames` (`camera.exposure_us`, `camera.white_balance_k` to fix them).

**Hand-eye calibration with a board** (block 2, the alternative to the calibration page, [D-028](docs/decisions.md)): print the ChArUco board (`uv run python -m sorter.calibration.board`: 7 × 5 squares of 45 mm, 32 mm 6×6 ArUco markers; check the printed size), lay it flat under `look_bg` and set `calibration.board_z_mm` to its top's height above the table (default 16). Stop `manual` (the tool needs the arm and the camera), then `sudo uv run python -m sorter.calibration.hand_eye` (root on macOS for the camera): the arm visits `calibration.poses` views around `look_bg` and writes `config/hand_eye.yaml` (commit it). It prints the reprojection error (under ~1 px is good) and warns when the result is far from the nominal mount. Then start `manual` again and recompute the look poses for the new mount (the page's step 3 needs a fit from its own clicks, so through the API): `curl -XPOST localhost:8000/api/calibrate -H 'content-type: application/json' -d '{"action": "compute_look"}'`, then the same with `"save_look"` (writes `look_box` / `look_bg` and the ROIs to `config/rig.yaml`). `--sim` rehearses it on the simulator and reports the error against the true mount.

**Color classifier** (block 4) segments the background with a remote SAM3 service ([D-013](docs/decisions.md)). To use it (`backends.color_classifier: real`), put the API key in `config/local.yaml` (`color_classifier: {sam: {api_key: ...}}`) or in the `SAM3_API_KEY` env var. With `sim.use_sam3: true`, set `sam.prompts: [blob]` and `sam.threshold: 0.3`, since SAM3 doesn't see the rendered cloth as clothing. Tuning tool: `uv run python -m sorter.color_classifier.stats <observation.npz ...>` or `--sim 3` (`--prompts a,b`, `--threshold`); it prints the color stats of every item (`--save DIR` writes overlays).

**Real arm** (block 5, [D-019](docs/decisions.md)): `uv sync --extra hardware` adds `motorbridge`. `run` without `--sim` uses it; with `arm.dry_run: true` it first runs `rebot_b601`'s simulated motors (the same control loop and safety checks, no bus). The CAN setup (`can0`, zero calibration) is in [rebot_b601/README.md](rebot_b601/README.md). The arm's speed is `arm.speed_scale` (1.0 of `rebot_b601`'s joint speeds at start) up to `arm.max_speed_scale` (1.4, at most ~1.43: the motors' velocity limit), with a peak acceleration of `REBOT_JOINT_ACCEL` (4) ([D-033](docs/decisions.md)); the **Speed** slider in the dashboard's bar (`POST /api/speed`) changes it while running, until a restart.

**ROS 2 track:** `ros2_ws/` holds a separate ROS 2 Jazzy + MoveIt 2 cloth pick-and-place task (proposed, [D-014](docs/decisions.md)). Setup and run commands: [ros2_ws/README.md](ros2_ws/README.md).

## Working with AI agents

Every block is designed to be handed to an agent on its own. See [AGENTS.md](AGENTS.md) for the parallel workflow. In Claude Code:

```text
/start-block 04      # pick up a block: reads its task file, creates a branch, marks it in progress
/sync-docs           # before finishing: bring docs in line with the code changes
```
