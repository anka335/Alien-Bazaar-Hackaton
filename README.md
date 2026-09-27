# Alien Bazaar Hackathon — Sock-Sorting Rover

A robotic arm on a rover collects socks from the floor and sorts them by color.

**Load:** the rover stops next to socks. The arm looks at the floor with the depth camera on its wrist, classifies each sock as **light**, **dark** or **colored**, picks it up and drops it into the cargo box on the rover (one box for now; where the color sorting happens is open, [D-035](docs/decisions.md)). **Unload:** at a station the arm empties the box into the laundry bins. A live dashboard shows the camera, the decisions, a 3D view and the counters.

Everything is built and accepted on a MuJoCo simulator first ([D-032](docs/decisions.md)); the real rover and arm come after.

## Hardware

| Component | Details |
| --- | --- |
| Rover | built and driven by others; four wheels, the arm stands on its deck, ~160 mm above the floor |
| Robot arm | Seeed reBot Arm B601-RS (RobStride motors): 6 DoF + parallel gripper, driven through [`rebot_b601/`](rebot_b601/README.md) |
| Camera | Intel RealSense D435i RGB-D on the arm's wrist (eye-in-hand), USB 3 |
| Cargo box | one cardboard box, 150 × 150 × 60 mm, left of the arm and a bit behind, over the rear left wheel |
| Unload station | 3 laundry bins on the floor, right of the rover |

## Project plan

Three stages ([docs/plan.md](docs/plan.md)): [0 Preparation](docs/rover/0-preparation.md) (done: the shared base), then [A Loading](docs/rover/a-loading.md) and [B Unloading](docs/rover/b-unloading.md) in parallel, one agent each.

| File | What's inside |
| --- | --- |
| [docs/plan.md](docs/plan.md) | Stages and the status board |
| [docs/rover/](docs/rover/) | One brief per stage: goal, lane, tasks, known issues |
| [docs/architecture.md](docs/architecture.md) | Setup, components, loops, frames, contracts, simulator, who owns which path |
| [docs/decisions.md](docs/decisions.md) | Log of design decisions and why they were made |
| [AGENTS.md](AGENTS.md) | Rules for AI coding agents (and humans) working in this repo |

## Getting started

Requires [uv](https://docs.astral.sh/uv/) (it installs Python 3.11 and the dependencies) and, for the dashboard, Node 20+.

```bash
uv sync                                         # install
(cd frontend && npm install && npm run build)   # build the dashboard (again after a front-end change)
uv run python -m sorter run --sim               # load mode on the simulator; --mode unload for the other
uv run pytest                                   # tests
```

`run --sim` starts the state machine and the dashboard at <http://127.0.0.1:8000> ([D-031](docs/decisions.md)). The operator modes are `load`, `unload` (the state machine runs that loop; Start / Pause / Step / Stop / Reset) and the setup modes `manual`, `calibrate`; `--mode` picks the one to start in, `--autostart` presses Start. The mode changes between runs. Hold (the red button, Space / Esc) freezes the arm; Reset continues; Ctrl+C holds, goes to rest and turns the motors off. Other flags: `--no-dashboard`, `--config-dir`, `-v`. Every run is logged to `data/runs/<run_id>/` (`state_machine.save_runs: false` to turn off). The front end still shows the table-era "Auto" tab and 3D table until stage A updates it (A6).

## The simulator

`--sim` simulates the hardware only (the arm's motors and the wrist camera) in a MuJoCo scene ([D-021](docs/decisions.md)); calibration, the detectors and the loops run their real code on the rendered RGB-D frames, and the arm runs the real `rebot_b601` control loop. So `run` without `--sim` runs the same code on the rig.

- The scene is a shared base (floor, rover, deck, cargo box, arm) plus one scene file per stage: `src/sorter/sim/scenes/load/` (socks on the parquet floor around the rover) and `.../unload/` (the laundry bins, socks in the box); `sim.scenes` picks which ([D-034](docs/decisions.md)).
- Socks are cloth that falls, folds and hangs from the gripper. Cloth is expensive: `sim.realtime: 0` runs as fast as the CPU allows (3 socks ≈ 2.6× real time).
- The floor detector uses the render's segmentation instead of SAM3 (`sim.use_sam3: true` to call the service). `sim.miss_prob` makes grasps miss on purpose.

**Rover layout** (`sim.layout` in `config/default.yaml`, arm base at the origin on the deck, +x forward, +y left, mm; estimated from photos until the rover is measured, [D-035](docs/decisions.md)): the floor at z −160; the body 360 × 350 with Ø120 wheels at its corners; the electronics, power supply and power strip behind the arm; the cargo box inside 150 × 150 around (−50, 200), rim 25 above the deck; the floor view (the floor pick zone) 280 × 240 around (310, 0); the laundry bins (220 mm, 150 high) at (−140, −330), (100, −330), (340, −300). The arm's joint 1 turns ±145° and the gripper held down reaches ~100 mm above the deck, which is why the box is beside the arm, not behind it, and the bins are low. After changing the layout recompute the rig: `uv run python -m sorter.sim.layout --write` writes the poses, zones, ROIs and the arm's keep-out into `config/rig.yaml` and checks every pick and move (it must report 0 problems).

## Setup on the rig

**Manual mode** (`uv run python -m sorter manual`, or the Manual tab): the wrist camera, the 3D view, named poses and a tour through them (`look_floor`, `look_cargo`, the cargo and laundry drop poses, home), joint jog, gripper, Hold / Release, **Clear fault** after a blocked joint, and **Save current** to re-teach a pose into `config/rig.yaml` (commit it). Moves to and from a drop pose go via `home`. Without `config/hand_eye.yaml` it uses the nominal camera mount (`sim.camera_mount_mm`).

**Camera calibration** (the Calibrate tab, [D-026](docs/decisions.md)), no printed board: (1) the tip points at 6 spots on the floor view; stick a small dark tape square under each; (2) from 2–3 camera views the page finds and names the marks (click any it missed); (3) **Calibrate** writes the camera mount to `config/hand_eye.yaml` and recomputes `look_floor` / `look_cargo` with their ROIs into `config/rig.yaml` (commit both); (4) check each look pose. The camera needs ~18 cm to the floor for depth.

**With a printed board** ([D-028](docs/decisions.md)): `uv run python -m sorter.calibration.board` prints a ChArUco board (7 × 5 squares of 45 mm); lay it flat on the floor under `look_floor` and set `calibration.board_z_mm` to its top (floor −200 + thickness). Then `uv run python -m sorter.calibration.hand_eye` (`--sim` rehearses it) writes `config/hand_eye.yaml`.

**Camera** (`uv sync --extra camera` for `pyrealsense2`): on macOS run as root with the venv's Python (`sudo .venv/bin/python -m sorter run --mode manual`); the camera pauses the system UVC driver while it runs ([D-024](docs/decisions.md)); after a crash: `sudo killall -CONT UVCAssistant`. `camera.serial` picks one D435i.

**SAM3 segmentation** ([D-013](docs/decisions.md)): the floor detector's real backend calls the SAM3 service; put the API key in `config/local.yaml` (`color_classifier: {sam: {api_key: ...}}`) or the `SAM3_API_KEY` env var. Tuning tool: `uv run python -m sorter.color_classifier.stats <observation.npz ...>` or `--sim 3`.

**Real arm** ([D-019](docs/decisions.md)): `uv sync --extra hardware` adds `motorbridge`; `arm.dry_run: true` first runs `rebot_b601`'s simulated motors. CAN setup: [rebot_b601/README.md](rebot_b601/README.md). Speed: `arm.speed_scale` (1.0 at start) up to `arm.max_speed_scale` (1.4, at most ~1.43: the motors' velocity limit), also from the Speed slider at runtime ([D-030](docs/decisions.md), [D-033](docs/decisions.md)); start it low on the rig: the acceleration is higher than the arm has run so far.

**Config** is in `config/`: `default.yaml` (all sections), `rig.yaml` (computed: poses, zones, ROIs, the arm's floor limit and keep-out), `hand_eye.yaml` (the camera mount), and your own `local.yaml` (gitignored), e.g. `backends: {camera: real}`.

**ROS 2 track:** `ros2_ws/` holds a separate ROS 2 Jazzy + MoveIt 2 cloth task ([D-014](docs/decisions.md)), outside these stages: [ros2_ws/README.md](ros2_ws/README.md).

## Working with AI agents

Each stage is handed to one agent. See [AGENTS.md](AGENTS.md) for the parallel workflow. In Claude Code:

```text
/start-stage A       # pick up a stage: reads its brief, creates a branch, marks it in progress
/sync-docs           # before finishing: bring docs in line with the code changes
```
