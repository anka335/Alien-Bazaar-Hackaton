# Alien Bazaar Hackathon — Sock-Sorting Rover

A robotic arm on a rover collects socks from the floor and sorts them by color.

**Load:** the rover stops next to socks. The arm looks at the floor with the depth camera on its wrist, classifies each sock as **light**, **dark** or **colored**, picks it up and drops it into the matching compartment of the 3-compartment cargo box on the rover. **Unload:** at a station the arm empties each compartment into its laundry bin. A live dashboard shows the camera, the decisions, a 3D view and the counters.

Everything is built and accepted on a MuJoCo simulator first ([D-032](docs/decisions.md)); the real rover and arm come after.

## Hardware

| Component | Details |
| --- | --- |
| Rover | built and driven by others; the arm stands on its deck, 200 mm above the floor |
| Robot arm | Seeed reBot Arm B601-RS (RobStride motors): 6 DoF + parallel gripper, driven through [`rebot_b601/`](rebot_b601/README.md) |
| Camera | Intel RealSense D435i RGB-D on the arm's wrist (eye-in-hand), USB 3 |
| Cargo box | on the deck, left of the arm: 3 compartments (light, dark, colored) |
| Unload station | 3 laundry bins on the floor, right of the rover |

## Project plan

Stages ([docs/plan.md](docs/plan.md)): [0 Preparation](docs/rover/0-preparation.md) (done: the shared base); then [A Loading](docs/rover/a-loading.md), [B Unloading](docs/rover/b-unloading.md), [F Far detection](docs/rover/f-far-detection.md) and [N Navigation](docs/rover/n-navigation.md) in parallel, one agent each; then [C Full mission](docs/rover/c-mission.md) and [D Sim-to-real](docs/rover/d-sim-to-real.md); then the hardware.

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

- The scene is a shared base (floor, rover, deck, cargo box, arm) plus one scene file per stage: `src/sorter/sim/scenes/load/` (socks on the floor) and `.../unload/` (the laundry bins, socks in the compartments); `sim.scenes` picks which ([D-034](docs/decisions.md)).
- Socks are cloth that falls, folds and hangs from the gripper. Cloth is expensive: `sim.realtime: 0` runs as fast as the CPU allows (3 socks ≈ 2.6× real time).
- The floor detector uses the render's segmentation instead of SAM3 (`sim.use_sam3: true` to call the service). `sim.miss_prob` makes grasps miss on purpose.

**Rover layout** (`sim.layout` in `config/default.yaml`, arm base at the origin on the deck, +x forward, +y left, mm; placeholders until the rover is measured): the floor at z −200; the cargo box inside x −260..60, y 130..310, walls 50 mm; the floor view (the floor pick zone) 280 × 240 around (300, 0); the laundry bins (220 mm, 150 high) at (−140, −330), (100, −330), (340, −300). The arm's joint 1 turns ±145° and the gripper held down reaches ~100 mm above the deck, which is why the box is beside the arm and the bins are low. After changing the layout recompute the rig: `uv run python -m sorter.sim.layout --write` writes the poses, zones, ROIs and the arm's keep-out into `config/rig.yaml` and checks every pick and move (it must report 0 problems).

## Rover navigation sim

The Leo Rover 1.9 with an OAK-D in its own MuJoCo world, steered by camera-only commands ([D-037](docs/decisions.md), stage N).

- The Rover tab: `/rover` in the dashboard, or on its own `uv run python -m sorter.nav serve` → http://127.0.0.1:8010/rover (build the front end first). Click a pixel of the OAK-D view, then Face / Go to pixel; Forward, Turn, Scan, Seek sock, Clearance, arrow keys nudge, Esc stops; Approach sock runs the algorithm (`classic`, `sam3`, `seg` = sim oracle).
- One command per process, an episode in a directory: `uv run python -m sorter.nav new runs/e1 --scenario easy --seed 0`, then `... do runs/e1 go_to_pixel 220 109`, `... detect runs/e1 --detector classic|sam3|seg`, `... depth runs/e1 U V`, `... finish runs/e1` (the ground-truth score). Frames land in `runs/e1/frames/` (`NNN_grid.png` has a pixel grid and the goal zone).
- The approach algorithm: `uv run python -m sorter.nav auto --scenario clutter --seed 3 --detector classic`; the benchmark: `uv run python -m sorter.nav bench --scenarios easy,side,behind --seeds 0-9 --jobs 4` (a worker needs ~0.3 GB; `bench` caps the workers by free memory).
- `python -m sorter.nav commands` lists the commands, `scenarios` the presets. Config: `nav` (`leo`, `camera`, `goal`, `real`, …; `--set camera.pitch_deg=30` on the CLI). `sam3` needs the SAM3 key (below).

### Rover navigation on the real Leo

The same commands drive the real rover: `sorter.nav.real_leo` talks to the Leo over rosbridge (`/cmd_vel` out, `/merged_odom` in; no ROS install needed), `sorter.nav.real_oakd` reads the OAK-D through depthai v3 (RGB 640×480 + stereo depth aligned to it, extended disparity).

1. On the machine the OAK-D is plugged into (the rover's Raspberry Pi, USB 3): `uv sync --extra nav-hw` (installs depthai).
2. rosbridge on the rover: LeoOS runs it for its web UI (port 9090); if not, `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` there.
3. Check: `uv run python -m sorter.nav hw-check` (on the rover; from a laptop add `--rosbridge ws://10.0.0.1:9090`). It saves `data/nav_hw/hw_rgb.png` and `hw_depth.png`, compares the depth of the floor with the mount in `nav.camera` (fix `mount_xyz_m` / `pitch_deg` if they disagree) and checks `/merged_odom`. `--move` also turns 10° and back: clear space first.
4. Drive: `uv run python -m sorter.nav serve --real --host 0.0.0.0` on the rover and open `http://<rover-ip>:8010/rover` from a laptop (the rover's Wi-Fi: `http://10.0.0.1:8010/rover`), or one command per process: `... real look runs/r1`, `... real do runs/r1 go_to_pixel 320 300`, `... real detect runs/r1 --detector sam3`, `... real auto --detector sam3 --out runs/r2`.
5. Safety: speeds are capped by `nav.real.max_linear_mps` (0.25) and `max_angular_rps` (0.8); a twist is sent every control tick, so if the process dies the firmware stops the rover within 0.5 s; `forward` stops for obstacles in the depth image (the Leo has no bumper). Stop: Esc / Stop in the tab, Ctrl+C on the CLI.
6. The mount: OAK-D on the front of the top plate, 25° down (`nav.camera.mount_xyz_m`, `pitch_deg`); keep depth mode `extended` (normal mode has no depth closer than ~0.7 m). Measure the real mount and put it in `config/local.yaml` → `nav.camera`.

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
