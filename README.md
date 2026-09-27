# Alien Bazaar Hackathon — Sock-Sorting Rover

A robotic arm on a rover collects socks from the floor and sorts them by color.

**Load:** the rover stops next to socks. The arm looks at the floor with the depth camera on its wrist, classifies each sock as **light**, **dark** or **colored**, picks it up and drops it into the cargo box on the rover (one box for now; where the color sorting happens is open, [D-040](docs/decisions.md)). **Unload:** at a station the arm empties the box into the laundry bins. A live dashboard shows the camera, the decisions, a 3D view and the counters.

Everything is built and accepted on a MuJoCo simulator first ([D-032](docs/decisions.md)); the real rover and arm come after.

## Hardware

| Component | Details |
| --- | --- |
| Rover | built and driven by others; four wheels, the arm stands on its deck, 200 mm above the floor |
| Robot arm | Seeed reBot Arm B601-RS (RobStride motors): 6 DoF + parallel gripper, driven through [`rebot_b601/`](rebot_b601/README.md) |
| Camera | Intel RealSense D435i RGB-D on the arm's wrist (eye-in-hand), USB 3 |
| Cargo box | one cardboard box, 190 × 190 × 75 mm (outside), right of the arm and a bit behind |
| Unload station | 3 laundry bins (boxes like the cargo box) on the floor in front of the rover |

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

`run --sim` starts the state machine and the dashboard at <http://127.0.0.1:8000> ([D-031](docs/decisions.md)). The operator modes are `load`, `unload` (the state machine runs that loop; Start / Pause / Step / Stop / Reset) and the setup modes `manual`, `calibrate`; `--mode` picks the one to start in, `--autostart` presses Start. The mode changes between runs. Hold (the red button, Space / Esc) freezes the arm; Reset continues; Ctrl+C holds, goes to rest and turns the motors off. Other flags: `--no-dashboard`, `--config-dir`, `-v`. Every run is logged to `data/runs/<run_id>/` (`state_machine.save_runs: false` to turn off). On the rig (without `--sim`; `--record` / `--no-record` to override) the whole session is recorded to `data/sessions/<stamp>/`: a DEBUG log, the arm's telemetry at 50 Hz (measured and commanded joints, torque, gripper), every arm and detector call, the decision frames and the wrist camera's video; Enter in the terminal drops a mark. The front end has the Load / Unload tabs but still the table-era phase strip and 3D view until stage A updates it (A6).

## The simulator

`--sim` simulates the hardware only (the arm's motors and the wrist camera) in a MuJoCo scene ([D-021](docs/decisions.md)); calibration, the detectors and the loops run their real code on the rendered RGB-D frames, and the arm runs the real `rebot_b601` control loop. So `run` without `--sim` runs the same code on the rig.

- The scene is a shared base (floor, rover, deck, cargo box, arm) plus one scene file per stage: `src/sorter/sim/scenes/load/` (socks on the parquet floor around the rover) and `.../unload/` (the laundry bins, socks in the box); `sim.scenes` picks which ([D-034](docs/decisions.md)).
- Socks are cloth that falls, folds and hangs from the gripper. Cloth is expensive: `sim.realtime: 0` runs as fast as the CPU allows (3 socks ≈ 2.6× real time).
- The sim's wrist camera sits where the real one was calibrated (`config/hand_eye.yaml`); after a new calibration run `python -m sorter.sim.layout --write`.
- Watch one load run: `uv run mjpython -m sorter.sim.scenes.load.watch [--seed 3] [--socks 4]` (live in the MuJoCo viewer; `mjpython` on macOS), or `uv run python -m sorter.sim.scenes.load.watch --no-viewer --record run.mp4` (a video). The load benchmark: `uv run python -m sorter.sim.scenes.load.bench -n 50 --workers 3` (report in `data/bench/`).
- The floor detector uses the render's segmentation instead of SAM3 (`sim.use_sam3: true` to call the service). `sim.miss_prob` makes grasps miss on purpose.

**Unload** (stage B, [D-043](docs/decisions.md)): the socks piled in the cargo box, the three bins in front of the rover off their places by seed (parking); the loop finds everything with the wrist camera.

```bash
uv run mjpython -m sorter.sim.scenes.unload.demo      # one scenario, live in the MuJoCo viewer (--no-viewer --dashboard: the browser)
uv run python -m sorter.sim.scenes.unload.preview     # render the scene (--view: the MuJoCo viewer)
uv run python -m sorter.sim.scenes.unload.bench -n 20 # seeded scenarios, judged by the sim; report in data/bench/
```

**Rover layout** (`sim.layout` in `config/default.yaml`, arm base at the origin on the deck, +x forward, +y left, mm; measured, [D-042](docs/decisions.md), except the equipment, the box's center and the station): the floor at z −200; the deck 300 × 185 with the arm at its front edge; the body 420 × 420 with wheels 130 wide at its corners; the electronics, power supply and power strip behind the arm; the cargo box inside 182 × 182 around (−50, −200) (right of the arm, [D-045](docs/decisions.md)), rim 34 above the deck; the floor view 280 × 240 around (310, 0); the laundry bins (190 mm, 75 high) at (290, 220) light, (290, 0) dark, (290, −220) colored. The arm's joint 1 turns ±145° and the gripper held down reaches ~100 mm above the deck, which is why the box is beside the arm, not behind it, and the bins are low. After changing the layout recompute the rig: `uv run python -m sorter.sim.layout --write` writes the poses, zones, ROIs and the arm's keep-out into `config/rig.yaml` and checks every pick and move (it must report 0 problems).

## Rover navigation sim

The Leo Rover 1.9 with an OAK-D in its own MuJoCo world, steered by camera-only commands ([D-046](docs/decisions.md), stage N).

- The Rover tab: `/rover` in the dashboard, or on its own `uv run python -m sorter.nav serve` → http://127.0.0.1:8010/rover (build the front end first). Click a pixel of the OAK-D view, then Face / Go to pixel; Forward, Turn, Scan, Seek sock, Clearance, arrow keys nudge, Esc stops; Approach sock runs the algorithm (`classic`, `sam3`, `seg` = sim oracle). Below it, the command panel has every command as a card: its parameters as fields (a click on the view fills `u`, `v`), a Run button and its last result.
- One command per process, an episode in a directory: `uv run python -m sorter.nav new runs/e1 --scenario easy --seed 0`, then `... do runs/e1 go_to_pixel 220 109`, `... detect runs/e1 --detector classic|sam3|seg`, `... depth runs/e1 U V`, `... finish runs/e1` (the ground-truth score). Frames land in `runs/e1/frames/` (`NNN_grid.png` has a pixel grid and the goal zone).
- The approach algorithm: `uv run python -m sorter.nav auto --scenario clutter --seed 3 --detector classic`; the benchmark: `uv run python -m sorter.nav bench --scenarios easy,side,behind --seeds 0-9 --jobs 4` (a worker needs ~0.3 GB; `bench` caps the workers by free memory).
- Search for socks and drive up to them, one after another: `uv run python -m sorter.nav hunt --scenario multi --socks 3` (sim) or `... hunt --real --socks 3 --out runs/h1` (the real Leo); in Python `sorter.nav.hunt.hunt_socks(real=True, max_socks=3, on_reached=...)`. A reached sock is remembered by its odometry position and skipped; `on_reached` is where the arm's pick goes.
- Fast: stop the front bumper a few cm before the nearest sock: the **RUN ROBOT** button in the Rover tab (stops 30 cm before the sock by default, gap in cm next to it; SAM3 on the real rover; `POST /api/nav/run {detector, gap_m}`), or `... hunt [--real] --gap 0.05`, or `sorter.nav.hunt.approach_sock(rover, detector, gap_m=0.05)`.
- `python -m sorter.nav commands` lists the commands, `scenarios` the presets. Config: `nav` (`leo`, `camera`, `goal`, `real`, …; `--set camera.pitch_deg=30` on the CLI). `sam3` needs the SAM3 key (below).

### Rover navigation on the real Leo

The same commands drive the real rover over rosbridge (LeoOS runs it on port 9090 for its web UI; no ROS install needed here): `sorter.nav.real_leo` sends `cmd_vel` every control tick and reads `merged_odom`, or, if that topic is silent, integrates the wheel encoders (`firmware/wheel_states`) and the IMU's gyro (`imu/data`) itself. Topic names are relative, so rosbridge resolves them in the rover's namespace (`/leo/` on ours).

**Snap Spectacles teleop of the base:** `scripts/spectacles_base.sh` runs the lens's bridge and its ngrok tunnel ([docs/rover/spectacles-teleop.md](docs/rover/spectacles-teleop.md)). To set up a machine, an agent follows [docs/rover/spectacles-teleop-setup.md](docs/rover/spectacles-teleop-setup.md).

The camera (`nav.real.camera`, `auto` tries them in this order): the OAK-D's ROS driver on the rover over rosbridge (`/oak/rgb/image_raw/compressed`, `/oak/stereo/image_raw/compressedDepth`, `/oak/rgb/camera_info`); an OAK-D plugged into this machine (depthai v3, `uv sync --extra nav-hw`); none (gray frames, no obstacle guard: drive with care).

1. Join the rover's Wi-Fi (the rover is 10.0.0.1; `nav.real.rosbridge_url` defaults to `ws://10.0.0.1:9090`).
2. An OAK-D on this machine's USB needs the Luxonis udev rule once: `echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules && sudo udevadm control --reload-rules && sudo udevadm trigger`, then replug it.
3. Check: `uv run python -m sorter.nav hw-check` saves `data/nav_hw/hw_rgb.png` / `hw_depth.png`, compares the floor's depth with the mount in `nav.camera` (fix `mount_xyz_m` / `pitch_deg` if they disagree) and checks the odometry. `--move` also turns 10° and back: clear space first.
4. Drive: `uv run python -m sorter.nav serve --real` → http://127.0.0.1:8010/rover: the command panel's buttons move the rover (the status line shows the rosbridge URL, the camera, the odometry source). **Reconnect** reconnects (after plugging the camera in). On the rover itself: `serve --real --host 0.0.0.0 --rosbridge ws://127.0.0.1:9090` and open `http://10.0.0.1:8010/rover`. One command per process: `... real look runs/r1`, `... real do runs/r1 forward 0.3`, `... real auto --detector sam3 --out runs/r2`.
- Cardboard boxes marked with AprilTags (36h11, ours: 14 13 12 left to right, 3.5 cm): in the Rover tab **Remember boxes in view** saves their layout to `nav.boxes.memory` (`data/nav_boxes.json`), **GO TO BOX** drives until the bumper is 15 cm from the target tag (13 by default; config `nav.boxes`). A hidden target is placed from a remembered neighbor. CLI: `python -m sorter.nav boxes remember | go [--target 13] [--stop 0.15]`.
5. Stay on the rover's Wi-Fi: other saved networks can take over mid-run (`nmcli con modify LeoRover-9a1f connection.autoconnect-priority 10`). Safety: speeds are capped by `nav.real.max_linear_mps` (0.25) and `max_angular_rps` (0.8), a command by `max_command_s` (10 s); a twist goes out every tick, so if this process dies the firmware stops the rover within 0.5 s; if the odometry moves opposite to the command for 0.4 s the rover stops with a fault (a sign error would otherwise run away). Stop: Esc / Stop in the tab, Ctrl+C on the CLI.
6. The mount: OAK-D on the front of the top plate, 25° down (`nav.camera.mount_xyz_m`, `pitch_deg`); keep depth mode `extended`. Measure the real mount and put it in `config/local.yaml` → `nav.camera`.

## Setup on the rig

**Manual mode** (`uv run python -m sorter manual`, or the Manual tab): the wrist camera, the 3D view, named poses and a tour through them (`look_floor`, `look_cargo`, the cargo and laundry drop poses, home), joint jog, gripper, Hold / Release, **Clear fault** after a blocked joint, and **Save current** to re-teach a pose into `config/rig.yaml` (commit it). Moves to and from a drop pose go via `home`. Without `config/hand_eye.yaml` it uses the nominal camera mount (`sim.camera_mount_mm`).

**Camera calibration** (the Calibrate tab, [D-026](docs/decisions.md)), no printed board: (1) the tip points at 6 spots on the floor view; stick a small dark tape square under each; (2) from 2–3 camera views the page finds and names the marks (click any it missed); (3) **Calibrate** writes the camera mount to `config/hand_eye.yaml` and recomputes `look_floor` / `look_cargo` with their ROIs into `config/rig.yaml` (commit both); (4) check each look pose. The camera needs ~18 cm to the floor for depth.

**With a printed board** ([D-028](docs/decisions.md)): `uv run python -m sorter.calibration.board` prints a ChArUco board (7 × 5 squares of 45 mm); lay it flat on the floor under `look_floor` and set `calibration.board_z_mm` to its top (floor −200 + thickness). Then `uv run python -m sorter.calibration.hand_eye` (`--sim` rehearses it) writes `config/hand_eye.yaml`.

**Camera** (`uv sync --extra camera` for `pyrealsense2`): on macOS run as root with the venv's Python (`sudo .venv/bin/python -m sorter run --mode manual`); the camera pauses the system UVC driver while it runs ([D-024](docs/decisions.md)); after a crash: `sudo killall -CONT UVCAssistant`. `camera.serial` picks one D435i.

**SAM3 segmentation** ([D-013](docs/decisions.md)): the floor detector's real backend calls the SAM3 service; put the API key in `config/local.yaml` (`color_classifier: {sam: {api_key: ...}}`) or the `SAM3_API_KEY` env var. Tuning tool: `uv run python -m sorter.color_classifier.stats <observation.npz ...>` or `--sim 3`.

**Real arm** ([D-019](docs/decisions.md)): `uv sync --extra hardware` adds `motorbridge`; `arm.dry_run: true` first runs `rebot_b601`'s simulated motors. CAN setup: [rebot_b601/README.md](rebot_b601/README.md). Speed: `arm.speed_scale` (1.0 at start) up to `arm.max_speed_scale` (1.4, at most ~1.43: the motors' velocity limit), also from the Speed slider at runtime ([D-030](docs/decisions.md), [D-033](docs/decisions.md)); start it low on the rig: the acceleration is higher than the arm has run so far.

**Config** is in `config/`: `default.yaml` (all sections), `rig.yaml` (computed: poses, zones, ROIs, the arm's floor limit and keep-out), `hand_eye.yaml` (the camera mount), and your own `local.yaml` (gitignored), e.g. `backends: {camera: real}`.

**ROS 2 track:** `ros2_ws/` holds a separate ROS 2 Jazzy + MoveIt 2 cloth task ([D-014](docs/decisions.md)), outside these stages: [ros2_ws/README.md](ros2_ws/README.md). Its rover navigation drives the real Leo Rover in one room: RTAB-Map + Nav2 with an OAK-D on the rover's front, in `ros2_ws/src/rover_nav` ([D-037](docs/decisions.md), [D-038](docs/decisions.md); brief: [docs/rover/ros2-navigation.md](docs/rover/ros2-navigation.md)). A standalone MuJoCo sim of the Leo Rover from its official model, drivable by the jevomir VLM, is in [`rover_nav/sim`](ros2_ws/src/rover_nav/sim/README.md) ([D-039](docs/decisions.md)).

## Working with AI agents

Each stage is handed to one agent. See [AGENTS.md](AGENTS.md) for the parallel workflow. In Claude Code:

```text
/start-stage A       # pick up a stage: reads its brief, creates a branch, marks it in progress
/sync-docs           # before finishing: bring docs in line with the code changes
```
