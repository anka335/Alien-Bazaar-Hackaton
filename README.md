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
| [docs/tasks/](docs/tasks/) | One file per block: scope, acceptance criteria, open questions |
| [AGENTS.md](AGENTS.md) | Rules for AI coding agents (and humans) working in this repo |

## Getting started

Requires [uv](https://docs.astral.sh/uv/). It installs Python 3.11 and the dependencies itself.

```bash
uv sync                                   # install
uv run python -m sorter run --sim         # the whole loop on the simulated hardware
uv run pytest                             # tests
```

`run --sim` starts the loop and the dashboard at http://127.0.0.1:8000. Press Start there, or pass `--autostart`. The red Hold button (or Space / Esc on the page) freezes the arm; Reset continues. Ctrl+C holds the arm and shuts down. To open the dashboard from another device, set `dashboard.host: 0.0.0.0` in `config/local.yaml`. Other flags: `--no-dashboard`, `--config-dir`, `-v`. Every run is logged to `data/runs/<run_id>/` (turn off with `state_machine.save_runs: false`).

**The simulator** simulates the hardware only, the arm and the wrist camera, in a MuJoCo physics scene ([D-016](docs/decisions.md)); calibration, box detection and color classification run their real code on the rendered RGB-D frames, and the arm runs the real `rebot_b601` control loop on simulated motors. So `run` without `--sim` runs the same code on the rig. Clothes are cloth that falls, folds and hangs from the gripper. `sim.realtime` sets the speed (1 = real time, `0` = as fast as the CPU allows); `sim.engine: kinematic` is the quick physics-free world the tests use. The color classifier uses the render's segmentation instead of SAM3 (`sim.use_sam3: true` to call the service). The dashboard's main screen switches between the last decision frame and a **3D view** (also at http://127.0.0.1:8000/twin): the arm, the table, the clothes, and what the wrist camera sees.

**Table layout** ([D-015](docs/decisions.md)), arm base at the origin, +x forward, +y left, set in `sim.layout`: a low tray (inside 240 × 180 mm, walls 60 mm) centered at (0, −270), the gray mat (240 × 180 mm) at (270, 0), bins (180 mm, walls 150 mm) at (257, 306) light, (0, 400) dark, (−257, 306) colored. The wrist camera sits 140 mm behind the fingertips and 55 mm off-axis, looking along the gripper. The arm reaches the tray and the mat only with this geometry (the gripper pointing down tops out at ~120 mm TCP height), so build the real table the same way. After changing the layout, recompute the poses and zones: `uv run python -m sorter.sim.layout --write` (it checks that every pick plans).

**Config** is in `config/`: `default.yaml` (all sections), `rig.yaml` (poses, zones, ROIs of the rig; poses and zones computed for the layout), `hand_eye.yaml` (calibration result), and your own `local.yaml` (gitignored, machine overrides). `backends` chooses `real` or `sim` per component, for example in `config/local.yaml`:

```yaml
backends:
  camera: real
```

**Camera** (block 1): `uv sync --extra camera` adds `pyrealsense2` (Linux / Windows). `camera.serial` picks one D435i (empty = the first found); exposure and white balance lock after `camera.warmup_frames` (`camera.exposure_us`, `camera.white_balance_k` to fix them).

**Hand-eye calibration** (block 2): print the ChArUco board (`uv run python -m sorter.calibration.board`, 7 × 5 squares of 30 mm; check the printed size), lay it flat on the mat, then `uv run python -m sorter.calibration.hand_eye`: the arm visits `calibration.poses` views around `look_bg` and writes `config/hand_eye.yaml` (commit it). `--sim` rehearses it on the simulator and reports the error against the true mount.

**Color classifier** (block 4) segments the background with a remote SAM3 service ([D-013](docs/decisions.md)). To use it (`backends.color_classifier: real`), put the API key in `config/local.yaml` (`color_classifier: {sam: {api_key: ...}}`) or in the `SAM3_API_KEY` env var. With `sim.use_sam3: true`, set `sam.prompts: [blob]` and `sam.threshold: 0.3`, since SAM3 doesn't see the rendered cloth as clothing. Tuning tool: `uv run python -m sorter.color_classifier.stats <observation.npz ...>` or `--sim 3` (`--prompts a,b`, `--threshold`); it prints the color stats of every item (`--save DIR` writes overlays).

**Real arm** (block 5, [D-014](docs/decisions.md)): `uv sync --extra hardware` adds `motorbridge`. `run` without `--sim` uses it; with `arm.dry_run: true` it first runs `rebot_b601`'s simulated motors (the same control loop and safety checks, no bus). The CAN setup (`can0`, zero calibration) is in [rebot_b601/README.md](rebot_b601/README.md). The arm's speed is `arm.speed_scale` (0.5 of `rebot_b601`'s joint speeds, capped at 0.6).

## Working with AI agents

Every block is designed to be handed to an agent on its own. See [AGENTS.md](AGENTS.md) for the parallel workflow. In Claude Code:

```text
/start-block 04      # pick up a block: reads its task file, creates a branch, marks it in progress
/sync-docs           # before finishing: bring docs in line with the code changes
```
