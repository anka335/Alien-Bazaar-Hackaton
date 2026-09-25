# Alien Bazaar Hackathon — Robotic Laundry Sorter

An autonomous robotic system that takes clothes from a mixed pile and sorts them by color.

A robotic arm picks items one by one from a box of mixed clothing, using a depth camera mounted on its wrist to locate a grasp point. Each item is placed on a uniform background area, where a color classifier determines whether it is **light**, **dark**, or **colored**. The arm then picks the item up again and drops it into the matching bin, repeating the cycle until the box is empty. A live dashboard shows the camera feed, detected grasp points and colors, the current system state, and item counts per bin.

This demo addresses the core challenge of automated laundry handling — picking and sorting deformable clothing items from a cluttered pile — as the first step toward a fully automated wash–dry–sort pipeline.

## Hardware

| Component | Details |
| --- | --- |
| Robot arm | [Seeed reBot Arm B601-DM](https://www.seeedstudio.com/reBot-Arm-B601-DM-p-6740.html): 6 DoF + parallel gripper, Python SDK [`reBotArm_control_py`](https://github.com/Seeed-Projects/reBotArm_control_py) |
| Camera | RGB-D (depth) camera mounted on the arm's wrist (eye-in-hand), model: _TBD_ |
| Work area | at fixed positions: mixed-clothes box, uniform background area (mid-gray), 3 bins (light / dark / colored) |
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

_TBD. Filled in by block 0 once the stack and repo layout are fixed._

## Working with AI agents

Every block is designed to be handed to an agent on its own. See [AGENTS.md](AGENTS.md) for the parallel workflow. In Claude Code:

```text
/start-block 04      # pick up a block: reads its task file, creates a branch, marks it in progress
/sync-docs           # before finishing: bring docs in line with the code changes
```
