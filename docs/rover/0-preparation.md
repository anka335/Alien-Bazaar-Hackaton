# Rover stage 0: Preparation

**Status:** done · **Owner:** Softjey + Claude · **Branch:** `stage/0-preparation`
**Unblocks:** [A: Loading](a-loading.md), [B: Unloading](b-unloading.md).

## Goal

A thin shared base so that A and B can work in parallel without touching each other's code: the contracts, a base simulator scene with one scene file per stage, the arm's limits on the rover, the rig computed for the rover, and the wiring of the two modes ([D-033](../decisions.md)). Tuning the scenes, vision, loops, benchmarks and panels belongs to A and B.

## What it delivered

| # | Item | Where |
| --- | --- | --- |
| P1 | Contracts: zones `FLOOR` / `CARGO` / `LAUNDRY`, `Sock` / `FloorResult` / `FloorDetector`, rover phases, modes `load` / `unload`, `pick(..., yaw_rad)`, `drop_to_cargo`, `drop_to_laundry` | `core/types.py`, `core/protocols.py`, [architecture.md](../architecture.md) |
| P2 | MuJoCo base scene (floor 200 mm below the deck, rover chassis and wheels, deck, cargo box with dividers, the arm) and scene files per stage with a baseline each | `sim/physics/model.py`, `sim/scenes/load/`, `sim/scenes/unload/` |
| P3 | Rover layout and the layout tool: poses, zones, ROIs, keep-out, every pick and move checked (0 problems) | `sim/config.py`, `sim/layout.py`, `config/rig.yaml` |
| P4 | Arm on the rover: floor limit, keep-out boxes on every path, gripper yaw, drops via home | `arm/` |
| P5 | Shared state machine + one loop per mode (baselines), floor detector baseline, wiring | `orchestrator/`, `floor_detector/`, `app.py` |
| P6 | The table flow removed: the table loop and layout, the kinematic sim, the table-era docs | — |

Checked: `uv run pytest` passes; `python -m sorter.sim.layout` reports 0 problems; a scripted (no vision) run on the sim moved 3 socks from the floor into their compartments and socks from the compartments into the bins.

## Dropped from the original plan

- The `Rover` interface and the interlock: the rover stands still for our code. Back when the real rover interface is known.
- The shared benchmark: each stage builds its own (A7, B6).
- Scan poses, per-compartment look poses, parking noise, realistic socks and textures: A0–A1 and B0–B1.

## Known issues handed over

- xfail `tests/sim/test_physics.py::test_scripted_load` (A3) and `test_miss_prob_makes_a_grasp_catch_nothing` (A4).
- Skipped until rewritten for the rover: `tests/orchestrator/test_runlog.py` (A5), `tests/color_classifier/test_classifier.py` (A2), `tests/dashboard/test_server.py`, `test_calibrate.py` (A6), `test_manual.py` (B5).
- The front end still shows the table's "Auto" tab and 3D table (A6).
- The calibration page and the board tool were moved to the floor view but not rerun on the sim.

## Log

- 2026-09-27: stage defined ([D-032](../decisions.md)).
- 2026-09-27: done, scope cut to a thin base ([D-033](../decisions.md)).
