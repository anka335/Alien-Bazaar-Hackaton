# Rover stage 0: Preparation

**Status:** todo · **Owner:** — · **Branch:** —
**Blocks:** [A: Loading](a-loading.md), [B: Unloading](b-unloading.md). Both start when this stage is done.

## Goal

Everything that stages A and B share, fixed before they split: the contracts, the rover simulator scene, the reach layout, the arm's new limits and a benchmark that measures "good enough". After this stage A and B run in parallel without touching each other's code.

Sim first ([D-032](../decisions.md)): nothing here needs the hardware.

## Assumptions

- The rover is built and driven by others. It stops with a sock inside the arm's reach and tells us `stopped`. We answer `release()` when the arm is stowed and it may drive on.
- The onboard (cargo) box has **3 compartments**, one per color (light / dark / colored).
- The unload station has 3 laundry bins at fixed places relative to where the rover parks.

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| P1 | Contracts | todo | — | New contracts in `architecture.md`, stubs importable, old table-only contracts marked for removal |
| P2 | Rover sim scene | todo | — | `run --sim` shows the rover scene; a scenario is reproducible from its seed |
| P3 | Reach layout | todo | — | Layout tool passes on all poses and moves; the rover stop requirement is written down |
| P4 | Arm: stow and keep-out | todo | — | Tests: no motion enters the rover body or goes below the floor; `stow` reachable from every pose |
| P5 | Sim benchmark | todo | — | One command runs N seeded scenarios and prints the metrics |
| P6 | Drop the table flow | todo | — | Old table-only code and docs removed or marked; status board in `plan.md` is the rover plan |

### P1: Contracts

- Zones: `FLOOR`, `CARGO` (compartment per `ColorClass`), `LAUNDRY` (bin per `ColorClass`). `BACKGROUND` goes away.
- `Rover` protocol + stub: `wait_stopped(timeout)`, `is_stopped()`, `release()`. The sim stub is driven by the scenario generator (P2).
- Interlock, both ways: the arm moves only while the rover is stopped; `release()` only with the arm in `stow`.
- `FloorDetector` protocol: floor frame → list of socks (grasp pixel, grasp angle, color, confidence, mask). Reuses the shape of `ColorClassifier`'s result.
- Box detector: per-compartment ROI (same `BoxResult`).
- Hub: operator modes `load` and `unload` in place of `auto`.
- Update `architecture.md` (physical setup, zones, both loops) and the *Log* of affected task files.

### P2: Rover sim scene

- MuJoCo scene: the arm on a rover body at a configurable mount height, the cargo box with 3 compartments, a floor with interchangeable textures (plain, wood, carpet-like), socks as small cloth flexes in several colors, the unload station with 3 laundry bins.
- The rover does not drive. A scenario generator places the rover (and socks around it, or the rover at the station) from a seed and fires `stopped`.
- Parking noise at the station (x, y, yaw) is a config value, so B can test against it.
- Sim segmentation stays MuJoCo's, served as SAM3-style instances.
- `sim.miss_prob` for floor grasps, so the retry paths are exercised: the grip attachment is optimistic.

### P3: Reach layout

- Like `sorter.sim.layout`: mount height, the reachable ring on the floor, scan poses, compartment and bin poses; IK check of every pose and move.
- Output: the **rover stop requirement** ("sock between X and Y mm from the base, within ±θ"), handed to the rover team.

### P4: Arm: stow and keep-out

- `stow` pose for driving: low and compact.
- Keep-out box for the rover body and wheels; floor as the lower z limit.
- Scan poses, cargo drop poses per compartment, laundry drop poses per bin.

### P5: Sim benchmark

- `N` seeded scenarios (default 50) per mode, headless, `sim.realtime: 0`.
- Metrics: success rate, correct compartment / bin, time per sock, failures by type. Report to `data/bench/<run_id>/`.
- This is the yardstick for "good" in A and B.

### P6: Drop the table flow

- The mat, `LOOK_BG` / background-first loop ([D-008](../decisions.md)) and the table layout no longer apply. Remove or mark what A and B will replace, so no one builds on it.
- Rebuild the status board in [plan.md](../plan.md) around the rover stages.

## Owned paths (tentative, P1 fixes them)

`src/sorter/core/`, `src/sorter/sim/`, `src/sorter/rover/` (new), `src/sorter/arm/`, `tests/core/`, `tests/sim/`, `tests/rover/`, `tests/arm/`, `config/default.yaml` → `sim`, `rover`, `arm`.

## Exit criteria

- P1–P6 done.
- On the sim a scripted (no vision) load and unload run end to end: stopped → pick a known sock → cargo → release; station → compartment → bin.

## Open questions

- Rover dimensions and arm mount height. Until known: placeholders in config.
- The rover's real interface (ROS 2 topic, serial, HTTP). The stub hides it until stage 3.
- How `ros2_ws/` (MoveIt, `cloth_task`) relates to this plan. The plan builds on `sorter`'s MuJoCo sim.

## Log

- 2026-09-27: stage defined ([D-032](../decisions.md)).
