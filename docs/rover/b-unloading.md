# Rover stage B: Unloading

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `claude/b-unloading-stage-d746d4`
**Depends on:** [0: Preparation](0-preparation.md) (done). Runs in parallel with [A: Loading](a-loading.md).

## Goal

The rover is parked at the unload station. The arm takes every sock out of the cargo box, tells its color and drops it into the laundry bin of that color. The rover stands still; we don't talk to it ([D-032](../decisions.md), [D-034](../decisions.md)).

**Make it work perfectly in the simulator, as close to reality as you can get it:** a realistic scene (cargo box, socks in it, the station), a robust loop, and a benchmark that proves it. Hardware comes later.

## Where it stands

- **Geometry: the real rover** (photos, 2026-09-27; [D-040](../decisions.md)): one undivided cardboard box 150 × 150 × 60 mm to the arm's left and a bit behind it, so the color is told at unload; three boxes of the same size on the floor in front of the rover; a compact rover (~380 × 300 mm, deck ~160 mm up, electronics behind the arm). `sorter.sim.scenes.unload.rover` lays `REAL_ROVER` (estimates, to be measured) over the committed layout and computes its rig (cached in `data/unload_rig/`); the committed layout and `rig.yaml` still have the 3-compartment box of stage 0, since the box and the load flow are A's too: agree before moving them.
- **Camera:** the sim camera sits at the rig's measured hand-eye mount ([D-041](../decisions.md)). It doesn't see the fingertips; the held sock is seen from the `show_held` pose when it hangs ≥ 55–60 mm below the fingers, so the unload scene's socks are limp like real ones (`sim.unload.sock_young` / `sock_thickness_mm`, [D-042](../decisions.md)).
- **Loop** (`orchestrator/unload.py`, only the wrist camera): the bins are found once per run (`box_detector/station.py`: a square ring matched to the wall points); per sock the top of the pile (`box_detector/cargo.py`), the pick with finger directions tried in turn (open fingers kept off other socks, the gripper opened to ±30 mm), a shake, a second look into the box, `show_held` + the gripper's opening (anything held?); the color from the sock seen hanging (side-lit thresholds), else the box's best match (the one sock gone; of several, the one nearest the grasp; none gone, the grasp's target; [D-042](../decisions.md)); socks of different colors hanging → back into the box, and again; the drop into the found bin (fingers below its rim) via home, a look into the bin (only a seen drop is counted).
- **Benchmark:** `uv run python -m sorter.sim.scenes.unload.bench -n 20` (seeded; the station ±30 mm / ±5°, each bin ±15 mm / ±8°; judged by the sim's ground truth; report in `data/bench/`). Latest result in the *Log*.
- Commands: `python -m sorter.sim.scenes.unload.rover` (rig + pick check, 0 problems), `.preview [--view]`.

## Your lane

Owned: `src/sorter/sim/scenes/unload/`, `src/sorter/orchestrator/unload.py`, `src/sorter/box_detector/`, the unload panel in `frontend/` (B5), `tests/box_detector/`, `tests/orchestrator/test_unload.py`, `tests/sim/` tests of your scene; config `sim.unload`, `sim.layout.laundry`, `box_detector`, a new `unload` section.

Shared (the arm, the base scene incl. the cargo box, `sim/layout.py`, core, the state machine, the dashboard backend and the shared front end): change only through the contract rules in [AGENTS.md](../../AGENTS.md), and note it in A's *Log*. The cargo box is A's target too: move or resize it only by agreement.

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| B0 | Realistic unload scene | in progress | Softjey + Claude | Several socks in realistic piles, bins like the real ones, the station's position noisy (parking tolerance) by seed |
| B1 | Looking into the box and at the bins | done | Softjey + Claude | The box and every bin fully in view with depth |
| B2 | Box and bin detectors | in progress | Softjey + Claude | Correct "empty / grasp" on the benchmark; walls never taken for cloth; bins found within a few mm |
| B3 | Pick and drop with verification | in progress | Softjey + Claude | No sock lands in a wrong bin; a missed pick is retried; counters count verified drops only |
| B4 | Unload loop | in progress | Softjey + Claude | Runs end to end on the sim with holds and errors handled |
| B5 | Dashboard: unload panel | todo | — | Box view, grasp point, per-bin counters visible (on A6's shared front end) |
| B6 | Benchmark pass | in progress | Softjey + Claude | One command runs N seeded scenarios headless; ≥ 95 % of socks unloaded, 0 in a wrong bin |

### B0: Realistic unload scene

- Done: the real rover's geometry, `sim.unload.station_mm` / `station_deg` / `bin_mm` / `bin_deg` (parking and placement noise), socks of mixed colors piled in the box (`sim.unload.cargo`, `sock_mm`).
- Open: socks as A makes them (share the sock model); measure the rover (`REAL_ROVER`); the box's rim is about at deck level on the rover (the box hangs lower), the sim box stands on the deck.

### B2: Detectors

- Known sim gaps that show up here: cloth passes through cloth, so the fingers may pinch the sock under the target (the second look into the box tells which one went), and the fingers drag a neighboring sock out of the box.

### B3: Pick and drop with verification

- The held sock's color comes from the box, lit from above: hanging from the gripper it is seen from the side, in shade, and light socks look dark there.

### B4: Unload loop

- In `unload.py`; the shared state machine gives commands, hold, errors, the run log. Starts at the station, ends at `home`.

### B5: Dashboard

- Wait for A6's shared front-end update (tabs, 3D view from `parts`), then add the unload panel. Rewrite the skipped `tests/dashboard/test_manual.py` for the rover poses.

### B6: Benchmark

- `N` seeded scenarios (default 50), headless, `sim.realtime: 0`. Metrics: socks unloaded, in a wrong bin, left in the cargo box, time per sock, failures by type. Report to `data/bench/<run_id>/`.

## Exit criteria

B0–B6 done on the sim.

## Open questions

- Move the committed layout to the real rover's geometry (one box, color told at unload)? Decide with A: it changes A's drop and removes A's color step.

## Requests from other blocks

*None yet.*

## Log

- 2026-09-27: stage defined ([D-032](../decisions.md)).
- 2026-09-27: stage 0 done; the brief rewritten for what it delivered ([D-034](../decisions.md)).
- 2026-09-27: stages N (navigation), C (full mission), D (sim-to-real) added ([D-035](../decisions.md)): N4 docks at the station, which bounds B0's parking noise.
- 2026-09-27: B0 started: unload scene supports an undivided box; preview of the real rover's geometry.
- 2026-09-27: vision-only loop on the real rover's geometry, bench and demo (`python -m sorter.sim.scenes.unload.demo`). Bench, 8 × 6 socks, before the held-first color: 64.6 % in the right bin, 6 in a wrong one, 11 lost; bins found to 1–2 mm. The held-first color and the flatter initial pile are not benchmarked yet. Open problems: the sim's pile (cloth passes through cloth: the wrong sock pinched, the pile settles anew), neighbors dragged out of the box, the held sock seen only when it hangs ≥ 60 mm (the rig camera is off to the side) and side-lit color thresholds tuned on ~12 sim samples, ~50 s of sim per sock, the rover's dimensions are estimates, the committed layout still has stage 0's 3-compartment box.
- 2026-09-27: the loop kept putting socks back: the sim's stiff cloth stayed bunched at the fingers, below the camera's view from `show_held` (it sees ≥ 55 mm below them), and the box's "one sock gone" failed on a pile that settles anew. Fixed by limp socks in the unload scene and the box's best match as the fallback color ([D-042](../decisions.md)). Also fixed the dashboard's `/` crash (the front end still had the `auto` mode).
