# Rover stage B: Unloading

**Status:** todo · **Owner:** — · **Branch:** —
**Depends on:** [0: Preparation](0-preparation.md) (done). Runs in parallel with [A: Loading](a-loading.md).

## Goal

The rover is parked at the unload station. The arm empties the cargo box into the laundry bins. The box is one box now, not split by color ([D-035](../decisions.md)): whether unloading classifies each sock's color to pick its bin, or everything goes into one bin, is open. The rover stands still; we don't talk to it ([D-032](../decisions.md), [D-034](../decisions.md)).

**Make it work perfectly in the simulator, as close to reality as you can get it:** a realistic scene (cargo box, socks in it, the station), a robust loop, and a benchmark that proves it. Hardware comes later.

## What you start from

- `uv run python -m sorter run --sim --mode unload`: the MuJoCo scene of the rover as on its photos: the arm on its deck 160 mm above the floor, one cardboard cargo box to the arm's left (150 × 150 mm inside, 60 deep, rim 25 mm above the deck; [D-035](../decisions.md)), the `sim.unload.cargo` socks in it, and the 3 laundry bins on the floor to the right. The dashboard's front end is not updated yet (A6 does the shared part).
- A **baseline loop** in `src/sorter/orchestrator/unload.py`: `LOOK_CARGO` → `SENSE_CARGO` (the depth box detector on each compartment's image area, projected from `look_cargo`) → `PICK_FROM_CARGO` (fingers along the compartment's long side, yaw π/2) → `DROP_TO_LAUNDRY(c)` → `LOOK_CARGO`. It has never run end to end, and it counts a sock before checking it: expect to fix it. It walks `cargo.compartments`, which is empty with the one box, so it finds nothing until reworked.
- The arm: `pick(target, Zone.CARGO, yaw_rad)`, `drop_to_laundry(color)`; every path is checked against the floor and the keep-out (the rover body, the cargo walls and dividers). `rig.yaml` has `look_cargo`, `laundry_<color>`, zone `cargo`. A scripted pick from a compartment into its bin works (`tests/sim/test_physics.py::test_scripted_unload`).
- Scene file you own: `src/sorter/sim/scenes/unload/scene.py` (the bins and the socks in the compartments). `PhysicsWorld.location(item)` tells where each sock ended (`("cargo", None)` in the box, `laundry` + color).
- Contracts and the sim: [architecture.md](../architecture.md).

## Your lane

Owned: `src/sorter/sim/scenes/unload/`, `src/sorter/orchestrator/unload.py`, `src/sorter/box_detector/`, the unload panel in `frontend/` (B5), `tests/box_detector/`, `tests/orchestrator/test_unload.py`, `tests/sim/` tests of your scene; config `sim.unload`, `sim.layout.laundry`, `box_detector`, a new `unload` section.

Shared (the arm, the base scene incl. the cargo box, `sim/layout.py`, core, the state machine, the dashboard backend and the shared front end): change only through the contract rules in [AGENTS.md](../../AGENTS.md), and note it in A's *Log*. A also needs new named poses: keep such changes small and separate. The cargo box is A's target too: move or resize it only by agreement.

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| B0 | Realistic unload scene | todo | — | Several socks per compartment in realistic piles, bins like the real ones, the station's position noisy (parking tolerance) by seed |
| B1 | Looking into each compartment | todo | — | Every compartment fully in view with depth, from its own look pose |
| B2 | Compartment detector | todo | — | Correct "empty / grasp" per compartment on the benchmark; walls and dividers never taken for cloth |
| B3 | Pick and drop with verification | todo | — | No sock lands in a wrong bin; a missed pick is retried; counters count verified drops only |
| B4 | Unload loop | todo | — | Runs end to end on the sim with holds and errors handled |
| B5 | Dashboard: unload panel | todo | — | Compartment view, grasp point, per-bin counters visible (on A6's shared front end) |
| B6 | Benchmark pass | todo | — | One command runs N seeded scenarios headless; ≥ 95 % of socks unloaded, 0 in a wrong bin |

### B0: Realistic unload scene

- Socks as A makes them (share the sock model with A), `n` per compartment, dropped in and settled. Bins of real laundry-basket size where the arm still reaches them (the gripper held down reaches ~100 mm above the deck; the layout tool checks the drop poses).
- Parking tolerance: in the sim the rover doesn't move, so shift the station (bins) by a seeded (x, y, yaw) noise. If the fixed drop poses can't absorb it, add a correction (e.g. a marker on the station seen from a look pose) and record the choice in `decisions.md`.

### B1: Looking into each compartment

- `look_cargo` (camera ~270 mm above the box floor) sees ~281 × 211 mm: the whole 150 × 150 mm box.

### B2: Compartment detector

- The depth box detector with one ROI per compartment (the baseline projects the compartment rectangle), parameters for socks (small, light, thin). `EMPTY` confirmed `empty_confirmations` times before moving on. `avoid` per compartment.

### B3: Pick and drop with verification

- Verify by looking at the compartment again (observation-driven, [D-008](../decisions.md)): fewer socks → counted; same → failure, retry.
- A sock that falls into another compartment would end up in the wrong bin: the drop path must not cross other compartments, and the detector must not look outside its ROI.

### B4: Unload loop

- In `unload.py`; the shared state machine gives commands, hold, errors, the run log. Starts at the station, ends at `home`.

### B5: Dashboard

- Wait for A6's shared front-end update (tabs, 3D view from `parts`), then add the unload panel. Rewrite the skipped `tests/dashboard/test_manual.py` for the rover poses.

### B6: Benchmark

- `N` seeded scenarios (default 50), headless, `sim.realtime: 0`. Metrics: socks unloaded, in a wrong bin, left in the cargo box, time per sock, failures by type. Report to `data/bench/<run_id>/`.

## Exit criteria

B0–B6 done on the sim.

## Open questions

- Color sorting with one box ([D-035](../decisions.md)): classify each sock while unloading (reuse A's color classifier), or one bin?

## Requests from other blocks

*None yet.*

## Log

- 2026-09-27: stage defined ([D-032](../decisions.md)).
- 2026-09-27: stage 0 done; the brief rewritten for what it delivered ([D-034](../decisions.md)).
- 2026-09-27 (from A): the rover layout follows its photos ([D-035](../decisions.md)): deck 160 mm above the floor, one cargo box 150 × 150 × 60 mm at (−50, 200) with no compartments (`compartment(c)` = the whole box, `location()` → `("cargo", None)`), keep-out for the wheels and the equipment behind the arm. The laundry bins didn't move. `unload.py` walks the compartments and now finds none.
