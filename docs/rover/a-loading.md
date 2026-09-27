# Rover stage A: Loading

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `claude/pensive-banach-a6b5d1`
**Depends on:** [0: Preparation](0-preparation.md) (done). Runs in parallel with [B: Unloading](b-unloading.md).

## Goal

The rover has stopped next to socks on the floor. The arm finds them, classifies each sock's color on the spot, picks it up and drops it into the cargo box (one box, [D-035](../decisions.md)), until no sock is left in reach. The rover stands still; we don't talk to it ([D-032](../decisions.md), [D-034](../decisions.md)).

**Make it work perfectly in the simulator, as close to reality as you can get it:** a realistic scene (socks, floor, light, camera noise), a robust loop, and a benchmark that proves it. Hardware comes later.

## What you start from

- `uv run python -m sorter run --sim` (mode `load`): the MuJoCo scene of the rover as on its photos ([D-035](../decisions.md)): the arm on its deck 160 mm above the floor, the equipment behind it, the cardboard cargo box (150 × 150 × 60 mm) to its left over the rear wheel, a parquet floor, and the socks of `sim.load.socks` (sock-shaped, ~200 × 90 mm) anywhere the arm reaches (`sim.load.area: reach`; `view` puts them in the floor view). The dashboard's front end is not updated yet (A6).
- A **baseline loop** in `src/sorter/orchestrator/load.py` (it looks only at the floor view, so it misses socks elsewhere until A1): `SCAN` (observe the floor from `look_floor`) → `SENSE_FLOOR` → `PICK_FROM_FLOOR` → `DROP_TO_CARGO` → `SCAN`, counted when the next look shows one sock fewer. It has never run end to end: expect to fix it.
- A **baseline floor detector** `ClassifierFloorDetector` (`src/sorter/floor_detector/`): block 4's color classifier over `views.floor.roi`, on the render's segmentation in the sim. No grasp angle, no mask.
- The arm: `pick(target, Zone.FLOOR, yaw_rad)` (gripper down, fingers opening along `yaw_rad`), `drop_to_cargo(color)`; every path is checked against the floor and the rover's keep-out. `rig.yaml` has `look_floor`, `cargo_<color>` (all three over the one box), zone `floor` = the floor view (280 × 240 mm around (310, 0)).
- Scene file you own: `src/sorter/sim/scenes/load/scene.py` (`add()` gets the MJCF world and returns the socks). `PhysicsWorld.location(item)` tells where each sock ended (`("cargo", None)` in the box): the ground truth for tests and the benchmark.
- Contracts and the sim: [architecture.md](../architecture.md).

## Your lane

Owned: `src/sorter/sim/scenes/load/`, `src/sorter/orchestrator/load.py`, `src/sorter/floor_detector/`, `src/sorter/color_classifier/`, the shared front-end update and the load panel in `frontend/` (A6), `tests/floor_detector/`, `tests/color_classifier/`, `tests/orchestrator/test_load.py`, `tests/sim/` tests of your scene; config `sim.load`, `sim.layout.floor_view`, `color_classifier`, a new `floor_detector` / `load` section.

Shared (the arm, the base scene, `sim/layout.py`, core, the state machine, the dashboard backend): change only through the contract rules in [AGENTS.md](../../AGENTS.md), and note it in B's *Log*. B also needs new named poses: keep such changes small and separate.

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| A0 | Realistic load scene | in progress | Softjey + Claude | Sock-shaped cloth of real sizes, several floor textures, lighting and depth noise like the D435i on a real floor; a scene is reproducible from its seed |
| A1 | Floor scan | todo | — | A sock is found wherever it lies in the reachable ring, not only in the floor view |
| A2 | Floor detector + color | todo | — | ≥ 95 % correct colors across floor textures on the benchmark; grasp angle and mask filled |
| A3 | Grasp from the floor | todo | — | ≥ 90 % successful grasps on the benchmark |
| A4 | Verify and retry | todo | — | Every failure path has a test; counters only count socks that are in their compartment |
| A5 | Load loop | todo | — | Runs end to end on the sim with holds and errors handled |
| A6 | Dashboard: front end + load panel | todo | — | Tabs Load / Unload, the 3D view from `/api/twin/layout` `parts`, rover phases; the load panel shows the scan frame, found socks, per-compartment counters |
| A7 | Benchmark pass | todo | — | One command runs N seeded scenarios headless; ≥ 90 % of socks end in the right compartment |

### A0: Realistic load scene

- Done: the rover, the box and the equipment from photos; herringbone parquet like the real floor; socks with a sock outline (`sock_shape`: leg, heel bend, rounded toe; `ItemSpec.rest_m`), ~70 % flat, the rest bunched, spread over the reachable ring.
- Left: more floor textures (plain, carpet-like) picked by seed; sock knit texture and a two-tone heel and toe; the lamp and the camera's noise as on the rig. Socks look ~12 mm thick (the flex radius); the arm is the URDF's green, the real one is lime.
- Cloth costs simulation time (3 socks ≈ 2.6× real time): keep the benchmark fast enough.

### A1: Floor scan

- Scan poses around the arm (a few joint-1 angles, camera down): new named poses through `sim/layout.py` (shared: tell B). Merge detections from all views in arm coordinates, drop duplicates, pick the target (nearest, then most confident); stop early once a confident target is in view.
- Widen `zones.floor` to what the arm really reaches (the layout tool checks every pick).

### A2: Floor detector + color

- Segmentation: the render's segmentation in the sim, SAM3 with the prompt `"sock"` on hardware. Color from the mask pixels (the color classifier's logic). Reject blobs too small, too large or not sock-shaped. Stateless: frame in, socks out.
- Replace the baseline detector (or grow it); rewrite the skipped `tests/color_classifier/test_classifier.py` on the floor scene.

### A3: Grasp from the floor

- Grasp point: mask center or thickest part; yaw across the sock's short axis (PCA of the mask), converted from the image to the arm frame. Floor height from the depth around the sock, not a constant.
- The scripted floor → box drop (`test_scripted_load`) works; over seeds 0–5 one sock (seed 5) ended on the box's rim (`other`). The box is small (150 mm): aim the drop and check where it lands.

### A4: Verify and retry

- After a pick: the gripper (`likely_empty` is only a hint), and the next look. Miss → retry up to `load.max_attempts`, then skip the sock and log it.
- `sim.miss_prob = 1` makes a floor grasp miss (6 of 6 seeds in the new scene).

### A5: Load loop

- In `load.py`; the shared state machine gives commands, hold, errors, the run log. Rewrite the skipped `tests/orchestrator/test_runlog.py` on it.

### A6: Dashboard

- The front end (`frontend/`) still has the table's "Auto" tab and 3D table. Update the shared parts (`api.ts` modes, `TopBar` tabs `/load`, `/unload`, the 3D view drawing `parts` generically, the phases), then the load panel. Tell B when the shared part is merged: B's panel builds on it.
- Rewrite the skipped `tests/dashboard/test_server.py` and `test_calibrate.py` (calibration now on the floor view).

### A7: Benchmark

- `N` seeded scenarios (default 50), headless, `sim.realtime: 0`. Metrics: socks in the right compartment, wrong compartment, left on the floor, time per sock, failures by type. Report to `data/bench/<run_id>/`. This is the yardstick for "done".

## Exit criteria

A0–A7 done on the sim.

## Open questions

- Several socks at one stop: collect all in reach (planned) or only the one the rover stopped for?
- With one box ([D-035](../decisions.md)): does loading still need the color (for the dashboard, or to hand it to unloading), or does sorting happen at unload?

## Requests from other blocks

*None yet.*

## Log

- 2026-09-27: stage defined ([D-032](../decisions.md)).
- 2026-09-27: stage 0 done; the brief rewritten for what it delivered ([D-034](../decisions.md)).
- 2026-09-27: A0 started: the rover, the one cargo box and the parquet floor from photos, sock-shaped socks over the reachable ring ([D-035](../decisions.md)); the shared layout, keep-out and base scene changed (noted in B's Log).
