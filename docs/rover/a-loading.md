# Rover stage A: Loading

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `claude/pensive-banach-a6b5d1`
**Depends on:** [0: Preparation](0-preparation.md) (done). Runs in parallel with [B: Unloading](b-unloading.md).

## Goal

The rover has stopped next to socks on the floor. The arm finds them, classifies each sock's color on the spot, picks it up and drops it into the cargo box (one box, [D-040](../decisions.md)), until no sock is left in reach. The rover stands still; we don't talk to it ([D-032](../decisions.md), [D-034](../decisions.md)).

**Make it work perfectly in the simulator, as close to reality as you can get it:** a realistic scene (socks, floor, light, camera noise), a robust loop, and a benchmark that proves it. Hardware comes later.

## Where it stands

- **Loop** (`orchestrator/load.py`, [D-041](../decisions.md)): scans a ring of 7 poses around the arm, picks the nearest sock (a closer, aimed look if cut off), drops it after it stops swinging, counts it only if its spot is empty and the box's surface rose where it landed. Retries up to `load.max_attempts`, then leaves the sock.
- **Detector** (`floor_detector/`): `SockDetector`, masks sized in mm from depth, grasp at the widest part, yaw across the sock there.
- **Sim**: the camera where the real one was calibrated; the rover, box and parquet from photos; sock-shaped socks over the reachable ring; cloth lies on cloth.
- **See it**: `uv run mjpython -m sorter.sim.scenes.load.watch` (live in the MuJoCo viewer) or `uv run python -m sorter.sim.scenes.load.watch --no-viewer --record run.mp4` (a video of one run); `uv run python -m sorter.sim.scenes.load.bench -n 50 --workers 3` (the numbers).
- **Numbers so far**: 12 scenes, 29 socks: 93 % of the reachable socks ended in the box (before the last changes). A partial 50-scene run after them: 15 of 18 socks in the box in 9 scenes, one scene ended in ERROR (5 failures in a row, 0 of 4 loaded).

### Known issues

- **Seed 0 of the 50-scene run: ERROR after 5 consecutive failures, 0/4 loaded.** Not investigated. Likely several socks close together or a sock the pick keeps missing: failures from different socks add up to `max_consecutive_failures`.
- **Counting**: a sock in the box is sometimes not counted (white socks especially: the raised area stays small), and once one was counted that wasn't there. `load.raised_mm` / `min_raised_mm2` and the box view need tuning with data.
- **`look_cargo`** sees the box from only ~210 mm (the arm can't get the camera higher over a box that close to its base): the box just fits the frame; a sock draped over the far wall is cut off.
- **Drops**: coming in at the drop height dragged the hanging sock over the box's wall; now the arm comes in high, lets go under the rim and backs out (`arm.cargo_drop_*`). The box (150 mm) is shorter than a sock (200 mm): with 3+ socks the pile reaches the rim and a sock can still flop over it.
- **Speed**: ~50 s of sim time per sock; a 4-sock scene takes 2–6 min wall (cloth, 3 scenes in parallel). A pile in the box costs ~5x per step with `sim.cloth_collisions`.
- **Sim nondeterminism**: the loop's thread timing changes when the arm gets its next command, so a seed doesn't replay exactly.
- **Not done**: A6 (dashboard front end), floor textures other than parquet (A0), the unload loop still walks compartments (B).

## Your lane

Owned: `src/sorter/sim/scenes/load/`, `src/sorter/orchestrator/load.py`, `src/sorter/floor_detector/`, `src/sorter/color_classifier/`, the shared front-end update and the load panel in `frontend/` (A6), `tests/floor_detector/`, `tests/color_classifier/`, `tests/orchestrator/test_load.py`, `tests/sim/` tests of your scene; config `sim.load`, `sim.layout.floor_view`, `color_classifier`, a new `floor_detector` / `load` section.

Shared (the arm, the base scene, `sim/layout.py`, core, the state machine, the dashboard backend): change only through the contract rules in [AGENTS.md](../../AGENTS.md), and note it in B's *Log*. B also needs new named poses: keep such changes small and separate.

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| A0 | Realistic load scene | in progress | Softjey + Claude | Sock-shaped cloth of real sizes, several floor textures, lighting and depth noise like the D435i on a real floor; a scene is reproducible from its seed |
| A1 | Floor scan | done | Softjey + Claude | A sock is found wherever it lies in the reachable ring, not only in the floor view |
| A2 | Floor detector + color | review | Softjey + Claude | ≥ 95 % correct colors across floor textures on the benchmark; grasp angle and mask filled |
| A3 | Grasp from the floor | review | Softjey + Claude | ≥ 90 % successful grasps on the benchmark |
| A4 | Verify and retry | in progress | Softjey + Claude | Every failure path has a test; counters only count socks that are in their compartment |
| A5 | Load loop | review | Softjey + Claude | Runs end to end on the sim with holds and errors handled |
| A6 | Dashboard: front end + load panel | todo | — | Tabs Load / Unload, the 3D view from `/api/twin/layout` `parts`, rover phases; the load panel shows the scan frame, found socks, per-compartment counters |
| A7 | Benchmark pass | in progress | Softjey + Claude | One command runs N seeded scenarios headless; ≥ 90 % of socks end in the right compartment |

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
- With one box ([D-040](../decisions.md)): does loading still need the color (for the dashboard, or to hand it to unloading), or does sorting happen at unload?

## Requests from other blocks

*None yet.*

## Log

- 2026-09-27: stage defined ([D-032](../decisions.md)).
- 2026-09-27: stage 0 done; the brief rewritten for what it delivered ([D-034](../decisions.md)).
- 2026-09-27: A0 started: the rover, the one cargo box and the parquet floor from photos, sock-shaped socks over the reachable ring ([D-040](../decisions.md)); the shared layout, keep-out and base scene changed (noted in B's Log).
- 2026-09-27: the load loop scanning a ring around the arm, the sock detector, the camera from the hand-eye result, the benchmark and the watch tool ([D-041](../decisions.md)); shared: `ArmController.go_to` / `aim_camera`, `Observer.observe(zone, pose)` / `observe_point`, `plan_move`, `arm.link5_points_mm`, `arm.drop_settle_s`, phases `AIM` / `CHECK_LOAD`, scan poses in `rig.yaml`, `sim.cloth_collisions` (noted in B's Log).
- 2026-09-27: the rover measured ([D-042](../decisions.md)): deck 200 mm high, arm at the front edge, the box 190 mm outside; `rig.yaml` recomputed. The benchmark numbers above are from the old layout.
- 2026-09-27: stages N (navigation), C (full mission), D (sim-to-real) added ([D-035](../decisions.md)): repositioning and the rover interface are C's, N hands over at the stop requirement.
