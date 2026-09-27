# Rover stage A: Loading

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `claude/pensive-banach-a6b5d1`
**Depends on:** [0: Preparation](0-preparation.md) (done). Runs in parallel with [B: Unloading](b-unloading.md).

## Goal

The rover has stopped next to socks on the floor. The arm finds them, classifies each sock's color on the spot, picks it up and drops it into the cargo box (one box, [D-040](../decisions.md)), until no sock is left in reach. The rover stands still; we don't talk to it ([D-032](../decisions.md), [D-034](../decisions.md)).

**Make it work perfectly in the simulator, as close to reality as you can get it:** a realistic scene (socks, floor, light, camera noise), a robust loop, and a benchmark that proves it. Hardware comes later.

## Where it stands

- **Loop** (`orchestrator/load.py`, [D-041](../decisions.md)): scans a ring of 7 poses around the arm, picks the nearest sock (a closer, aimed look if cut off), drops it after it stops swinging, counts it only if its spot is empty and the box's surface rose where it landed. Retries up to `load.max_attempts`, then leaves the sock. Two sightings are one sock when one's grasp point lies on the other's cloth (its mask on the floor, `load.same_sock_mm`): the grasp point moves between views, the cloth doesn't. A closer look that doesn't find the sock counts as a failed attempt. A pick that doesn't plan at the sock's yaw is turned off it in 15° steps up to 90° ([D-051](../decisions.md)).
- **Detector** (`floor_detector/`): `SockDetector`, masks sized in mm from depth, grasp at the widest part, yaw across the sock there.
- **Sim**: the camera where the real one was calibrated; the rover, box and parquet from photos; sock-shaped socks over the reachable ring; cloth lies on cloth.
- **See it**: `uv run mjpython -m sorter.sim.scenes.load.watch` (live in the MuJoCo viewer) or `uv run python -m sorter.sim.scenes.load.watch --no-viewer --record run.mp4` (a video of one run); `uv run python -m sorter.sim.scenes.load.bench -n 50 --workers 3` (the numbers).
- **Numbers so far** (the box on the right, the arm base turned and leaning as measured, D-045 / D-049 / D-050): 3 scenes, 10 socks, all in the floor zone: 9 in the box (90 %), counted exactly (none extra), colors 100 %, every run DONE. `watch --seed 3` and `--seed 0`: 4/4, a sock that fell on the way was found on the floor and loaded again.
- **Scene**: `watch` and `bench` put the socks inside `rig.yaml`'s floor zone (`sim.load.area: zone`); the default ring (`reach`) is for the tests.

### Known issues

- **A sock sometimes falls on the way to the box** (1 of ~14 picks): it lands beside the box or on the rover. On the floor the loop finds it and picks it again; on the rover it is lost. Not seen why yet (the overview camera doesn't show the box behind the arm).
- **Gripper reading**: on the rig a sock squeezed at 4.5 Nm reads 0.000–0.012, like a gripper closed on nothing (one read 0.025): the reading can't tell them apart. A `likely_empty` pick is checked with a look at the spot: the sock still there is a miss, gone means held and it goes to the box (`CHECK_LOAD` then confirms). In the sim a pinched sock keeps the fingers `PINCHED_SOCK_M` (2.5 mm) apart and reads 0.028 against `empty_below` 0.02.
- **`look_cargo`** sees the box from ~220 mm above its floor, the highest the arm gets: the frame just covers the box's inside; a sock draped over a wall is cut off.
- **Drops**: the arm comes in high, lets go under the rim and backs out (`arm.cargo_drop_*`), tilted by the first tilt × azimuth that plans. The box (182 mm) is shorter than a sock (200 mm): a full box can still shed one over the rim.
- **Speed**: `watch` runs the arm at `--arm-speed 1.4` (the motors' max, as the tests and the bench do); ~40 s of sim time per sock; 3 scenes take ~4–5 min wall on 3 workers. A pile in the box costs ~5x per step with `sim.cloth_collisions`.
- **Sim nondeterminism**: the loop's thread timing changes when the arm gets its next command, so a seed doesn't replay exactly.
- **Not done**: A6 (dashboard front end), floor textures other than parquet (A0).

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

- The front end (`frontend/`) has the `load` / `unload` modes and the tabs `/load`, `/unload` (both on the old run page, `AutoPage`), but still the table's 3D view and phase strip. Update the rest of the shared parts (the 3D view drawing `parts` generically, the phases), then the load panel. Tell B when the shared part is merged: B's panel builds on it.
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
- 2026-09-27 (from B): shared front end: the `auto` mode replaced by `load` / `unload` (the back end's modes since stage 0; `/` crashed on `TAB_MODES["unload"]`), tabs Load / Unload both on the old run page `AutoPage`, `isRunMode()` in `api.ts`. The 3D view, the phase strip and the load panel are still A6's.
- 2026-09-27 (from B): shared `ItemSpec` takes the cloth's `young` and `thickness_m`; the defaults are the base's (1e5 Pa, 4 mm), so your scene is unchanged. B's socks are limp (1e4 Pa, 1 mm) so a held one hangs into the camera's view ([D-044](../decisions.md)); worth trying for A's too if the held sock matters to you.
- 2026-09-27 (from B, [D-043](../decisions.md)): shared `sim.layout` tool: the cargo zone is B's pick zone (16 mm off the walls, corners cut by 45 mm, `grasp_depth_mm` 8; the check plans a pick at 0/90/45/135° and needs one), and `rig.yaml` has `show_held` (found with a MuJoCo collision check, `layout.collision_check`). The station (`sim.layout.laundry`) moved in front of the rover: 190 mm boxes at x 290, y ±220 / 0, so with both scenes (`run --sim`'s default) the bins stand in your floor ring; your bench and watch use only `load`. B's `sim.camera_mount_T` is gone in favor of your `camera_T_link5_cam`.
- 2026-09-27: the cargo box moved to the arm's right ([D-045](../decisions.md)): `sim.layout.cargo.center_mm` (−50, −200); `rig.yaml` recomputed (look_cargo, cargo_*, scan poses, both zones, views, keep-out changed). Rebase and rerun your layout-dependent tests.
- 2026-09-27 (shared, [D-049](../decisions.md)): the arm stands turned on the rover, joint 1 = 0 to its right: `arm.base_yaw_deg: -90` turns FK / IK into the rover's frame (the layout stays as it was); `rest` faces forward; `rig.yaml` recomputed (every pose's joint 1 changed). Seed joint 1 with `kinematics.joint1_toward(x, y)`, never `-atan2(y, x)`.
- 2026-09-27 (shared, [D-050](../decisions.md)): the base leans ~6° forward on the rover: `arm.base_tilt_deg` levels the frame, `floor_z_mm` −192, the floor zone stops the fingertips 5 mm above it; the cargo box is at (−220, −190) (photo), the PSU on the left; `rig.yaml` recomputed. The rig's auto white balance turns white socks pink: `python -m sorter.camera.wb` fixes it per room. Gripper 4.5 / 2.5 Nm, `empty_below` 0.02.


- 2026-09-27: shared code touched by stage N ([D-046](../decisions.md)): the root config has a `nav` section, `create_app(..., nav=)` mounts `/api/nav/*` and the `/rover` page, the front end has a Rover tab (`TopBar.tsx`, `main.tsx`), `pyproject.toml` an optional `nav-hw` extra (depthai). Nothing else in the shared code changed.
- 2026-09-27 (shared, [D-050](../decisions.md)): the cargo box is at (−185, −225) (the wrist camera saw its corner); the Leo's raised top cover is a keep-out block (`sim.layout.equipment.leo_top`); `kinematics.camera_look` refuses a view with a keep-out box in its line of sight, and `look_cargo` looks from straight above; `rig.yaml` recomputed.
- 2026-09-27 (from the rig): an empty gripper reading after a pick is checked with the camera (`_still_on_floor`): a squeezed sock reads like nothing, and the loop went on scanning with the sock in the gripper.


