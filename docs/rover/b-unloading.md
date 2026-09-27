# Rover stage B: Unloading

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `claude/b-unloading-stage-d746d4`
**Depends on:** [0: Preparation](0-preparation.md) (done). Runs in parallel with [A: Loading](a-loading.md).

## Goal

The rover is parked at the unload station. The arm takes every sock out of the cargo box, tells its color and drops it into the laundry bin of that color. The rover stands still; we don't talk to it ([D-032](../decisions.md), [D-034](../decisions.md)).

**Make it work perfectly in the simulator, as close to reality as you can get it:** a realistic scene (cargo box, socks in it, the station), a robust loop, and a benchmark that proves it. Hardware comes later.

## Where it stands

- **Geometry: the committed layout** (the rover measured, [D-042](../decisions.md)): one cardboard box 182 × 182 mm inside, 75 deep, to the arm's right and a bit behind it ([D-045](../decisions.md)), so the color is told at unload; the station ([D-043](../decisions.md), not measured): three boxes like it (190 mm, 75 high) on the floor in a row in front of the rover. `python -m sorter.sim.layout --write` computes B's part of `rig.yaml` too: the cargo pick zone and `show_held`. The real run (`python -m sorter run --mode unload`) and the sim use the same rig.
- **Camera:** the sim camera sits at the rig's measured hand-eye mount ([D-041](../decisions.md)). It doesn't see the fingertips; the held sock is seen from the `show_held` pose when it hangs ≥ 55–60 mm below the fingers, so the unload scene's socks are limp like real ones (`sim.unload.sock_young` / `sock_thickness_mm`, [D-044](../decisions.md)).
- **Loop** (`orchestrator/unload.py`, only the wrist camera): the bins are found once per run (`box_detector/station.py`: a square ring matched to the wall points); per sock the top of the pile (`box_detector/cargo.py`), the pick with finger directions tried in turn (open fingers kept off other socks, the gripper opened to ±27 mm), a shake, a second look into the box, `show_held` + the gripper's opening (anything held?); the color from the sock seen hanging (side-lit thresholds), else the box's best match (the one sock gone; of several, the one nearest the grasp; none gone, the grasp's target; [D-044](../decisions.md)); socks of different colors hanging → back into the box, and again; the drop into the found bin (fingers below its rim) via home, a look into the bin (only a seen drop is counted).
- **Benchmark:** `uv run python -m sorter.sim.scenes.unload.bench -n 20` (seeded; the station ±30 mm / ±5°, each bin ±15 mm / ±8°; judged by the sim's ground truth; report in `data/bench/`). Latest result in the *Log*.
- Commands: `python -m sorter.sim.scenes.unload.demo` (one scenario, live), `.preview [--view]`.

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

- Done: the measured rover, the station in front of it, `sim.unload.station_mm` / `station_deg` / `bin_mm` / `bin_deg` (parking and placement noise), socks of mixed colors piled in the box (`sim.unload.cargo`, `sock_mm`).
- Open: socks as A makes them (`load.scene.sock_shape`, limp); the station's real place and bins; the box's center and the wheel diameter aren't measured ([D-042](../decisions.md)).

### B2: Detectors

- Known sim gaps that show up here: the fingers may pinch the sock under the target (the second look into the box tells which one went) and drag a neighboring sock out of the box. `sim.cloth_collisions` (on by default) makes the pile stack instead of pass through itself, but a 6-sock pile then runs at ~0.05× real time (without: ~0.25×; the solver of ~1150 cloth DOF is ~80 % of a step), so the demo and the benchmark (`unload_config`) turn it off; with it on, the scene starts the socks 45 mm apart (at 6 mm they start tangled and fly out of the box).

### B3: Pick and drop with verification

- The held sock's color comes from the box, lit from above: hanging from the gripper it is seen from the side, in shade, and light socks look dark there.
- A drop counts when the look into the bin sees its cloth grow by ≥ 0.3 mm of mean height over the bin floor, counting only what stands > 6 mm above it (the depth over an empty floor is off by up to ~5 mm; a sock lying there stands 10–17 mm).

### B4: Unload loop

- In `unload.py`; the shared state machine gives commands, hold, errors, the run log. Starts at the station, ends at `home`.

### B5: Dashboard

- Wait for A6's shared front-end update (tabs, 3D view from `parts`), then add the unload panel. Rewrite the skipped `tests/dashboard/test_manual.py` for the rover poses.

### B6: Benchmark

- `N` seeded scenarios (default 50), headless, `sim.realtime: 0`. Metrics: socks unloaded, in a wrong bin, left in the cargo box, time per sock, failures by type. Report to `data/bench/<run_id>/`.

## Exit criteria

B0–B6 done on the sim.

## Open questions

- The station's real place and bins: measure them, set `sim.layout.laundry`, recompute the rig.

## Requests from other blocks

*None yet.*

## Log

- 2026-09-27: stage defined ([D-032](../decisions.md)).
- 2026-09-27: stage 0 done; the brief rewritten for what it delivered ([D-034](../decisions.md)).
- 2026-09-27 (from A): the rover layout follows its photos ([D-040](../decisions.md)): deck 160 mm above the floor, one cargo box 150 × 150 × 60 mm at (−50, 200) with no compartments (`compartment(c)` = the whole box, `location()` → `("cargo", None)`), keep-out for the wheels and the equipment behind the arm. The laundry bins didn't move. `unload.py` walks the compartments and now finds none.
- 2026-09-27 (from A, [D-041](../decisions.md)): the sim camera is the calibrated one; `look_cargo` is now ~210 mm over the box floor (tilt allowed); `rig.yaml` has `scan_1` … `scan_7`; joint moves swing joint 1 first or last or go via home (`Controller.plan_move`); the camera body is in the collision checks (`arm.link5_points_mm`); drops wait `arm.drop_settle_s`; `CARGO_MARGIN_MM` along is 50; `sim.cloth_collisions` (on) makes piles in the box stack and ~5x slower; `PhysicsWorld.location` says `laundry` only with the unload scene.
- 2026-09-27 (from A): `ArmController.drop_to_cargo` comes in high over the box, goes straight down under the rim, lets go and backs out (`arm.cargo_drop_*`); `drop_to_laundry` is unchanged. The live viewer in `load.watch` draws a copy of the sim data (as `unload.demo` does).
- 2026-09-27 (from A): the rover measured ([D-042](../decisions.md)): deck 200 mm above the floor, 300 × 185 with the arm at its front edge, body 420 × 420, the box 182 × 182 inside, 75 deep, floor z −41, rim z 34; `rig.yaml` recomputed, poses and zones changed. Rebase and rerun your layout-dependent tests.
- 2026-09-27: stages N (navigation), C (full mission), D (sim-to-real) added ([D-035](../decisions.md)): N4 docks at the station, which bounds B0's parking noise.
- 2026-09-27: B0 started: unload scene supports an undivided box; preview of the real rover's geometry.
- 2026-09-27: vision-only loop on the real rover's geometry, bench and demo (`python -m sorter.sim.scenes.unload.demo`). Bench, 8 × 6 socks, before the held-first color: 64.6 % in the right bin, 6 in a wrong one, 11 lost; bins found to 1–2 mm. The held-first color and the flatter initial pile are not benchmarked yet. Open problems: the sim's pile (cloth passes through cloth: the wrong sock pinched, the pile settles anew), neighbors dragged out of the box, the held sock seen only when it hangs ≥ 60 mm (the rig camera is off to the side) and side-lit color thresholds tuned on ~12 sim samples, ~50 s of sim per sock, the rover's dimensions are estimates, the committed layout still has stage 0's 3-compartment box.
- 2026-09-27: the loop kept putting socks back: the sim's stiff cloth stayed bunched at the fingers, below the camera's view from `show_held` (it sees ≥ 55 mm below them), and the box's "one sock gone" failed on a pile that settles anew. Fixed by limp socks in the unload scene and the box's best match as the fallback color ([D-044](../decisions.md)). Also fixed the dashboard's `/` crash (the front end still had the `auto` mode).
- 2026-09-27: on the measured rover (A's layout merged): the station moved into the committed `sim.layout.laundry` (in front of the rover, [D-043](../decisions.md)); `sorter.sim.layout` computes the cargo pick zone (corners cut by 45 mm for the bigger box) and `show_held` into `rig.yaml`, so `run --mode unload` runs on the rig without an overlay; `unload.rover` / `REAL_ROVER` removed. Not benchmarked on the new layout yet.
- 2026-09-27: the cargo box moved to the arm's right ([D-045](../decisions.md)): `sim.layout.cargo.center_mm` (−50, −200); `rig.yaml` recomputed (look_cargo, cargo_*, scan poses, both zones, views, keep-out changed). Rebase and rerun your layout-dependent tests.
- 2026-09-27: on the box on the right: the bins' look poses are the ones a move from `home` reaches (else the IK bent the tool into the base column and every run stopped at `look_cargo`); the drop check counts cloth above the depth noise (it missed most drops); the demo and the benchmark run without cloth-on-cloth collisions (the pile flew apart: the socks started tangled). Bench, 6 × 3 socks: 72 % in the right bin, 0 in a wrong one, 5 of 18 elsewhere (a missed grasp drags the sock out of the box: 14 of 32 picks took nothing and went to a bin empty), bins found to ≤ 1.8 mm, ~60 s of sim per sock.
- 2026-09-27 (from A): shared arm and layout: `arm.cargo_drop_tilt_deg` is now `cargo_drop_tilts_deg` ([20, 30, 10], the first that plans: with the box on the right only 30° reaches); `sim.layout.check` plans the drop from above; `keep_out` keeps all four wheels (the front right one sticks out from under the box: the palm hit it on a floor pick). `rig.yaml` recomputed.
- 2026-09-27 (from A): shared kinematics: the keep-out check also samples the gripper housing (`HOUSING_MM`: the finger rail and the motor, from the meshes; the sim's `palm*` geoms are the same boxes): the rail is 184 mm wide and hit the box wall and a wheel while the plan said clear. Fewer poses plan near the box; `rig.yaml` recomputed.
- 2026-09-27 (from A): **blocks unload picks**: with the gripper's real finger rail in the keep-out check (184 mm across, 64 mm above the TCP) `sim.layout.check` reports 44 problems, all `pick cargo`: the rail is wider than the box's inside (182 mm) and reaches the rim when the fingers go near the box floor. Floor picks, moves and drops pass. Needs a decision in B (rail along the diagonal only, a shallower grasp, a wider or lower box).
- 2026-09-27 (shared, [D-047](../decisions.md)): the arm stands turned on the rover, joint 1 = 0 to its right: `arm.base_yaw_deg: -90` turns FK / IK into the rover's frame (the layout stays as it was); `rest` faces forward; `rig.yaml` recomputed (every pose's joint 1 changed). Seed joint 1 with `kinematics.joint1_toward(x, y)`, never `-atan2(y, x)`.
- 2026-09-27 (shared, [D-048](../decisions.md)): the base leans ~6° forward on the rover: `arm.base_tilt_deg` levels the frame, `floor_z_mm` −192, the floor zone stops the fingertips 5 mm above it; the cargo box is at (−220, −190) (photo), the PSU on the left; `rig.yaml` recomputed. The rig's auto white balance turns white socks pink: `python -m sorter.camera.wb` fixes it per room. Gripper 4.5 / 2.5 Nm, `empty_below` 0.02.


