# Rover stage N: Navigation

**Status:** in progress · **Owner:** Viacheslav + Claude · **Branch:** `feat/rover-nav`
**Depends on:** [0: Preparation](0-preparation.md) (done). Runs in parallel with [A](a-loading.md), [B](b-unloading.md) and [F](f-far-detection.md); drives to F's targets.

## Goal

The rover drives in the simulator: it drives to the socks that [F](f-far-detection.md) sees from afar, stops so a sock is within the arm's reach (hand-off to A), and docks at the unload station precisely enough for B's fixed bin poses ([D-035](../decisions.md)). If the real rover's navigation stays with the other team, this stage stands in for it in the sim, and N4 plus the interface remain ours.

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| N1 | Driving rover in MuJoCo | review | Viacheslav + Claude | The rover base moves (x, y, yaw from wheel speeds, some slip) in a room with walls, a few obstacles, socks on the floor and the station |
| N2 | Driving to a target | todo | — | Given F's target (arm frame, at the time of the frame), the rover turns it into the room frame with its pose and drives there around obstacles; until F is ready, targets come from the sim's ground truth |
| N3 | Approach and stop | review | Viacheslav + Claude | Close to the target, the rover re-checks it and stops with the sock inside the arm's reach (the stop requirement from `sim.layout`) on ≥ 95 % of approaches |
| N4 | Docking at the station | todo | — | Final approach on a marker (e.g. ArUco) at the station; parking error within what B's drop poses tolerate |
| N5 | Room coverage | todo | — | Where to drive when F sees no sock; ends when the room is clear or the cargo box is full |

## What exists ([D-037](../decisions.md))

- `src/sorter/nav/`: the Leo Rover 1.9 in its own MuJoCo world (not the arm scene), its firmware emulated, an OAK-D with stereo-like depth, seeded scenarios (`python -m sorter.nav scenarios`), the command set (`python -m sorter.nav commands`), sock detectors (`classic`, `sam3`, `seg` = sim oracle), the approach algorithm (`controller.py`), a CLI for one-command-per-process episodes and a benchmark. How to run: [README → Rover navigation sim](../../README.md#rover-navigation-sim).
- The real rover and camera behind the same commands: `real_leo.py` (rosbridge), `real_oakd.py` (depthai v3), `real.py` (session, `hw-check`, `real do|look|detect|auto`, `serve --real`). Tested against a fake rosbridge only; not yet on the hardware.
- N3's goal zone is `nav.goal`: the sock's center 0.33–0.53 m ahead of the rover's center, ±0.10 m sideways (a placeholder for the stop requirement from `sim.layout` until the arm sits on the Leo). Any sock counts.
- The Rover tab (`/rover`) runs a live episode, or the real rover with `serve --real`.

### Benchmark (the approach algorithm, `classic` detector, seeds 0–9 per scenario)

| easy | near | side | behind | far | obstacle | clutter | wall | dim | plain | bunched | multi | all |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 100 % | 100 % | 100 % | 100 % | 100 % | 70 % | 70 % | 100 % | 80 % | 100 % | 90 % | 100 % | 92 % |

No collisions. On unseen seeds 10–19 (plus `random`): 91 %, one collision (obstacle). The `sam3` detector (the real SAM3 service, prompt `a sock lying on the floor`) reached 80 % on easy / multi / bunched / clutter seeds 0–4. Reproduce: `python -m sorter.nav bench --scenarios easy,near,side,behind,far,obstacle,clutter,wall,dim,plain,bunched,multi --seeds 0-9 --detector classic`.

## Your lane

`src/sorter/nav/`, `tests/nav/`, `frontend/src/pages/RoverPage.tsx`, `frontend/src/styles/rover.css`, `config/default.yaml` → `nav`.

## Notes & risks

- Skid-steer turns in MuJoCo slip more than the real tyres: the sim turns at ~55 % of the commanded rate (the commands close the loop on the gyro).
- The OAK-D (not Pro) is passive stereo: plain floors give little depth; commands fall back to the floor-plane ray through the pixel.
- No GPU on the dev machine: EGL renders on the CPU; a bench worker needs ~0.3 GB; `bench` caps its workers by free memory.
- Left to do: obstacle and clutter failures are give-ups (the sock hidden behind furniture, search loops); DONE is decided from one frame (a second, consistent frame would stop the last false claims in dim light); nothing guards turns in place against things beside the rover (the camera doesn't see them); the real hardware is untested.
- Camera settings measured on the sim: pitch 20–25° (35° loses the floor beyond ~2 m, 15° drops the pick zone off the image), depth mode `extended`.

## Open questions

- Who navigates the real rover: us or the other team?
- The rover's sensors besides the wrist camera (lidar, own camera, wheel odometry)?
- The demo space: a room or a marked area on the floor?

## Log

- 2026-09-27: commands, detector and algorithm tuned by 10 parallel agents and merged (benchmark 43 % → 92 %); real rover (rosbridge) and OAK-D (depthai) backends, `hw-check`, `serve --real`.
- 2026-09-27: N1 and N3 started in `sorter.nav` on `feat/rover-nav` ([D-037](../decisions.md)).

- 2026-09-27: stage defined ([D-035](../decisions.md)).
- 2026-09-27: far detection moved to its own stage F ([D-036](../decisions.md)); N2 is now driving to F's target.
