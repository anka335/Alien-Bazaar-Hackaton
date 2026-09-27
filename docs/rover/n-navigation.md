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

## What exists ([D-046](../decisions.md))

- `src/sorter/nav/`: the Leo Rover 1.9 in its own MuJoCo world (not the arm scene), its firmware emulated, an OAK-D with stereo-like depth, seeded scenarios (`python -m sorter.nav scenarios`), the command set (`python -m sorter.nav commands`), sock detectors (`classic`, `sam3`, `seg` = sim oracle), the approach algorithm (`controller.py`), a CLI for one-command-per-process episodes and a benchmark. How to run: [README → Rover navigation sim](../../README.md#rover-navigation-sim).
- The real rover and camera behind the same commands: `real_leo.py` (rosbridge; odometry from `merged_odom` or wheels + IMU), `real_camera.py` (the OAK-D's ROS driver over rosbridge, or none), `real_oakd.py` (a local OAK-D, depthai v3), `real.py` (session, `hw-check`, `real do|look|detect|auto`, `serve --real`). Connected to our Leo (ROS 2, namespace `/leo/`, `merged_odom` silent, so wheels + IMU): odometry at rest verified; driving by the buttons not yet verified by us.
- `hunt.py`: `hunt_socks()`, one call for the whole search-and-approach on the sim or the real Leo (`python -m sorter.nav hunt [--real]`): the approach algorithm per sock, then `on_reached`, then the next sock; reached socks (odometry position, 0.3 m) are ignored afterwards. Without an arm the rover may drive over a reached sock on the way to the next.
- The fast approach (`hunt.approach_sock`, the Rover tab's **RUN ROBOT**, `hunt --gap 0.05`): seek, one fast leg, then turn + drive until the bumper is `gap_m` before the sock's near edge (nearest mask depth point / floor ray through the mask's bottom row; the last ~15 cm on odometry). Sim, `classic`, 7 scenarios × seeds 0–2, target 5 cm: default mount 21/21, true gap 3.3–10.7 cm (mean 4.8); the measured real mount (`local.yaml`, 12.5° down) 18/21.
- `boxes.py`: cardboard boxes marked with AprilTags (36h11, 3.5 cm tags; ids 14 13 12): tag poses by `solvePnP` from the corners (matches the depth to 1-3 cm at 1.1 m), the boxes' face normal from the line through the tags (a single small tag's own normal is noisy), `remember_boxes` saves the layout relative to the target tag, `approach_box` searches (turns, a second look first: the RGB's autofocus hunts after the start), lines up in front of the face when seen obliquely, then drives in camera-checked steps to `stop_m` (the obstacle guard lowered to just short of it: `forward(..., guard_m)`). Rover tab: GO TO BOX, Remember boxes in view. First real run: 1.08 m → stopped 0.17 m from tag 13 (target 0.15) in 7 commands, 11 s.
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

- 2026-09-27: the OAK-D's first ~1.5 s of frames are nearly black (auto-exposure): `RealOakD` now waits for it to settle. The camera's pitch measured 16.1° at 13:00 (12.5° in the morning): the mount moves, re-check with `hw-check`.
- 2026-09-27: RUN ROBOT on the real Leo (SAM3, the default there; classic misses a sock in dim light) reached the sock. Found: the laptop roamed off `LeoRover-9a1f` onto another saved Wi-Fi mid-run (the `keepalive ping timeout` error): rosbridge now reconnects and the odometry stream is the liveness check (no odometry 1 s while driving: fault). The real rover coasts ~0.4 s after a stop (0.53 m → 0.63 m, 10° → 18°): `nav.real.stop_lead_s` stops early by that. RUN ROBOT stops 30 cm before the sock by default.
- 2026-09-27: first real run of `hunt --real`: the OAK-D (depthai, on the laptop) gave almost no depth (sparse stripes, no floor), so the obstacle guard stopped every move early, then the device crashed; later the odometry fault stopped the rover (commanded left turn, wheels+IMU said right; a wheel on a cable?). Check depth in `hw-check` before driving.
- 2026-09-27: `hunt_socks` (search and approach several socks, sim and real); the sim builds with a camera mounted below the top plate (the measured real mount in `config/local.yaml`).
- 2026-09-27: commands, detector and algorithm tuned by 10 parallel agents and merged (benchmark 43 % → 92 %); real rover (rosbridge) and OAK-D (depthai) backends, `hw-check`, `serve --real`.
- 2026-09-27: N1 and N3 started in `sorter.nav` on `feat/rover-nav` ([D-046](../decisions.md)).

- 2026-09-27: stage defined ([D-035](../decisions.md)).
- 2026-09-27: far detection moved to its own stage F ([D-036](../decisions.md)); N2 is now driving to F's target.
- 2026-09-27: the ROS 2 track has a MuJoCo Leo Rover (official `leo_description` model, firmware-like `cmd_vel`, wheel + gyro odometry, cameras) in `ros2_ws/src/rover_nav/sim` ([D-039](../decisions.md), [brief](ros2-navigation.md)); N1 can reuse its model and meshes.
