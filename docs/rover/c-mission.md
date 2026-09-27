# Rover stage C: Full mission

**Status:** in progress · **Owner:** Softjey + Claude · **Branch:** `main`
**Depends on:** [A: Loading](a-loading.md), [B: Unloading](b-unloading.md), [F: Far detection](f-far-detection.md), [N: Navigation](n-navigation.md).

## Goal

The whole idea end to end in the simulator: drive, collect socks at several stops, return to the station, unload, every sock in its bin ([D-035](../decisions.md)).

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| C1 | Rover interface + interlock | in progress | Softjey + Claude | A `Rover` protocol with a sim backend (`wait_stopped`, `release`, move requests); the arm moves only while the rover is stopped, the rover drives only with the arm stowed |
| C2 | Stow pose | todo | — | A low, compact driving pose, reachable from every other pose, clear of the keep-out |
| C3 | Mission loop | in progress | Softjey + Claude | stop → load → stow → release … → station → unload → stow, with holds and errors handled; a new operator mode or a mode above load / unload |
| C4 | Repositioning | todo | — | A sock seen but out of reach makes the arm ask the rover to move (~10–15 cm) instead of skipping it |
| C5 | Full cargo box | in progress | Softjey + Claude | A full compartment sends the rover to unload even with socks left on the floor |
| C6 | Mission benchmark | todo | — | N seeded missions headless: socks in the right bin, left behind, time per mission |

## What exists ([D-052](../decisions.md))

- `src/sorter/mission/`: `run_mission()`, CLI `uv run python -m sorter.mission --seed 0 [--out DIR --video] [--capacity 6] [--no-arm]`. Drive in the nav world → load in a fresh arm world with the socks in reach placed where they lie → loaded socks leave the nav world → … → drive home on the odometry → the station's AprilTags. The two worlds run in turn, so the arm only moves while the rover stands (C1's interlock, by construction; no `Rover` protocol yet). `capacity` ends the collecting (C5). Unload at the station is B's (not called).
- Scenario `mission` (`sorter.nav.scenario`): 4 socks, 1 obstacle, 1 distractor, the station against the −x wall with tags 14 13 12 (8 cm, `nav.boxes.station_tag_m`).
- Numbers: `--no-arm` seeds 0–2: 4/4, 4/4, 3/4 socks reached (seed 2: one not found), station gap 13–15 cm (target 15). With the arm: seed 0 4/4 in the cargo box (57 s wall, station 13.4 cm), seed 1 4/4 (light, dark, 2 colored; 36 s, station 14.5 cm); ~6–10 s wall per stop.
- **Mission tab** (`/mission`, `sorter.mission.control`): START MISSION / STOP. On the rig (`uv run python -m sorter run`): the real Leo (rosbridge, OAK-D, SAM3 if configured) + the real arm in one process: per sock the fast approach stops the bumper `gap_m` (10 cm) before it, the load run picks what is in reach and ends at home, only then the rover drives on; a load error or hold ends the mission with the rover stopped. The station: its tag 13 is looked for at the start (**start the rover facing the station, 1–2 m away**), the rover drives back 1.3 m in front of it on the odometry, then GO TO BOX (tag not found: back off 0.5 m, again). On `run --sim`: `run_mission` behind the same buttons.
- The real-rover loop against the nav sim (`RealSession` over `RoverSim`, a stand-in arm; `tests/mission/test_control.py`), started 1.5 m in front of the station: seeds 0–2 station 3/3 (14–15 cm), socks 4/4, 3/4, 2/4 (the fast approach sometimes stalls at a wall: "obstacle ahead" until it gives up). Without the station seen at the start, the sim's odometry drifted ~1.3 m over a mission: going back to the start point missed the station.
- Not yet on the hardware: never run with the arm on the Leo. Not yet: the arm world is fresh per stop in the sim; no stow pose (C2), no repositioning (C4).

## Open questions

- Must the arm be stowed while driving, or may it hold any pose?
- How many stops per trip: until the cargo box is full, or a fixed number?

## Log

- 2026-09-27: the Mission tab: one Start for the real Leo + the real arm (or the sim); shared code: `create_app(..., mission=)`, `app.run` makes the `MissionControl`, `/mission` in `PAGES`, the front end's TopBar / main.
- 2026-09-27: mission loop on the sim: `sorter.mission` hands over between the nav and arm worlds ([D-052](../decisions.md)).
- 2026-09-27: stage defined ([D-035](../decisions.md)).
- 2026-09-27: shared code touched by stage N ([D-046](../decisions.md)): the root config has a `nav` section, `create_app(..., nav=)` mounts `/api/nav/*` and the `/rover` page, the front end has a Rover tab (`TopBar.tsx`, `main.tsx`), `pyproject.toml` an optional `nav-hw` extra (depthai). Nothing else in the shared code changed.
