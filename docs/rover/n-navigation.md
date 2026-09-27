# Rover stage N: Navigation

**Status:** todo · **Owner:** — · **Branch:** —
**Depends on:** [0: Preparation](0-preparation.md) (done). Runs in parallel with [A](a-loading.md) and [B](b-unloading.md).

## Goal

The rover drives in the simulator: it finds socks in a room, stops so a sock is within the arm's reach (hand-off to A), and docks at the unload station precisely enough for B's fixed bin poses ([D-035](../decisions.md)). If the real rover's navigation stays with the other team, this stage stands in for it in the sim, and N4 plus the interface remain ours.

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| N1 | Driving rover in MuJoCo | todo | — | The rover base moves (x, y, yaw from wheel speeds, some slip) in a room with walls, a few obstacles, socks on the floor and the station |
| N2 | Finding socks from afar | todo | — | Socks are detected while driving (the arm holds the camera in a search pose) and located on the floor |
| N3 | Approach and stop | todo | — | The rover stops with the sock inside the arm's reach (the stop requirement from `sim.layout`) on ≥ 95 % of approaches |
| N4 | Docking at the station | todo | — | Final approach on a marker (e.g. ArUco) at the station; parking error within what B's drop poses tolerate |
| N5 | Room coverage | todo | — | A search pattern that covers the room and ends when it is clear or the cargo box is full |

## Your lane

To be fixed when the stage starts: likely `src/sorter/rover/` (new), a room scene `src/sorter/sim/scenes/room/`, `tests/rover/`. The moving base changes the shared base scene: a contract change (AGENTS.md).

## Open questions

- Who navigates the real rover: us or the other team?
- The rover's sensors besides the wrist camera (lidar, own camera, wheel odometry)?
- The demo space: a room or a marked area on the floor?

## Log

- 2026-09-27: stage defined ([D-035](../decisions.md)).
