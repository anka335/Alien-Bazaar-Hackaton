# Rover stage N: Navigation

**Status:** todo · **Owner:** — · **Branch:** —
**Depends on:** [0: Preparation](0-preparation.md) (done). Runs in parallel with [A](a-loading.md), [B](b-unloading.md) and [F](f-far-detection.md); drives to F's targets.

## Goal

The rover drives in the simulator: it drives to the socks that [F](f-far-detection.md) sees from afar, stops so a sock is within the arm's reach (hand-off to A), and docks at the unload station precisely enough for B's fixed bin poses ([D-035](../decisions.md)). If the real rover's navigation stays with the other team, this stage stands in for it in the sim, and N4 plus the interface remain ours.

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| N1 | Driving rover in MuJoCo | todo | — | The rover base moves (x, y, yaw from wheel speeds, some slip) in a room with walls, a few obstacles, socks on the floor and the station |
| N2 | Driving to a target | todo | — | Given F's target (arm frame, at the time of the frame), the rover turns it into the room frame with its pose and drives there around obstacles; until F is ready, targets come from the sim's ground truth |
| N3 | Approach and stop | todo | — | Close to the target, the rover re-checks it and stops with the sock inside the arm's reach (the stop requirement from `sim.layout`) on ≥ 95 % of approaches |
| N4 | Docking at the station | todo | — | Final approach on a marker (e.g. ArUco) at the station; parking error within what B's drop poses tolerate |
| N5 | Room coverage | todo | — | Where to drive when F sees no sock; ends when the room is clear or the cargo box is full |

## Your lane

To be fixed when the stage starts: likely `src/sorter/rover/` (new), a room scene `src/sorter/sim/scenes/room/`, `tests/rover/`. The moving base changes the shared base scene: a contract change (AGENTS.md).

## Open questions

- Who navigates the real rover: us or the other team?
- The rover's sensors besides the wrist camera (lidar, own camera, wheel odometry)?
- The demo space: a room or a marked area on the floor?

## Log

- 2026-09-27: stage defined ([D-035](../decisions.md)).
- 2026-09-27: far detection moved to its own stage F ([D-036](../decisions.md)); N2 is now driving to F's target.
