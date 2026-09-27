# Rover stage F: Far detection

**Status:** todo · **Owner:** — · **Branch:** —
**Depends on:** [0: Preparation](0-preparation.md) (done). Runs in parallel with [A](a-loading.md), [B](b-unloading.md) and [N](n-navigation.md); [N](n-navigation.md) drives to its targets.

## Goal

See socks from afar: from a search pose (the arm raises the wrist camera and looks ahead), find every sock 0.5–3 m away on the floor and say where it is, so the rover can drive there ([D-036](../decisions.md)). Stage A's floor detector works at arm's reach, looking straight down; this one works at a distance, looking at a slant, where a sock is a few dozen pixels.

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| F1 | Far scene | todo | — | A sim scene with the rover standing still and socks scattered 0.5–3 m around it, on several floors, with distractors (other objects, shadows) |
| F2 | Search poses | todo | — | Named poses that point the camera ahead / to the sides from high up; together they cover the area around the rover |
| F3 | Far detector | todo | — | Socks found from the search poses: ≥ 90 % recall up to 2 m, few false positives; each with its position on the floor (arm frame) and a rough color |
| F4 | Target list | todo | — | Detections from several views and frames merged, duplicates dropped, ranked (nearest first); the output N drives to |
| F5 | Benchmark | todo | — | Seeded scenes headless: recall and false positives by distance, position error on the floor |

## Contract with N

F gives targets on the floor in the arm frame (x, y, how sure, rough color) at the time of the frame; N turns them into the room frame with the rover's pose and drives there. N re-checks close up; the final stop is A's floor detector.

## Your lane

To be fixed when the stage starts: likely `src/sorter/far_detector/` (new), a scene `src/sorter/sim/scenes/far/`, `tests/far_detector/`. New search poses go through `sim/layout.py` (shared).

## Open questions

- Depth is poor beyond ~2 m on the D435i: floor-plane intersection from the pixel instead of depth?
- Does the rover have its own forward camera to use instead of the wrist camera?

## Log

- 2026-09-27: stage defined ([D-036](../decisions.md)).
- 2026-09-27: shared code touched by stage N ([D-046](../decisions.md)): the root config has a `nav` section, `create_app(..., nav=)` mounts `/api/nav/*` and the `/rover` page, the front end has a Rover tab (`TopBar.tsx`, `main.tsx`), `pyproject.toml` an optional `nav-hw` extra (depthai). Nothing else in the shared code changed.
