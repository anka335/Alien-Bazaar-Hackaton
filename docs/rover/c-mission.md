# Rover stage C: Full mission

**Status:** todo · **Owner:** — · **Branch:** —
**Depends on:** [A: Loading](a-loading.md), [B: Unloading](b-unloading.md), [F: Far detection](f-far-detection.md), [N: Navigation](n-navigation.md).

## Goal

The whole idea end to end in the simulator: drive, collect socks at several stops, return to the station, unload, every sock in its bin ([D-035](../decisions.md)).

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| C1 | Rover interface + interlock | todo | — | A `Rover` protocol with a sim backend (`wait_stopped`, `release`, move requests); the arm moves only while the rover is stopped, the rover drives only with the arm stowed |
| C2 | Stow pose | todo | — | A low, compact driving pose, reachable from every other pose, clear of the keep-out |
| C3 | Mission loop | todo | — | stop → load → stow → release … → station → unload → stow, with holds and errors handled; a new operator mode or a mode above load / unload |
| C4 | Repositioning | todo | — | A sock seen but out of reach makes the arm ask the rover to move (~10–15 cm) instead of skipping it |
| C5 | Full cargo box | todo | — | A full compartment sends the rover to unload even with socks left on the floor |
| C6 | Mission benchmark | todo | — | N seeded missions headless: socks in the right bin, left behind, time per mission |

## Open questions

- Must the arm be stowed while driving, or may it hold any pose?
- How many stops per trip: until the cargo box is full, or a fixed number?

## Log

- 2026-09-27: stage defined ([D-035](../decisions.md)).
- 2026-09-27: shared code touched by stage N ([D-040](../decisions.md)): the root config has a `nav` section, `create_app(..., nav=)` mounts `/api/nav/*` and the `/rover` page, the front end has a Rover tab (`TopBar.tsx`, `main.tsx`), `pyproject.toml` an optional `nav-hw` extra (depthai). Nothing else in the shared code changed.
