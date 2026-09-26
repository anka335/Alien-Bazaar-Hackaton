# Rover stage B: Unloading

**Status:** todo · **Owner:** — · **Branch:** —
**Depends on:** [0: Preparation](0-preparation.md). Runs in parallel with [A: Loading](a-loading.md).

## Goal

The rover is parked at the unload station. The arm empties the cargo box compartment by compartment: every sock from compartment *c* goes into laundry bin *c*. The color is already known from loading ([D-032](../decisions.md)), so there is no color classification here.

```text
for c in (light, dark, colored):
  LOOK_CARGO(c) → SENSE_CARGO → PICK_FROM_CARGO → DROP_TO_LAUNDRY(c) ─┐
       ▲                                                              │
       └──────────────────────────────────────────────────────────────┘
       empty × N → next compartment
DONE → STOW
```

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| B1 | Station and parking tolerance | todo | — | All bins reachable with the parking noise from P2 |
| B2 | Compartment detector | todo | — | Correct "empty / grasp" per compartment on the benchmark |
| B3 | Pick and drop with verification | todo | — | No sock lands in the wrong bin; a missed pick is retried |
| B4 | Unload state machine | todo | — | Loop runs end to end on the sim with holds and errors handled |
| B5 | Dashboard: unload panel | todo | — | Compartment view, grasp point, per-bin counters visible |
| B6 | Benchmark pass | todo | — | ≥ 95 % of socks unloaded, 0 in a wrong bin, over the unload benchmark |

### B1: Station and parking tolerance

- Bin positions in the rover frame, from config (`unload.bins`).
- Park noise (x, y, yaw) from the scenario generator. If the drop poses can't absorb it, add a correction (e.g. a marker on the station seen from a look pose) and record the choice in `decisions.md`.

### B2: Compartment detector

- Reuse block 3's depth box detector with one ROI per compartment and parameters for socks (small, light, thin).
- `EMPTY` confirmed `unload.empty_confirmations` times before moving to the next compartment.
- `avoid` list per compartment, as in the current loop.

### B3: Pick and drop with verification

- Pick from the compartment, drop over bin *c*.
- Verify by looking at the compartment again (observation-driven, [D-008](../decisions.md)): fewer socks → counted; same → failure, retry.
- A sock that fell back into another compartment ends up in the wrong bin: the detector must not look outside its ROI, and the drop path must not cross other compartments.

### B4: Unload state machine

- Own module `src/sorter/orchestrator/unload.py`; the mode `unload` in the Hub.
- Starts only with the rover stopped at the station (interlock from P1); ends in `stow`.
- Hold, pause, step and errors as in the current loop.

## Owned paths (tentative)

`src/sorter/box_detector/`, `src/sorter/orchestrator/unload.py`, `tests/box_detector/`, `tests/orchestrator/test_unload.py`, the unload panel in `frontend/src/`, `config/default.yaml` → `box_detector`, `unload`.

Shared with A (arm, sim, core): change only through the contract rules in [AGENTS.md](../../AGENTS.md).

## Exit criteria

B1–B6 done on the sim. Hardware comes in stage 3.

## Open questions

- Compartment walls: high enough that a dropped sock never bounces into the next one? Decide in P2 with the scene.

## Requests from other blocks

_None yet._

## Log

- 2026-09-27: stage defined ([D-032](../decisions.md)).
