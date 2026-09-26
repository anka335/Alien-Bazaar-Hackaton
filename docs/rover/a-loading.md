# Rover stage A: Loading

**Status:** todo · **Owner:** — · **Branch:** —
**Depends on:** [0: Preparation](0-preparation.md). Runs in parallel with [B: Unloading](b-unloading.md).

## Goal

The rover stops next to a sock. The arm finds it on the floor, classifies its color on the spot, picks it up and drops it into the cargo compartment of that color, then stows and releases the rover. No table, no background mat ([D-032](../decisions.md)).

```text
WAIT_STOPPED → SCAN → SENSE_FLOOR → PICK_FROM_FLOOR → VERIFY → DROP_TO_CARGO(color)
                 ▲                                                   │
                 └──────────────── more socks in view ◄──────────────┤
                                                   none left ─► STOW → release()
```

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| A1 | Floor scan | todo | — | A target is found wherever the sock lies in the reachable ring |
| A2 | Floor detector + color | todo | — | ≥ 95 % correct colors across floor textures on the benchmark |
| A3 | Grasp from the floor | todo | — | ≥ 90 % successful grasps on the benchmark |
| A4 | Verify and retry | todo | — | Every failure path has a test |
| A5 | Load state machine | todo | — | Loop runs end to end on the sim with holds and errors handled |
| A6 | Dashboard: load panel | todo | — | Scan frame, found socks, per-compartment counters, rover state visible |
| A7 | Benchmark pass | todo | — | ≥ 90 % of socks end in the right compartment over the load benchmark |

### A1: Floor scan

- Walk the scan poses from P3 (a few joint-1 angles, camera down and forward).
- Merge detections from all views in arm coordinates, drop duplicates, pick the target (nearest, then most confident).
- Stop scanning early once a confident target is in view.

### A2: Floor detector + color

- Segmentation: MuJoCo instances on the sim; SAM3 with the text prompt `"sock"` on hardware (stage 3).
- Color from the mask pixels, reusing block 4's color logic (import it; changes go through its task file).
- Reject blobs that are too small, too large or not sock-shaped.
- Stateless: frame in, list of socks out ([D-008](../decisions.md) spirit).

### A3: Grasp from the floor

- Grasp point: mask center (or thickest part), gripper yaw across the sock's short axis (PCA of the mask).
- Floor height from the depth around the sock (plane fit), not from a constant.
- Pinch depth just above the floor; fingers must not hit it (P4 limit).

### A4: Verify and retry

- After the pick: look at the same spot again, and check the gripper (`likely_empty`).
- Miss → retry up to `load.max_attempts`, then skip the sock, log it, stow and release the rover.
- Counters go up only after a verified drop.

### A5: Load state machine

- Own module `src/sorter/orchestrator/load.py`; the mode `load` in the Hub.
- Interlock: nothing moves until `stopped`; `release()` only from `stow`.
- Hold, pause, step and errors as in the current loop.

## Owned paths (tentative)

`src/sorter/floor_detector/` (new), `src/sorter/orchestrator/load.py`, `tests/floor_detector/`, `tests/orchestrator/test_load.py`, the load panel in `frontend/src/`, `config/default.yaml` → `floor_detector`, `load`.

Shared with B (arm, sim, core): change only through the contract rules in [AGENTS.md](../../AGENTS.md).

## Exit criteria

A1–A7 done on the sim. Hardware comes in stage 3.

## Open questions

- Several socks at one stop: collect all in reach (planned) or only the one the rover stopped for?
- What to do with a sock we can't classify confidently: most likely class (current policy) or a fixed compartment?

## Requests from other blocks

_None yet._

## Log

- 2026-09-27: stage defined ([D-032](../decisions.md)).
