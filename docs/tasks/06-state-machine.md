# Block 6: State Machine

**Status:** review · **Owner:** Softjey + Claude · **Branch:** `block/06-state-machine`
**Owned paths:** `src/sorter/orchestrator/`, `tests/orchestrator/`, `config/default.yaml` → `state_machine`

## Goal

The main loop that ties all modules together and survives the usual failures without human help.

## Scope

- [x] Implement the loop exactly as in [architecture.md → Main loop](../architecture.md#main-loop-state-machine-walkthrough): observation-driven, background first ([D-008](../decisions.md)).
- [x] Take every decision frame through `observer.observe(zone)`.
- [x] Resolve `pending` in `SENSE_BG`: missed grasp from the box (→ `avoid`), verified drop (blob count down → counter), failed pick from the background (retry).
- [x] Box handling: `avoid` list, `NO_GRASP` → clear `avoid` once, `EMPTY` confirmed `empty_confirmations` times → `DONE`, `TargetRejected` → `avoid` and re-detect on the same observation.
- [x] `failures ≥ max_consecutive_failures` → `ERROR`, paused.
- [x] Controls via the Hub: start / pause / resume / step (one phase) / stop / reset. `HOLD` arrives as `EStopped` from the blocked arm call → `HELD`.
- [x] Publish `Status` on every phase change and a `Decision` after every sense phase.
- [x] Run log: every observation and decision to `data/runs/<run_id>/` (`sorter.core.io`).
- [x] Measure the cycle time (`last_cycle_s`).
- [x] Tests on the simulator: happy path, missed grasp from the box, failed drop, double grasp, `TargetRejected`, hold + reset, too many failures.

## Depends on / Unblocks

- Depends on: 0 (contracts, Hub, simulator). Integration needs 2 to 5.
- Unblocks: 7 (live status), 8.

## Acceptance criteria

- On the simulator: sorts all items; each failure path is covered by a test.
- On the real rig: completes a full box without manual intervention (except hold).

## Notes & risks

- Keep the state machine dumb and explicit. Decisions like "which point" belong to vision, and "how to move" belongs to the arm.
- Low-confidence color: sort into the most likely class and log a warning (current policy).
- An item that falls out of the gripper on the way to a bin is not seen by the camera and is still counted. Accepted for the demo.
- Cycle time is measured between two `LOOK_BG` starts while running; a cycle with a pause, hold, error, or step is not timed.
- Tests (`tests/orchestrator/`) script failures by wrapping sim methods (`patch_once`), so each path is deterministic. The shared `sim_config` fixture turns run logs off; tests that check them use `tmp_path`.

## Open questions

_None._

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: loop redesigned: background first, blob-count drop verification, `avoid` list, `look(zone)` via the Observer, `HELD` phase (D-006, D-008, D-009).
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/orchestrator/config.py` (placeholder). A placeholder loop is in `sorter/orchestrator/state_machine.py` (`StateMachine(system).run(stop)`, used by `app.py` and the smoke test `tests/sim/test_smoke.py`); replace it, keep the entry point. See architecture.md → Wiring.
- 2026-09-25: loop implemented on the simulator with tests for every failure path. Run log added (`sorter/orchestrator/runlog.py`, format in architecture.md → Recording format). New config keys `low_confidence`, `save_runs`, `runs_dir`. Unexpected exceptions → `ERROR` instead of killing the thread. `tests/conftest.py` (block 0): `sim_config` sets `save_runs: false`.
- 2026-09-25 (block 4): `classify` of the real color classifier raises `SegmentationError` (a `SorterError`) when the SAM3 service fails; the loop already turns it into `ERROR`, paused (D-013).
- 2026-09-26 (block 5): the sim arm is the real controller with kinematics (D-014). `world.looking_at` is derived from the joints; `ArmController` is unchanged. The sim tests run with `sim.time_scale: 0` (instant).
- 2026-09-26 (Softjey + Claude): `run --sim` runs the loop on the physics simulator with the real vision and calibration (D-016); about a minute per item at `sim.realtime: 0`.
- 2026-09-26 (Softjey + Claude): operator modes (D-026): outside the auto mode the Hub gives the state machine no commands (it idles); the dashboard leaves auto only while the state machine is idle.
