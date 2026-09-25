# Block 6: State Machine

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** `src/sorter/orchestrator/`, `tests/orchestrator/`, `config/default.yaml` → `state_machine`

## Goal

The main loop that ties all modules together and survives the usual failures without human help.

## Scope

- [ ] Implement the loop exactly as in [architecture.md → Main loop](../architecture.md#main-loop-state-machine-walkthrough): observation-driven, background first ([D-008](../decisions.md)).
- [ ] Take every decision frame through `observer.observe(zone)`.
- [ ] Resolve `pending` in `SENSE_BG`: missed grasp from the box (→ `avoid`), verified drop (blob count down → counter), failed pick from the background (retry).
- [ ] Box handling: `avoid` list, `NO_GRASP` → clear `avoid` once, `EMPTY` confirmed `empty_confirmations` times → `DONE`, `TargetRejected` → `avoid` and re-detect on the same observation.
- [ ] `failures ≥ max_consecutive_failures` → `ERROR`, paused.
- [ ] Controls via the Hub: start / pause / resume / step (one phase) / stop / reset. `HOLD` arrives as `EStopped` from the blocked arm call → `HELD`.
- [ ] Publish `Status` on every phase change and a `Decision` after every sense phase.
- [ ] Run log: every observation and decision to `data/runs/<run_id>/` (`sorter.core.io`).
- [ ] Measure the cycle time (`last_cycle_s`).
- [ ] Tests on the simulator: happy path, missed grasp from the box, failed drop, double grasp, `TargetRejected`, hold + reset, too many failures.

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

## Open questions

_None._

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: loop redesigned: background first, blob-count drop verification, `avoid` list, `look(zone)` via the Observer, `HELD` phase (D-006, D-008, D-009).
- 2026-09-25 (block 0): skeleton ready. Config model of this block in `src/sorter/orchestrator/config.py` (placeholder). A placeholder loop is in `sorter/orchestrator/state_machine.py` (`StateMachine(system).run(stop)`, used by `app.py` and the smoke test `tests/sim/test_smoke.py`); replace it, keep the entry point. See architecture.md → Wiring.
