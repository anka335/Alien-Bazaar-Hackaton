# Block 6: State Machine

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** _TBD in block 0_ (orchestrator / main loop, entry point)

## Goal

The main loop that ties all modules together and survives the usual failures without human help.

## Scope

- [ ] Main loop: pick from box → place on background → classify → pick from background → drop into bin → verify → repeat.
- [ ] Park the arm out of view before every frame used for a decision.
- [ ] Failure handling:
  - **missed grasp from box:** background empty after placing → tell the box detector (so it tries elsewhere) and retry;
  - **missed grasp from background:** item still on background after drop → retry pick from background; the counter doesn't go up;
  - **two items grabbed at once:** detect if possible (blob too large / two blobs) → handle per team decision;
  - **empty box:** confirmed on N consecutive frames → finish;
  - **too many consecutive failures** → pause and show an error on the dashboard.
- [ ] Bin counters are incremented **only after a verified drop**.
- [ ] Control: start / pause / resume / single-step / stop (from the dashboard). **Step mode** is the backup for a live demo.
- [ ] Publish a status snapshot for the dashboard per the block 0 contract: state, counters, last detections, errors, recent log.
- [ ] Save frames and decisions of each cycle to disk for debugging.
- [ ] Runs end to end on stubs / simulator. A test covers the happy path and each failure path.

## Depends on / Unblocks

- Depends on: 0 (contracts, stubs). Integration needs 2–5.
- Unblocks: 7 (live status), 8.

## Acceptance criteria

- On stubs/sim: sorts all items; each failure path is exercised by a test.
- On the real rig: completes a full box without manual intervention (except e-stop).

## Notes & risks

- Keep the state machine dumb and explicit. Decisions like "which point" belong to vision modules.
- Measure the cycle time. It affects how many items fit into the demo slot.

## Open questions

- What to do with a double grasp? What to do with a low-confidence color?

## Requests from other blocks

_None yet._

## Log

_Significant changes to this block's scope or contracts, one line each (date: what, why)._
