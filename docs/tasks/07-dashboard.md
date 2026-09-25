# Block 7: Dashboard

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** _TBD in block 0_ (dashboard server and frontend)

## Goal

A live view that makes the demo understandable to the audience and gives the operator control.

## Scope

- [ ] Live camera feed with overlays: box and background ROIs, detected grasp point, item mask/center, detected color label.
- [ ] Optional depth view (colorized) next to the color feed.
- [ ] Current system state, visually prominent.
- [ ] Item counters per bin.
- [ ] Controls: start / pause / step / **emergency stop** (big, always visible).
- [ ] Recent events / error log.
- [ ] Works on stubs / simulator before hardware is ready.

## Depends on / Unblocks

- Depends on: 0 (status snapshot + control contract, stubs).
- Unblocks: 8.

## Acceptance criteria

- Readable from 3 meters on a projector or laptop screen.
- Feed latency low enough that overlays match what the arm is doing.
- E-stop from the dashboard stops the arm (verified on the real rig).

## Notes & risks

- Keep the tech simple (a single page, no build step, unless the team prefers otherwise). Record the choice in `docs/decisions.md`.

## Open questions

- Runs on the same machine as the robot loop, or on a separate laptop/screen?

## Requests from other blocks

_None yet._

## Log

_Significant changes to this block's scope or contracts, one line each (date: what, why)._
