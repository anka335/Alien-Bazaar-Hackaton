# Block 8: Demo Preparation

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** `docs/demo.md` (runbook). Tuned values go into the config sections of the owning blocks.

## Goal

A demo that works on stage, and a backup if it doesn't.

## Scope

- [ ] **Day 1:** choose the demo clothes. Unambiguous colors, fabrics that grasp reliably, no mid-gray. Share them with blocks 3, 4, 5 for tuning.
- [ ] Full end-to-end rehearsals. Log success rate and cycle time per run here.
- [ ] Record a backup video of a successful run.
- [ ] Demo runbook (`docs/demo.md`): rig setup, calibration touch test, startup commands, reset between runs, what to do on failure (step mode, HOLD, hardware e-stop, restart).
- [ ] Pitch notes: problem, how it works, what's next (wash–dry–sort pipeline).

## Depends on / Unblocks

- Depends on: integration of 1–7. Clothes selection has no dependencies.

## Acceptance criteria

- ≥ 3 consecutive full rehearsals without intervention.
- Backup video recorded and stored where the team can access it offline.

## Notes & risks

- Venue lighting differs from the lab. The dedicated lamp should dominate. Re-check thresholds on site.
- Transport may shift the zones. Re-place them on the tape marks and run the touch test. The hand-eye result survives transport as long as the camera mount is untouched (D-006).

## Open questions

- Demo slot length? It determines how many items to sort live.

## Log

- 2026-09-25: runbook covers the touch test and HOLD instead of full recalibration and e-stop (D-006, D-009).
- 2026-09-25 (block 7): Space or Esc on the dashboard also sends HOLD; `/snapshot/decision.jpg` saves the current decision frame (e.g. for backup material).
- 2026-09-25 (block 4): the real color classifier needs network access to the SAM3 service and its API key in `config/local.yaml` (D-013). Check both before the demo; SAM3 prompt and threshold are validated on the demo clothes.
- 2026-09-26 (block 5): build the table to `sim.layout` (README → Table layout, D-015): the arm reaches the tray and the mat only with that geometry. The dashboard's 3D view shows the arm and the table, useful for the audience and for rehearsing on the simulator.
