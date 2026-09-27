# Rover stage D: Sim-to-real check

**Status:** todo · **Owner:** — · **Branch:** —
**Depends on:** [C: Full mission](c-mission.md) (its benchmark).

## Goal

Before the hardware: check that the pipeline survives a less ideal simulator. Not more realism for its own sake, but the errors the rig will have ([D-035](../decisions.md)).

## Tasks

Status values: `todo` · `in progress` · `blocked` · `review` · `done`.

| # | Task | Status | Owner | Done when |
| --- | --- | --- | --- | --- |
| D1 | Calibration error | todo | — | The camera mount off by 5–10 mm / 2–3° in the sim, against the pipeline's nominal one |
| D2 | Camera like a D435i | todo | — | Depth noise, holes and edge artifacts like the real camera on a real floor |
| D3 | Friction grip | todo | — | An option to grip by friction instead of attaching the cloth vertices |
| D4 | Real SAM3 on renders | todo | — | The benchmarks run with `sim.use_sam3` (the real service) instead of the render's segmentation |
| D5 | Arm like the rig | todo | — | Joint lag and a slower control loop as seen on the real arm |
| D6 | Benchmark under perturbations | todo | — | A's, B's and C's benchmarks with D1–D5 on hold up (a small drop is fine) |

## Log

- 2026-09-27: stage defined ([D-035](../decisions.md)).
