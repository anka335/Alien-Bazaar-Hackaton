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
| D6 | Benchmark under perturbations | todo | — | A's, B's, F's and C's benchmarks with D1–D5 on hold up (a small drop is fine) |

## Log

- 2026-09-27: stage defined ([D-035](../decisions.md)).
- 2026-09-27 (from the first rig runs, `data/sessions/` on the rig laptop): for D5, the control loop over USB-CAN on macOS runs up to 120 ms late (20 ms expected) and the feedback arrives at ~10 Hz: a joint follows its setpoint ~0.3 s late (12° at 40°/s), [D-046](../decisions.md). The arm's base leans ~6° forward on the rover (measured from the floor, [D-048](../decisions.md)); after `arm.base_tilt_deg` the floor still came out 1.2° off in the next run (the rover stood differently): a residual lean of 1–2° is realistic for D1. The room's LED light turns white socks pink (magenta: no white balance takes it out) — for D2.

