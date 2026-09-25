# Block 0: Contracts & Skeleton

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** `pyproject.toml`, `uv.lock`, `.gitignore`, tooling config, `src/sorter/core/`, `src/sorter/sim/`, `src/sorter/app.py`, `src/sorter/__main__.py`, `tests/core/`, `tests/sim/`, structure of `config/default.yaml`, `docs/architecture.md`

## Goal

Make parallel work possible: implement the stack, repo layout, shared types, and stubs defined in [architecture.md](../architecture.md), so every block can develop and test without hardware or other blocks.

## Scope

- [x] Stack chosen: Python 3.12, uv, pytest, ruff, FastAPI ([D-005](../decisions.md)).
- [x] Contracts, repo layout, and config keys defined in `docs/architecture.md`.
- [ ] Project skeleton per _Repo layout_: `pyproject.toml` (depends on the arm SDK), package dirs with empty `__init__.py`, ruff + pytest config, `.gitignore` (`data/`, `config/local.yaml`).
- [ ] `sorter.core`: `types`, `errors`, `config` (YAML merge + pydantic; each block's section model is a placeholder its owner fills in), `io` (`save_observation` / `load_observation`), `hub`, `observer`, `HubLogHandler`.
- [ ] Protocols for `Camera`, `BoxDetector`, `ColorClassifier`, `Calibration`, `ArmController`.
- [ ] Simulator (`sorter.sim`): `SimWorld`, `SimCamera`, `SimArm`, `SimCalibration`, sim vision. Per-component selection through `backends`.
- [ ] `sorter/app.py`: build the system from config, start threads, Ctrl+C → hold → shutdown. CLI `python -m sorter run [--sim]`.
- [ ] Minimal state machine and dashboard placeholders, enough for the smoke test. Blocks 6 and 7 replace them.
- [ ] Smoke test: the loop sorts all sim items end to end.
- [ ] `README.md` → _Getting started_, `AGENTS.md` → _Conventions_: install, run, test commands.

## Out of scope

Real implementations of any block.

## Depends on / Unblocks

- Depends on: nothing.
- Unblocks: 3, 4, 5, 6, 7.

## Acceptance criteria

- A fresh clone installs and runs `python -m sorter run --sim` and the smoke test with the commands from `README.md`.
- The types in `sorter.core` match `docs/architecture.md`.

## Notes & risks

- Keep interfaces minimal. They will change during integration, and that's fine if `architecture.md` follows.
- The arm SDK pulls Pinocchio. Check that `uv sync` works on the team laptops early.

## Open questions

- Camera model? It decides the camera SDK dependency (Orbbec / RealSense).

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: contracts, repo layout, config keys, and stack fixed in `architecture.md` (eye-in-hand, observation-driven loop, D-005 to D-009).
