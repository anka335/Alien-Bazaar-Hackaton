# Block 0: Contracts & Skeleton

**Status:** review · **Owner:** Softjey + Claude · **Branch:** `block/00-contracts`
**Owned paths:** `pyproject.toml`, `uv.lock`, `.python-version`, `.gitignore`, tooling config, `src/sorter/core/`, `src/sorter/sim/`, `src/sorter/app.py`, `src/sorter/__main__.py`, `tests/conftest.py`, `tests/core/`, `tests/sim/`, structure of `config/default.yaml`, `docs/architecture.md`

## Goal

Make parallel work possible: implement the stack, repo layout, shared types, and stubs defined in [architecture.md](../architecture.md), so every block can develop and test without hardware or other blocks.

## Scope

- [x] Stack chosen: Python 3.11, uv, pytest, ruff, FastAPI ([D-005](../decisions.md), [D-010](../decisions.md)).
- [x] Contracts, repo layout, and config keys defined in `docs/architecture.md`.
- [x] Project skeleton per _Repo layout_: `pyproject.toml` (the arm SDK is added by block 5, D-010), package dirs with empty `__init__.py`, a placeholder `config.py` per block, ruff + pytest config, `.gitignore` (`data/`, `config/local.yaml`).
- [x] `sorter.core`: `types`, `errors`, `config` (YAML merge + pydantic; each block's section model is a placeholder its owner fills in), `io` (`save_observation` / `load_observation`), `hub`, `observer`, `HubLogHandler` (`log`), `System`.
- [x] Protocols (`sorter.core.protocols`) for `Camera`, `BoxDetector`, `ColorClassifier`, `Calibration`, `ArmController`.
- [x] Simulator (`sorter.sim`): `SimWorld`, `SimCamera`, `SimArm`, `SimCalibration`, sim vision. Per-component selection through `backends`.
- [x] `sorter/app.py`: build the system from config, start threads, Ctrl+C → hold → shutdown. CLI `python -m sorter run [--sim]`.
- [x] Minimal state machine and dashboard placeholders, enough for the smoke test. Blocks 6 and 7 replace them.
- [x] Smoke test: the loop sorts all sim items end to end (`tests/sim/test_smoke.py`, 3 seeds, with misses and a double grasp).
- [x] `README.md` → _Getting started_, `AGENTS.md` → _Conventions_: install, run, test commands.

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
- The arm SDK pulls Pinocchio and needs Python < 3.12 (D-010). It is not a dependency yet; block 5 adds it and checks `uv sync` on the team laptops.
- The simulator never reports a wrong `likely_empty`, so the "missed grasp from box" path (empty background after `PLACE_ON_BG`) is not exercised by the smoke test. Block 6 covers it with its own tests.

## Open questions

- Camera model? It decides the camera SDK dependency (Orbbec / RealSense).

## Requests from other blocks

_None yet._

## Log

- 2026-09-25: contracts, repo layout, config keys, and stack fixed in `architecture.md` (eye-in-hand, observation-driven loop, D-005 to D-009).
- 2026-09-25: skeleton, `sorter.core`, simulator, wiring, placeholders for blocks 6 and 7, smoke test. Python 3.11 instead of 3.12 (D-010). Added to the contracts: `sorter.core.protocols`, `System`, backend factories `sorter.<package>.backend.create(cfg)`, config models in `sorter/<package>/config.py` (architecture.md → Wiring, Config).
