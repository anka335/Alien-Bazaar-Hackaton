# Agent Guide

Instructions for AI coding agents (Claude Code, Codex, Cursor, …) and humans working in this repo. Read this file fully before starting work.

## Project in one paragraph

A robotic arm on a rover sorts socks by color. **Load:** the rover stops next to socks; the arm finds them on the floor with the RGB-D camera on its wrist, classifies each one's color (light / dark / colored) and drops it into the matching compartment of the cargo box on the rover. **Unload:** at a station the arm empties each compartment into its laundry bin. A live dashboard shows the process. Everything is built on the MuJoCo simulator first. This is a **hackathon project**: favor simple, working, demo-able solutions over generality. Details are in [README.md](README.md).

## Where things are

- [docs/plan.md](docs/plan.md): the stages and the **status board**
- [docs/rover/](docs/rover/): one brief per stage (0 preparation, done; A loading; B unloading), the unit of work for an agent: goal, lane, tasks, known issues
- [docs/architecture.md](docs/architecture.md): physical setup, components, loops, coordinate frames, **contracts between the stages**, simulator, lanes (who owns which path)
- [docs/decisions.md](docs/decisions.md): decision log

## Ground rules (already decided, don't relitigate without a new entry in decisions.md)

- Vision uses **classic CV plus depth**, not trained models, unless a stage brief says otherwise. Exception: masks come from a remote SAM3 service ([D-013](docs/decisions.md)); in the sim from the render's segmentation.
- Vision outputs **pixel coordinates**. Only the calibration module converts pixels to arm coordinates.
- The simulator stands in for the hardware (camera, arm): develop and test on `--sim` ([D-021](docs/decisions.md), [D-033](docs/decisions.md)).
- Safety comes first: every arm path is checked against the floor and the rover's keep-out boxes, every pick target against its zone, and a stop (hold) is always available. Never disable the motors except at the rest pose: the arm falls.

## Parallel workflow

Two agents work at the same time, **one agent per stage** (A loading, B unloading).

1. **Pick up a stage.** Use `/start-stage A` in Claude Code, or do it by hand:
   - read the brief `docs/rover/<a|b>-*.md`, `docs/rover/0-preparation.md` (what the base gives, known issues) and `docs/architecture.md`;
   - create branch `stage/<a|b>-short-name` from fresh `main`. For both agents on one machine, use a separate worktree: `git worktree add ../abh-a -b stage/a-loading`;
   - set the stage to `in progress` with the owner in the status board ([docs/plan.md](docs/plan.md)) and in the brief's header.
2. **Stay in your lane.** Edit only the paths your stage owns ([architecture.md → Repo layout](docs/architecture.md#repo-layout)), plus your brief. If you need a change in the other stage's code, write it under *Requests from other blocks* in its brief instead.
3. **Shared code is a contract.** Core, the arm, the base scene, the layout tool, the state machine, the dashboard backend and the front end's shared part belong to both. To change one:
   - update `docs/architecture.md` in the same PR;
   - add a line to the *Log* of the other stage's brief;
   - say so explicitly in the PR description;
   - keep it a small PR of its own, so the other agent can rebase onto it early.
4. **Finish.** Run your stage's tests, run `/sync-docs`, update the status board, and open a PR into `main`.

## Keeping docs in sync (mandatory)

Docs are part of the change. **A significant change is not done until the docs reflect it, in the same commit or PR.** Use this table:

| If you changed… | Update |
|---|---|
| an interface, data type, or data flow between stages, or shared code | `docs/architecture.md` + *Log* of the other stage's brief |
| a design choice or trade-off (library, algorithm, approach) | new entry in `docs/decisions.md` |
| stage progress, scope, or acceptance criteria | the stage brief + status board in `docs/plan.md` |
| setup, run commands, dependencies, hardware, config keys | `README.md` |
| repo layout or path ownership | `docs/architecture.md` → Repo layout + *Your lane* in the briefs |
| the workflow or rules for agents | this file (`AGENTS.md`) |
| a discovered risk, sim or hardware quirk others should know about | the relevant stage brief |

Not significant, so no doc update is needed: internal refactors, bug fixes that don't change behavior or interfaces, test-only changes.

Keep docs short and factual. Describe the current state, not history: history lives in git and in `decisions.md`. Remove things that are no longer true.

## Conventions

- Commit messages: English, [Conventional Commits](https://www.conventionalcommits.org/) (`feat(load): ...`, `fix(arm): ...`, `docs: ...`). Use the stage (`load`, `unload`) or the shared package as scope.
- Python 3.11, uv, pytest, ruff ([D-005](docs/decisions.md), [D-010](docs/decisions.md)). The dashboard's front end is React + TypeScript + Vite in `frontend/` ([D-031](docs/decisions.md)). Package layout and path ownership: [docs/architecture.md → Repo layout](docs/architecture.md#repo-layout).
- Commands (from the repo root):
  - install: `uv sync`
  - run on the simulator: `uv run python -m sorter run --sim [--mode load|unload]` (dashboard at http://127.0.0.1:8000)
  - recompute the rig after a layout change: `uv run python -m sorter.sim.layout --write` (must report 0 problems)
  - tests: `uv run pytest tests/<package>` or a single test, e.g. `uv run pytest tests/sim/test_physics.py::test_scripted_unload`
  - lint and format: `uv run ruff check --fix . && uv run ruff format .`
  - dashboard front end: `cd frontend && npm install && npm run build` (typecheck: `npm run typecheck`, hot reload: `npm run dev`); the build is not committed
- Keep package `__init__.py` files empty. The real backend of a component is `sorter/<package>/backend.py` with `create(cfg)` ([architecture.md → Wiring](docs/architecture.md#wiring)); the model of your config section is in `sorter/<package>/config.py`.
- Develop against the simulator: `build_system(cfg, sim=True)` in tests (the `sim_config` fixture: as fast as possible, the empty base scene; set `sim.scenes` to what the test needs; cloth is slow, so use few socks), `backends` in `config/local.yaml` to swap one component to `real`.
- Run the tests relevant to your change (prefer single tests or files over the full suite) before committing.
- Never commit machine-specific artifacts (calibration results, recorded frames, local config overrides) unless a stage brief says so.
