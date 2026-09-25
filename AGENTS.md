# Agent Guide

Instructions for AI coding agents (Claude Code, Codex, Cursor, …) and humans working in this repo. Read this file fully before starting work.

## Project in one paragraph

A robotic arm sorts clothes by color: it picks an item from a mixed box, uses an RGB-D camera on its wrist to find the grasp point, places the item on a uniform background, classifies its color (light / dark / colored), and drops it into one of 3 bins. A live dashboard shows the process. This is a **hackathon project**: favor simple, working, demo-able solutions over generality. Details are in [README.md](README.md).

## Where things are

- [docs/plan.md](docs/plan.md): blocks, dependency graph, **status board**
- [docs/tasks/NN-*.md](docs/tasks/): one file per block, the unit of work for an agent
- [docs/architecture.md](docs/architecture.md): components, data flow, coordinate frames, **interfaces (contracts between blocks)**
- [docs/decisions.md](docs/decisions.md): decision log

## Ground rules (already decided, don't relitigate without a new entry in decisions.md)

- Vision uses **classic CV plus depth**, not trained models, unless a block's task file says otherwise.
- Vision outputs **pixel coordinates**. Only the calibration module converts pixels to arm coordinates.
- Every hardware-facing module has a **stub/mock** so other blocks can develop and test without hardware.
- Safety comes first: every arm target is checked against workspace limits, and a stop (hold) is always available. Never disable the motors except at the rest pose: the arm falls.

## Parallel workflow

Several agents work at the same time, **one agent per block**.

1. **Pick up a block.** Use `/start-block NN` in Claude Code, or do it by hand:
   - read `docs/tasks/NN-*.md`, its dependencies, and `docs/architecture.md`;
   - create branch `block/NN-short-name` from fresh `main`. For several agents on one machine, use a separate worktree: `git worktree add ../abh-NN -b block/NN-short-name`;
   - set the block to `in progress` with the owner in the status board ([docs/plan.md](docs/plan.md)) and in the task file header.
2. **Stay in your lane.** Edit only the paths your block owns (listed in the task file once block 0 defines the layout), plus the docs of your block. If you need a change in another block's code, note it in that block's task file under *Requests from other blocks* instead of editing its code.
3. **Contracts are shared.** An interface in `docs/architecture.md` is a contract between blocks. To change one:
   - update `docs/architecture.md` in the same PR;
   - add a line to the *Log* of every affected task file;
   - say so explicitly in the PR description.
4. **Use stubs, not waiting.** If a dependency isn't ready, code against its interface using its stub.
5. **Finish.** Run the tests of your block, run `/sync-docs`, update the status board, and open a PR into `main`.

## Keeping docs in sync (mandatory)

Docs are part of the change. **A significant change is not done until the docs reflect it, in the same commit or PR.** Use this table:

| If you changed… | Update |
|---|---|
| an interface, data type, or data flow between blocks | `docs/architecture.md` + *Log* of affected task files |
| a design choice or trade-off (library, algorithm, approach) | new entry in `docs/decisions.md` |
| block progress, scope, or acceptance criteria | the block's task file + status board in `docs/plan.md` |
| setup, run commands, dependencies, hardware, config keys | `README.md` |
| repo layout or path ownership | `docs/architecture.md` + *Owned paths* in task files |
| the workflow or rules for agents | this file (`AGENTS.md`) |
| a discovered risk or hardware quirk others should know about | *Notes & risks* of the relevant task file |

Not significant, so no doc update is needed: internal refactors, bug fixes that don't change behavior or interfaces, test-only changes.

Keep docs short and factual. Describe the current state, not history: history lives in git and in `decisions.md`. Remove things that are no longer true.

## Conventions

- Commit messages: English, [Conventional Commits](https://www.conventionalcommits.org/) (`feat(vision): ...`, `fix(arm): ...`, `docs: ...`). Use the block name as scope.
- Python 3.11, uv, pytest, ruff ([D-005](docs/decisions.md), [D-010](docs/decisions.md)). Package layout and path ownership: [docs/architecture.md → Repo layout](docs/architecture.md#repo-layout).
- Commands (from the repo root):
  - install: `uv sync`
  - run on the simulator: `uv run python -m sorter run --sim` (dashboard at http://127.0.0.1:8000)
  - tests: `uv run pytest tests/<package>` or a single test, e.g. `uv run pytest tests/sim/test_smoke.py`
  - lint and format: `uv run ruff check --fix . && uv run ruff format .`
- Keep package `__init__.py` files empty. The real backend of a component is `sorter/<package>/backend.py` with `create(cfg)` ([architecture.md → Wiring](docs/architecture.md#wiring-block-0)); the model of your config section is in `sorter/<package>/config.py`.
- Develop against the simulator: `build_system(cfg, sim=True)` in tests, `backends` in `config/local.yaml` to swap one component to `real`.
- Run the tests relevant to your change (prefer single tests or files over the full suite) before committing.
- Never commit machine-specific artifacts (calibration results, recorded frames, local config overrides) unless a task file says so.
