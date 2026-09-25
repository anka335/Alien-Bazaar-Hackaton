# Block 0: Contracts & Skeleton

**Status:** todo · **Owner:** — · **Branch:** —
**Owned paths:** repo root config (build/tooling files), shared types module, `docs/architecture.md`

## Goal

Make parallel work possible: fix the stack, the repo layout, and the interfaces between blocks, and provide stubs so every block can develop and test without hardware or other blocks.

## Scope

- [ ] Choose language, package manager, test runner, linter. Document them in `AGENTS.md` → *Conventions* and in `README.md` → *Getting started*.
- [ ] Define the repo layout and the **directory ⇄ block ownership map** in `docs/architecture.md`. Fill *Owned paths* in every task file.
- [ ] Define every contract listed in `docs/architecture.md` → *Contracts*: shared types, units, coordinate frames, "nothing found" signals, blocking semantics.
- [ ] Config format and location, and which keys belong to which block.
- [ ] Stubs/mocks for camera, arm, box detector, color classifier, calibration. A simple simulator is a bonus: a fake scene the fake arm can change, so the state machine runs end to end.
- [ ] Entry point to run the system with stubs (e.g. `run --sim`).
- [ ] A smoke test that runs the loop on stubs.

## Out of scope

Real implementations of any block.

## Depends on / Unblocks

- Depends on: nothing. Hardware models (arm SDK, camera SDK) help to shape the interfaces; ask the team.
- Unblocks: 3, 4, 5, 6, 7.

## Acceptance criteria

- `docs/architecture.md` has no _TBD_ in *Contracts* and *Repo layout*.
- A fresh clone can install and run the stub pipeline and its test with the commands from `README.md`.
- Every task file has *Owned paths* filled in.

## Notes & risks

- Keep interfaces minimal: only what the state machine and dashboard actually need. They will change during integration, and that's fine if the docs follow.
- Interfaces are designed together with the team and agents. Propose, discuss, then commit.

## Open questions

- Arm model and SDK? Camera model and SDK (RealSense / OAK-D / other)?
- Single process with threads, or separate processes (e.g. dashboard as a separate service)?

## Requests from other blocks

_None yet._

## Log

_Significant changes to this block's scope or contracts, one line each (date: what, why)._
