---
description: Bring project docs in line with the code changes on the current branch
---

Make the project docs reflect the changes on this branch.

1. Collect the changes: `git diff $(git merge-base HEAD main)` plus uncommitted and untracked files.
2. For each change, decide whether it is significant using the table in `AGENTS.md` → "Keeping docs in sync", and which doc must reflect it.
3. Read those docs and update them: `docs/architecture.md`, `docs/decisions.md`, the stage brief in `docs/rover/` (task table, Log, Requests from other blocks), the status board in `docs/plan.md`, `README.md`, `AGENTS.md`. Describe the current state; remove statements that are no longer true.
4. If a contract changed, add a Log line to every affected stage brief.
5. Report to the user what you updated and why, and anything you were unsure about. Don't commit unless asked.
