---
description: Pick up a project block (0-8) - read its task, create a branch, mark it in progress
argument-hint: <block number, e.g. 04>
---

Pick up block $ARGUMENTS of the project.

1. Read `AGENTS.md`, `docs/plan.md`, `docs/architecture.md`, and the task file `docs/tasks/$ARGUMENTS-*.md` (a number without a leading zero also means that block). Also read the task files of the blocks it depends on.
2. Check the status board in `docs/plan.md`. If the block is already `in progress` with another owner, stop and ask the user.
3. Create branch `block/NN-short-name` from an up-to-date `main`. If the working tree is not clean, or other agents work in this checkout, suggest a separate `git worktree` instead and ask.
4. Ask the user for the owner name, then set status `in progress`, owner, and branch in the status board and in the task file header.
5. Summarize for the user: goal, scope checklist, dependencies and their status, open questions that block you. Propose an approach and wait for confirmation before writing code.
