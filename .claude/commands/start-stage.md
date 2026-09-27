---
description: Pick up a rover stage (A or B) - read its brief, create a branch, mark it in progress
argument-hint: <stage letter, e.g. A>
---

Pick up rover stage $ARGUMENTS of the project.

1. Read `AGENTS.md`, `docs/plan.md`, `docs/architecture.md`, the stage brief `docs/rover/<letter>-*.md` (lowercase letter, e.g. `a-loading.md`), and `docs/rover/0-preparation.md` (what the base gives and the known issues).
2. Check the status board in `docs/plan.md`. If the stage is already `in progress` with another owner, stop and ask the user.
3. Create branch `stage/<letter>-short-name` from an up-to-date `main`. If the working tree is not clean, or another agent works in this checkout, suggest a separate `git worktree` instead and ask.
4. Ask the user for the owner name, then set status `in progress`, owner, and branch in the status board and in the brief's header.
5. Summarize for the user: goal, your lane (owned paths), the task list, known issues handed over, open questions. Propose an approach and wait for confirmation before writing code.
