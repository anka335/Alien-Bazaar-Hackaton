@AGENTS.md

## Claude Code specifics

- `/start-block NN`: pick up a block (see `.claude/commands/start-block.md`).
- `/sync-docs`: check the current diff against the "Keeping docs in sync" table and update docs. Run it before every commit that touches code, and always before opening a PR.
- If you notice a significant change without matching doc updates (yours or already in the branch), fix the docs before reporting the task done.
