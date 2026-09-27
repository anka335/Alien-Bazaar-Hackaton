@AGENTS.md

## Claude Code specifics

- `/start-stage A` (or `B`, `N`, `C`, `D`): pick up a rover stage (see `.claude/commands/start-stage.md`).
- `/sync-docs`: check the current diff against the "Keeping docs in sync" table and update docs. Run it before every commit that touches code, and always before opening a PR.
- If you notice a significant change without matching doc updates (yours or already in the branch), fix the docs before reporting the task done.
