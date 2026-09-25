#!/usr/bin/env bash
# Stop hook: if code changed on this branch but no docs did, ask Claude to check the docs once.
input=$(cat)
# Nudge only once per stop sequence, never loop.
if echo "$input" | grep -qE '"stop_hook_active"[[:space:]]*:[[:space:]]*true'; then exit 0; fi

cd "${CLAUDE_PROJECT_DIR:-.}" 2>/dev/null || exit 0
base=$(git merge-base HEAD origin/main 2>/dev/null || git merge-base HEAD main 2>/dev/null) || exit 0

changed=$({ git diff --name-only "$base"; git ls-files --others --exclude-standard; } 2>/dev/null | sort -u)
[ -z "$changed" ] && exit 0

code=$(echo "$changed" | grep -vE '^(docs/|\.claude/|\.github/)|\.md$')
docs=$(echo "$changed" | grep -E '^docs/|\.md$')

if [ -n "$code" ] && [ -z "$docs" ]; then
  {
    echo "Code changed on this branch but no docs were updated:"
    echo "$code" | head -20
    echo "Check AGENTS.md -> 'Keeping docs in sync'. If a change is significant, update the docs (/sync-docs). If not, say so briefly and finish."
  } >&2
  exit 2
fi
exit 0
