#!/usr/bin/env bash
# ez_arm launcher: the rebot venv + the repo packages on the path.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONNOUSERSITE=1 PYTHONPATH="$ROOT:$ROOT/rebot_b601:$ROOT/src"
exec "$ROOT/rebot_b601/.venv/bin/python" -m ez_arm "$@"
