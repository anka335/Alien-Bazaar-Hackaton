#!/usr/bin/env bash
# No sudo needed. Into ros2_ws/.pydeps (gitignored):
#   pydantic 2 for cloth_detector_node, which imports the sorter's block-4 code (Ubuntu's
#   python3-pydantic is v1); motorbridge-smart-servo for leader_teleop (StarArm102 leader).
set -euo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
uv pip install --target "$ws/.pydeps" --python /usr/bin/python3 "pydantic>=2.7" motorbridge-smart-servo
echo "installed into $ws/.pydeps"
