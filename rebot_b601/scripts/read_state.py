#!/usr/bin/env python3
"""Read joint angles / TCP position (motors stay limp).  (thin wrapper: python -m rebot_b601 state ...)"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rebot_b601.cli import main  # noqa: E402

sys.exit(main(["state", *sys.argv[1:]]))
