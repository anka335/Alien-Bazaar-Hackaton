#!/usr/bin/env python3
"""Move the TCP to x y z [m] (asks for confirmation).  (thin wrapper: python -m rebot_b601 xyz ...)"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rebot_b601.cli import main  # noqa: E402

sys.exit(main(["xyz", *sys.argv[1:]]))
