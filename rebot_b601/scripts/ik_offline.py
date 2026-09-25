#!/usr/bin/env python3
"""Offline inverse kinematics for x y z [m]; never touches the hardware.  (thin wrapper: python -m rebot_b601 ik ...)"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rebot_b601.cli import main  # noqa: E402

sys.exit(main(["ik", *sys.argv[1:]]))
