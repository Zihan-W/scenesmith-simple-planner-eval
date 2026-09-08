#!/usr/bin/env python3
"""Compatibility command; implementation lives in tools.calibration.prepare_zerith_pick_eval_scene."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.calibration.prepare_zerith_pick_eval_scene import *
from tools.calibration.prepare_zerith_pick_eval_scene import main

if __name__ == "__main__":
    main()
