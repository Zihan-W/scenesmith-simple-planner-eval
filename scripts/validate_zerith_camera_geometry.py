#!/usr/bin/env python3
"""Compatibility command; implementation lives in tools.calibration.validate_zerith_camera_geometry."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.calibration.validate_zerith_camera_geometry import *
from tools.calibration.validate_zerith_camera_geometry import main

if __name__ == "__main__":
    main()
