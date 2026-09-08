"""Legacy calibration-tool placement, read from the single experiment profile."""

import json
from pathlib import Path

_PROFILE = json.loads((Path(__file__).resolve().parents[1] / "experiments/profiles.json").read_text())["initial_state"]["picklift"]
ROBOT_BASE_XYZ_METERS = tuple(_PROFILE["robot_xyz"])
ROBOT_BASE_YAW_DEG = _PROFILE["robot_yaw_deg"]
PICK_RAIL_POSITION_METERS = _PROFILE["rail_position"]
