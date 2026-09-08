"""External expert policy configuration; only this module reads IK artifacts."""

import dataclasses
import json
import os
from pathlib import Path

import numpy as np

from src.online_manipulation import (
    PickLiftPolicy,
    PickLiftPolicyConfig,
    PickLiftTask,
    Pose,
    ZerithEnvironmentConfig,
    make_zerith_robot_spec,
)


from src.online_manipulation.recipes.pick_policy import build_policy


def make_policy(config: ZerithEnvironmentConfig) -> PickLiftPolicy:
    """Load only this policy's assets; never construct or reset an environment."""
    root = Path(__file__).resolve().parents[3]
    return build_policy(
        config,
        pick_artifact_root=Path(os.environ["PICK_ARTIFACT_ROOT"]),
        calibration_json=root / "experiments/inputs/pick_lift/pick_lift_calibration.json",
    )
