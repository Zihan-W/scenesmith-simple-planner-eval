"""PickLift environment configuration; no policy imports or IK-file dependency."""

import dataclasses
import json
import os
from pathlib import Path

from src.online_manipulation import (
    ObservedBodySpec,
    PickLiftTask,
    PickLiftTaskConfig,
    ScenarioSpec,
    TimingConfig,
    VisualizationConfig,
    NullTask,
    ZerithEnvironmentConfig,
    make_zerith_robot_spec,
)


from src.online_manipulation.recipes.pick_environment import make_config


def make_env_config() -> ZerithEnvironmentConfig:
    """Return environment configuration without constructing a policy."""
    return make_config(
        repository_root=Path(__file__).resolve().parents[3],
        scene_root=Path(os.environ["SCENE_ROOT"]),
        pick_artifact_root=Path(os.environ["PICK_ARTIFACT_ROOT"]),
    )


def make_visual_env_config() -> ZerithEnvironmentConfig:
    """Select visualization independently of the policy."""
    config = make_env_config()
    return dataclasses.replace(
        config,
        scenario=dataclasses.replace(
            config.scenario, visualization=VisualizationConfig(enabled=True)
        ),
    )
