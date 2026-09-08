"""Environment-only configuration for the portable minimal scene."""

import dataclasses
from pathlib import Path

from src.online_manipulation import (
    NullTask,
    ObservedBodySpec,
    ScenarioSpec,
    VisualizationConfig,
    ZerithEnvironmentConfig,
)


from src.online_manipulation.recipes.minimal import make_config


def make_env_config() -> ZerithEnvironmentConfig:
    """Return only environment configuration, without selecting a policy."""
    root = Path(__file__).resolve().parents[2]
    return make_config(root)


def make_visual_env_config() -> ZerithEnvironmentConfig:
    """Enable Meshcat in scene configuration for optional HTML recording."""
    config = make_env_config()
    scenario = dataclasses.replace(
        config.scenario, visualization=VisualizationConfig(enabled=True)
    )
    return dataclasses.replace(config, scenario=scenario)
