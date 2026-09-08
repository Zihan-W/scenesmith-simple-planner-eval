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


def make_config(repository_root: Path) -> ZerithEnvironmentConfig:
    """Build a portable config for the repository-owned minimal scene."""
    root = Path(repository_root).resolve()
    scene_root = root / "models" / "online_env_minimal_scene"
    return ZerithEnvironmentConfig(
        scenario=ScenarioSpec(
            dmd_path=scene_root / "scene.dmd.yaml",
            package_xmls=(scene_root / "package.xml",),
            observed_bodies=(
                ObservedBodySpec(
                    observation_name="portable_object",
                    model_instance_name="portable_test_box",
                    body_name="base_link",
                    write_back=True,
                ),
            ),
        ),
        robot_model_dir=root / "models" / "zerith_drake",
        # The floor top is z=0.0. This leaves the wheel collision proxies
        # 1.6 mm above it in the default configuration.
        robot_xyz=(0.0, 0.0, 0.2315),
        robot_yaw_deg=0.0,
        rail_position=0.4,
        q_home_left=(0.0,) * 7,
        episode_duration=1.0,
        task=NullTask(),
    )


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
