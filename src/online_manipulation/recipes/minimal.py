"""Environment-only configuration for the portable minimal scene."""

import dataclasses
import json
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
    profiles = json.loads((root / "experiments/profiles.json").read_text())
    scene = profiles["scene"]["minimal"]
    initial = profiles["initial_state"]["minimal"]
    return ZerithEnvironmentConfig(
        scenario=ScenarioSpec(
            dmd_path=root / scene["dmd"],
            package_xmls=tuple(root / p for p in scene["package_xmls"]),
            observed_bodies=tuple(ObservedBodySpec(**b) for b in scene["observed_bodies"]),
        ),
        robot_model_dir=root / profiles["robot"]["zerith_left"]["model_dir"],
        # The floor top is z=0.0. This leaves the wheel collision proxies
        # 1.6 mm above it in the default configuration.
        **initial,
        episode_duration=1.0,
        task=NullTask(),
    )
