"""Record actual base/dual-arm state using only the public environment API."""

import argparse
import csv
import dataclasses
import json
from pathlib import Path

from src.online_manipulation import (
    BaseConfig,
    BaseVelocityAction,
    HoldAction,
    RuntimeConfig,
    ScenarioSpec,
    VisualizationConfig,
    ZerithMobileRobotAdapter,
    make_env,
    make_zerith_dual_spec,
)


def make_config(mode, *, meshcat=False, obstacle=False, repository_root=None):
    """Select a self-contained scene and explicit mobile base mode."""
    root = Path(repository_root) if repository_root is not None else Path(__file__).resolve().parents[3]
    profiles = json.loads((root / "experiments/profiles.json").read_text())
    initial = profiles["initial_state"]["mobile_planar" if mode == "planar_kinematic" else "mobile"]
    scene_profile = profiles["scene"]["mobile"]
    scene = (root / scene_profile["dmd"]).parent
    # Ideal planar mode clears the original CAD supports; wheel dynamics
    # settles onto its explicitly levelled tire/support collision surfaces.
    base = BaseConfig(
        mode=mode, base_height_m=initial["robot_xyz"][2]
    )
    spec = make_zerith_dual_spec(
        robot_model_dir=root / profiles["robot"]["zerith_dual"]["model_dir"],
        **initial,
    )
    adapter = ZerithMobileRobotAdapter(spec, base)
    return RuntimeConfig(
        ScenarioSpec(
            scene / ("obstacle.dmd.yaml" if obstacle else "empty.dmd.yaml"),
            (scene / "package.xml",),
            ground_geometries=tuple(tuple(p) for p in scene_profile["ground_geometries"]),
            visualization=VisualizationConfig(enabled=meshcat),
        ),
        adapter,
        episode_duration=60,
    )
