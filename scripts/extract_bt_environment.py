#!/usr/bin/env python3
"""Extract behavior-tree planning metadata from a reset SceneSmith runtime."""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from src.online_manipulation import (
    build_navigation_map,
    build_planning_query,
    load_experiment,
    make_env,
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _json_value(value):
    if dataclasses.is_dataclass(value):
        return {
            field.name: _json_value(getattr(value, field.name))
            for field in dataclasses.fields(value)
        }
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return value


def derive_bt_skills(spec, observation) -> list[str]:
    """Derive executable BT leaves from measured and declared capabilities."""
    skills = ["Wait"]
    if observation.base and hasattr(spec, "base_link_name"):
        skills.append("NavigateTo")
    dual = {"left", "right"}
    if dual.issubset(spec.arm_groups) and dual.issubset(spec.grippers):
        skills.extend(("SetDualJointTargets", "ParkedDualTargetsReached"))
    return skills


def build_metadata(*, experiment_path, experiment, observation, reset_info, navigation_map, seed):
    """Build a stable JSON payload solely from runtime/configuration objects."""
    config = experiment.environment_config
    scenario = config.scenario
    adapter = config.robot_adapter
    spec = adapter.spec
    robot_state = observation.robot.as_dict()
    scene_files = [scenario.dmd_path, *scenario.package_xmls, spec.model_path]
    files = {
        str(path): _sha256(path)
        for path in scene_files
        if Path(path).is_file()
    }
    return {
        "schema": "scenesmith.bt.environment.v2",
        "extraction": {
            "method": "scenesmith_runtime_reset_and_drake_proximity_geometry",
            "extractor": "scripts/extract_bt_environment.py",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "reset_seed": int(seed),
            "reset_time_s": float(observation.time_s),
            "experiment": str(experiment_path.resolve()),
            "experiment_sha256": _sha256(experiment_path),
            "source_sha256": files,
            "reset_info": _json_value(reset_info),
        },
        "scene": {
            "dmd_path": str(scenario.dmd_path),
            "package_xmls": [str(path) for path in scenario.package_xmls],
            "ground_body_names": list(scenario.ground_body_names),
            "ground_geometries": [list(value) for value in scenario.ground_geometries],
            "observed_body_specs": _json_value(scenario.observed_bodies),
        },
        "robot": {
            "name": spec.name,
            "model_instance_name": spec.model_instance_name,
            "model_path": str(spec.model_path),
            "model_sha256": _sha256(spec.model_path),
            "base_link_name": spec.base_link_name,
            "navigation_frame_name": getattr(adapter, "navigation_frame_name", None),
            "controlled_joints": _json_value(spec.controlled_joints),
            "arm_groups": _json_value(spec.arm_groups),
            "end_effector_frames": _json_value(spec.end_effector_frames),
            "grippers": _json_value(spec.grippers),
            "cameras": _json_value(spec.cameras),
            "measured_state": {
                "joint_names": robot_state["joint_names"],
                "q": robot_state["q"],
                "v": robot_state["v"],
                "gripper_widths_m": robot_state["gripper_widths_m"],
                "end_effectors": robot_state["end_effectors"],
            },
        },
        "base": _json_value(observation.base),
        "objects": {
            name: value.as_dict() for name, value in observation.objects.items()
        },
        "navigation_geometry": {
            "obstacle_aabbs_xy_m": [list(bounds) for bounds in navigation_map.obstacles],
            "robot_swept_radius_m": navigation_map.robot_radius_m,
            "map_bounds_xy_m": list(navigation_map.bounds),
            "resolution_m": navigation_map.resolution_m,
        },
        "available_skills": derive_bt_skills(spec, observation),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", required=True, type=Path)
    parser.add_argument("--repository-root", required=True, type=Path)
    parser.add_argument("--cache-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scene-root", type=Path)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--trust-factories", action="store_true")
    args = parser.parse_args()

    experiment = load_experiment(
        args.experiment,
        repository_root=args.repository_root,
        cache_root=args.cache_root,
        scene_root=args.scene_root,
        trust_factories=args.trust_factories,
    )
    environment = make_env(experiment.environment_config)
    observation, reset_info = environment.reset(seed=args.seed)
    config = experiment.environment_config
    query = build_planning_query(
        scenario=config.scenario,
        robot_adapter=config.robot_adapter,
        timing=config.timing,
    )
    navigation_map = build_navigation_map(
        query,
        navigation_frame=config.robot_adapter.navigation_frame_name,
        ground_body_names=config.scenario.ground_body_names,
        ground_geometries=config.scenario.ground_geometries,
    )
    metadata = build_metadata(
        experiment_path=args.experiment,
        experiment=experiment,
        observation=observation,
        reset_info=reset_info,
        navigation_map=navigation_map,
        seed=args.seed,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(args.output.resolve()),
        "schema": metadata["schema"],
        "objects": len(metadata["objects"]),
        "obstacles": len(metadata["navigation_geometry"]["obstacle_aabbs_xy_m"]),
        "available_skills": metadata["available_skills"],
    }))


if __name__ == "__main__":
    main()
