#!/usr/bin/env python3
"""Extract PickLift runtime state and a VeriGraph instance catalog."""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from src.online_manipulation import load_experiment, make_env


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _json(value):
    if dataclasses.is_dataclass(value):
        return {field.name: _json(getattr(value, field.name)) for field in dataclasses.fields(value)}
    if isinstance(value, dict):
        return {str(key): _json(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", required=True, type=Path)
    parser.add_argument("--repository-root", required=True, type=Path)
    parser.add_argument("--scene-root", required=True, type=Path)
    parser.add_argument("--cache-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=500)
    args = parser.parse_args()

    experiment = load_experiment(
        args.experiment,
        repository_root=args.repository_root,
        cache_root=args.cache_root,
        scene_root=args.scene_root,
    )
    environment = make_env(experiment.environment_config)
    observation, reset_info = environment.reset(seed=args.seed)
    config = experiment.environment_config
    task = config.task.task
    task_spec = task.config
    target_name = task_spec.target_observation_name
    support_models = sorted({name.split("::", 1)[0] for name in task_spec.support_contact_bodies})
    if len(support_models) != 1:
        raise ValueError("PickLift VeriGraph profile requires exactly one support asset")
    support_name = support_models[0]
    if target_name not in observation.objects:
        raise ValueError("PickLift target missing from reset observation")

    catalog = {
        "semantic_map_locations": {
            "pick_station": "SceneSmith living-room PickLift work area"
        },
        "location_asset_relations": {"pick_station": [support_name]},
        "check_location": "pick_station",
        "entities": {
            target_name: {
                "kind": "object",
                "movable": True,
                "can_contain": False,
                "can_support": False,
                "description": "small red rectangular PickLift target",
            },
            support_name: {
                "kind": "asset",
                "movable": False,
                "can_contain": False,
                "can_support": True,
                "description": "living-room coffee table supporting the red target",
            },
        },
    }
    snapshot = {
        "schema": "scenesmith.picklift.runtime_snapshot.v1",
        "extraction": {
            "method": "scenesmith_runtime_reset",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "seed": args.seed,
            "experiment": str(args.experiment.resolve()),
            "experiment_sha256": _sha256(args.experiment),
            "scene_dmd": str(config.scenario.dmd_path),
            "scene_dmd_sha256": _sha256(config.scenario.dmd_path),
            "robot_model": str(config.robot_model_dir),
            "robot_urdf_sha256": _sha256(config.robot_model_dir / "urdf/zerith_drake.urdf"),
            "reset_info": _json(reset_info),
        },
        "robot": {
            "name": experiment.resolved_config["robot_spec"]["name"],
            "controlled_joints": list(observation.robot.joint_names),
            "q": list(observation.robot.q),
            "gripper_width_m": observation.robot.gripper_width_m,
            "end_effector_pose": observation.robot.end_effector_pose.as_dict(),
        },
        "objects": {name: value.as_dict() for name, value in observation.objects.items()},
        "task": {
            **_json(task_spec),
            "reset_observation": _json(observation.task),
            "initial_relation_from_task_binding": f"{target_name}, on, {support_name}",
        },
        "available_bt_skills": ["Wait", "ExecutePickLift", "PickLiftSucceeded"],
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "catalog.json").write_text(json.dumps(catalog, indent=2) + "\n")
    (args.output_dir / "runtime_snapshot.json").write_text(json.dumps(snapshot, indent=2) + "\n")
    print(json.dumps({
        "output_dir": str(args.output_dir.resolve()),
        "target": target_name,
        "support": support_name,
        "target_pose": snapshot["objects"][target_name]["pose"],
    }))


if __name__ == "__main__":
    main()
