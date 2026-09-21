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
from PIL import Image

from simulation.src import load_experiment, make_env


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


def _camera_artifacts(observation, output_dir: Path) -> dict:
    """Persist robot-mounted sensor frames with compact provenance metadata."""
    camera_dir = output_dir / "cameras"
    camera_dir.mkdir(parents=True, exist_ok=True)
    result = {}
    for name, camera in sorted(observation.sensors.items()):
        metadata = camera.as_dict()
        files = {}
        if camera.rgb is not None:
            path = camera_dir / f"{name}_rgb.png"
            Image.fromarray(camera.rgb).save(path)
            files["rgb"] = {
                "path": str(path.relative_to(output_dir)),
                "sha256": _sha256(path),
            }
        if camera.depth is not None:
            path = camera_dir / f"{name}_depth_m.npy"
            np.save(path, camera.depth)
            valid = np.isfinite(camera.depth) & (camera.depth > 0.0)
            files["depth"] = {
                "path": str(path.relative_to(output_dir)),
                "sha256": _sha256(path),
                "unit": "m",
                "valid_pixel_fraction": float(valid.mean()),
                "valid_range_m": (
                    [float(camera.depth[valid].min()), float(camera.depth[valid].max())]
                    if np.any(valid)
                    else None
                ),
            }
        if camera.label is not None:
            path = camera_dir / f"{name}_label.npy"
            np.save(path, camera.label)
            labels, counts = np.unique(camera.label, return_counts=True)
            files["label"] = {
                "path": str(path.relative_to(output_dir)),
                "sha256": _sha256(path),
                "visible_labels": [
                    {
                        "label": int(label),
                        "name": camera.label_names.get(int(label), "unmapped"),
                        "pixel_count": int(count),
                    }
                    for label, count in zip(labels, counts)
                    if int(label) in camera.label_names
                ],
            }
        metadata["files"] = files
        result[name] = metadata
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", required=True, type=Path)
    parser.add_argument("--repository-root", required=True, type=Path)
    parser.add_argument("--scene-root", required=True, type=Path)
    parser.add_argument("--cache-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=500)
    parser.add_argument(
        "--required-camera",
        action="append",
        default=[],
        help="Require this robot-mounted camera and persist its reset frame.",
    )
    parser.add_argument(
        "--require-target-visible-in",
        action="append",
        default=[],
        help="Require the configured PickLift target to have label pixels in this camera.",
    )
    args = parser.parse_args()

    experiment = load_experiment(
        args.experiment,
        repository_root=args.repository_root,
        cache_root=args.cache_root,
        scene_root=args.scene_root,
    )
    environment = make_env(experiment.environment_config)
    observation, reset_info = environment.reset(seed=args.seed)
    missing_cameras = sorted(set(args.required_camera) - set(observation.sensors))
    if missing_cameras:
        raise ValueError(f"Required robot cameras are missing: {missing_cameras}")
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
    args.output_dir.mkdir(parents=True, exist_ok=True)
    cameras = _camera_artifacts(observation, args.output_dir)
    target_visibility = {}
    for name, camera in cameras.items():
        labels = camera.get("files", {}).get("label", {}).get("visible_labels", ())
        pixel_count = sum(
            item["pixel_count"]
            for item in labels
            if item["name"] == task_spec.target_contact_body
        )
        target_visibility[name] = {
            "target_contact_body": task_spec.target_contact_body,
            "visible": pixel_count > 0,
            "pixel_count": pixel_count,
        }
    invisible = sorted(
        name
        for name in args.require_target_visible_in
        if not target_visibility.get(name, {}).get("visible", False)
    )
    if invisible:
        raise ValueError(f"PickLift target is not visible in required cameras: {invisible}")
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
        "robot_camera_observations": cameras,
        "target_camera_visibility": target_visibility,
        "objects": {name: value.as_dict() for name, value in observation.objects.items()},
        "task": {
            **_json(task_spec),
            "reset_observation": _json(observation.task),
            "initial_relation_from_task_binding": f"{target_name}, on, {support_name}",
        },
        "available_bt_skills": ["Wait", "ExecutePickLift", "PickLiftSucceeded"],
    }
    (args.output_dir / "catalog.json").write_text(json.dumps(catalog, indent=2) + "\n")
    (args.output_dir / "runtime_snapshot.json").write_text(json.dumps(snapshot, indent=2) + "\n")
    print(json.dumps({
        "output_dir": str(args.output_dir.resolve()),
        "target": target_name,
        "support": support_name,
        "robot_cameras": sorted(cameras),
        "target_visible_in": sorted(
            name for name, value in target_visibility.items() if value["visible"]
        ),
        "target_pose": snapshot["objects"][target_name]["pose"],
    }))


if __name__ == "__main__":
    main()
