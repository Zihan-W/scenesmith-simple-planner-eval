"""Export actual Drake proximity geometry for the fixed-skeleton GPU adapter.

This is a scene/model export, not an optimizer and not a collision certificate.
No visual mesh is substituted for a collision shape. Filtered robot pairs and
ground geometry identifiers are recorded explicitly, never silently discarded.
"""

import argparse
from collections import Counter, defaultdict
import dataclasses
import hashlib
import itertools
import json
from pathlib import Path

from pydrake.geometry import Box, Capsule, Convex, Cylinder, Ellipsoid, HalfSpace, Mesh, Sphere

from planner.src.tamp.scenesmith import target_relative_base_candidates

from simulation.src.io.experiment import load_experiment
from simulation.src.tasks.evaluation import EvaluatedTask
from simulation.src.geometry.contact import PairContactPolicy, penetration_limit, permits_contact
from simulation.src.geometry.planning import build_planning_query
from simulation.src.geometry.scene_geometry import resolve_ground_geometries
from simulation.src.tasks.tasks import PickLiftTask


def stationary_target_world_check(query, task):
    """Audit only target/static pairs using the configured support permission.

    This snapshot check neither certifies the robot nor checks a carried/lifted
    target. It must not authorize dropping movable/world costs when a skeleton
    changes the target pose. Robot/target pairs are intentionally outside this
    diagnostic and must remain separate constraints.
    """
    if isinstance(task, EvaluatedTask):
        task = task.task
    if not isinstance(task, PickLiftTask):
        raise TypeError("The target-contact diagnostic requires a PickLiftTask")
    config = task.config
    support_bound = config.maximum_allowed_support_penetration_m
    if support_bound is None:
        support_bound = config.maximum_allowed_contact_penetration_m
    policy = PairContactPolicy.from_pairs(
        "stationary_target_support",
        ((config.target_contact_body, name) for name in config.support_contact_bodies),
        monitored_bodies=(config.target_contact_body,),
        maximum_allowed_penetration_m=support_bound,
    )
    influence = 0.05
    robot_prefix = query.robot_adapter.spec.model_instance_name + "::"
    pairs = query.collision_pairs(
        query.configuration(), influence_distance_m=influence,
        additional_body_names=(config.target_contact_body,),
    )
    records = []
    for pair in pairs:
        if config.target_contact_body not in (pair.body_a, pair.body_b):
            continue
        if pair.body_a.startswith(robot_prefix) or pair.body_b.startswith(robot_prefix):
            continue
        allowed = permits_contact(policy, pair.body_a, pair.body_b,
                                  pair.geometry_a, pair.geometry_b)
        bound = penetration_limit(policy, pair.body_a, pair.body_b,
                                  pair.geometry_a, pair.geometry_b)
        records.append({
            **dataclasses.asdict(pair), "allowed_support_contact": allowed,
            "maximum_allowed_penetration_m": bound,
            "nonpenetration_valid": pair.distance_m >= -bound,
            "safety_clearance_valid": allowed or pair.distance_m >= 0.005,
        })
    nonpenetration = all(item["nonpenetration_valid"] for item in records)
    safety = all(item["safety_clearance_valid"] for item in records)
    return {
        "scope": "stationary_target_static_world_only_not_robot_or_lift_certification",
        "geometry_evaluator": "Drake.ComputeSignedDistancePairwiseClosestPoints",
        "target_body": config.target_contact_body,
        "support_bodies": list(config.support_contact_bodies),
        "maximum_allowed_support_penetration_m": support_bound,
        "influence_distance_m": influence, "minimum_safety_clearance_m": 0.005,
        "valid": nonpenetration and safety,
        "nonpenetration_valid": nonpenetration, "safety_clearance_valid": safety,
        "pairs": records,
    }


def shape_document(shape):
    """Describe the actual proximity shape in its own metric geometry frame."""
    if isinstance(shape, Box):
        return {"kind": "box", "size_m": list(shape.size())}
    if isinstance(shape, Sphere):
        return {"kind": "sphere", "radius_m": shape.radius()}
    if isinstance(shape, (Cylinder, Capsule)):
        return {"kind": "capsule" if isinstance(shape, Capsule) else "cylinder",
                "radius_m": shape.radius(), "length_m": shape.length()}
    if isinstance(shape, Ellipsoid):
        return {"kind": "ellipsoid", "radii_m": [shape.a(), shape.b(), shape.c()]}
    if isinstance(shape, HalfSpace):
        return {"kind": "half_space"}
    if isinstance(shape, (Mesh, Convex)):
        source = shape.source()
        if not source.is_path():
            raise ValueError("An in-memory proximity mesh needs an explicit geometry export")
        path = Path(source.path()).resolve()
        return {"kind": "convex" if isinstance(shape, Convex) else "mesh",
                "path": str(path), "scale": list(shape.scale3()),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    raise TypeError(f"Unsupported Drake proximity shape: {type(shape).__name__}")


def stationary_support_check(query, geometry):
    """Audit existing exact wheel/floor permissions, without inventing contacts."""
    records = geometry["geometries"]
    lookup = {(f"{item['model']}::{item['body']}", item["name"]): item for item in records}
    geometry_query = query.plant.get_geometry_query_input_port().Eval(query.context)
    inspector = geometry_query.inspector()
    ids = {int(item.get_value()): item for item in inspector.GetAllGeometryIds()}
    pairs = []
    for (first, second), bound in query.support_geometry_limits_m.items():
        a, b = lookup[first], lookup[second]
        if a["robot"] == b["robot"]:
            raise ValueError("Expected one robot support and one static floor geometry")
        robot, floor = (a, b) if a["robot"] else (b, a)
        if not floor["ground"]:
            raise ValueError("Support permission must name an explicitly resolved ground geometry")
        distance = geometry_query.ComputeSignedDistancePairClosestPoints(ids[a["id"]], ids[b["id"]]).distance
        pairs.append({"robot_geometry_id": robot["id"], "robot_link": robot["body"],
                      "ground_geometry_id": floor["id"], "distance_m": distance,
                      "maximum_allowed_penetration_m": bound, "valid": distance >= -bound})
    payload = {"geometries": records, "support_pairs": pairs}
    return {"scope": "stationary_robot_support_only_not_moving_base_permission",
            "geometry_evaluator": "Drake.ComputeSignedDistancePairClosestPoints",
            "valid": bool(pairs) and all(item["valid"] for item in pairs), "pairs": pairs,
            "geometry_support_sha256": hashlib.sha256(json.dumps(
                payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()}


def export_query_geometry(query, config, resolved_config, *, check_stationary_target=False):
    """Export the supplied query, including any synchronized observed state."""
    plant, context = query.plant, query.context
    geometry_query = plant.get_geometry_query_input_port().Eval(context)
    inspector = geometry_query.inspector()
    grounds = resolve_ground_geometries(plant, inspector, config.scenario.ground_body_names,
                                        config.scenario.ground_geometries)
    records, robot_ids = [], defaultdict(list)
    for geometry_id in inspector.GetAllGeometryIds():
        if inspector.GetProximityProperties(geometry_id) is None:
            continue
        frame = inspector.GetFrameId(geometry_id)
        body = plant.GetBodyFromFrameId(frame)
        if body is None:
            raise ValueError(f"Proximity geometry has no plant body: {inspector.GetName(geometry_id)}")
        model_name = plant.GetModelInstanceName(body.model_instance())
        robot = body.model_instance() == query.robot_model_instance
        x_body_geometry = inspector.GetPoseInFrame(geometry_id)
        x_world_geometry = body.EvalPoseInWorld(context) @ x_body_geometry
        record = {"id": int(geometry_id.get_value()), "name": inspector.GetName(geometry_id),
                  "model": model_name, "body": body.name(), "robot": robot,
                  "ground": geometry_id in grounds, "shape": shape_document(inspector.GetShape(geometry_id)),
                  "body_from_geometry": x_body_geometry.GetAsMatrix4().tolist(),
                  "world_from_geometry": x_world_geometry.GetAsMatrix4().tolist()}
        records.append(record)
        if robot:
            robot_ids[body.name()].append(geometry_id)
    ignored, partial = defaultdict(list), []
    for first, second in itertools.combinations(sorted(robot_ids), 2):
        mask = [inspector.CollisionFiltered(a, b) for a in robot_ids[first] for b in robot_ids[second]]
        if all(mask):
            ignored[first].append(second)
        elif any(mask):
            partial.append([first, second])
    spec = query.robot_adapter.spec
    base_frame = plant.GetFrameByName(spec.base_link_name, query.robot_model_instance)
    body_poses = {name: plant.CalcRelativeTransform(
        context, base_frame, plant.GetBodyByName(name, query.robot_model_instance).body_frame()
    ).GetAsMatrix4().tolist() for name in robot_ids}
    result = {"scope": "drake_proximity_geometry_export_not_collision_certification",
              "robot_base_frame": spec.base_link_name,
              "geometries": records, "self_collision_ignore_fully_filtered": dict(ignored),
              "partially_filtered_body_pairs": partial, "base_from_robot_body": body_poses,
              "observed_bodies": [dataclasses.asdict(item) for item in config.scenario.observed_bodies],
              "resolved_experiment": resolved_config}
    if check_stationary_target:
        result["stationary_target_world_check"] = stationary_target_world_check(query, config.task_factory())
        task_config = resolved_config["task"]
        target_model, target_body = task_config["target_contact_body"].split("::", 1)
        result["base_candidates_xyz_yaw"] = target_relative_base_candidates(
            spec.base_pose, query.frame_pose(target_model, target_body))
        payload = {"geometries": [record for record in records if not record["robot"]],
                   "task": resolved_config["task"]}
        result["stationary_target_world_check"]["geometry_task_sha256"] = hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--scene-root", type=Path, required=True)
    parser.add_argument("--experiment", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--check-stationary-target", action="store_true",
                        help="Also audit the PickLift target's static-world support contacts")
    args = parser.parse_args()
    output = args.output_root.resolve()
    output.mkdir(parents=True, exist_ok=False)
    experiment = load_experiment(args.experiment, repository_root=args.repository_root,
                                 cache_root=output / "scene_cache", scene_root=args.scene_root,
                                 trust_factories=True)
    config = experiment.environment_config
    query = build_planning_query(scenario=config.scenario, robot_adapter=config.robot_adapter,
                                 timing=config.timing)
    result = export_query_geometry(query, config, experiment.resolved_config,
                                   check_stationary_target=args.check_stationary_target)
    records = result["geometries"]
    (output / "geometry.json").write_text(json.dumps(result, indent=2, default=str) + "\n")
    print(json.dumps({"output_root": str(output), "geometry_count": len(records),
                      "robot_geometry_count": sum(item["robot"] for item in records),
                      "shape_counts": dict(Counter(item["shape"]["kind"] for item in records)),
                      "stationary_target_world_valid": (
                          result["stationary_target_world_check"]["valid"]
                          if args.check_stationary_target else None),
                      "partially_filtered_body_pairs": result["partially_filtered_body_pairs"]}))


if __name__ == "__main__":
    main()
