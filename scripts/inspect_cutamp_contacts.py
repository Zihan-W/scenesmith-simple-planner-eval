"""Locate static OBBs overlapping the retained cuTAMP target-sphere model.

This is a geometric diagnostic, not a replacement optimizer cost or a safety
decision. Positive overlaps must be compared with original Drake contact rules.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from curobo.types.math import Pose


def matrix(values):
    return Pose.from_list(values).get_matrix().detach().cpu().numpy().reshape(4, 4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--problem", type=Path, required=True)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--robot-particle", type=int,
                        help="Inspect a retained robot candidate instead of the movable target")
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    problem = json.loads(args.problem.read_text())
    geometry = json.loads(args.geometry.read_text())
    records = {f"drake_{record['id']}": record for record in geometry["geometries"]}
    sphere_links = None
    if args.robot_particle is None:
        spheres = np.array(problem["movable_spheres_local"])
        transform = matrix(problem["movable_pose"])
        centers = spheres[:, :3] @ transform[:3, :3].T + transform[:3, 3]
    else:
        import copy
        import torch
        from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
        from curobo.types.robot import RobotConfig

        config = json.loads(Path(problem["inputs"]["robot_config"]["path"]).read_text())
        robot = CudaRobotModel(RobotConfig.from_dict(copy.deepcopy(config)).kinematics)
        candidates = json.loads((args.problem.parent / "candidate_assignments.json").read_text())["candidates"]
        candidate = next(item for item in candidates if item["particle_index"] == args.robot_particle)
        q = torch.tensor([[candidate["arm_joint_positions"][candidate["arm_joint_names"].index(name)]
                           for name in robot.joint_names]], device="cuda", dtype=torch.float32)
        spheres = robot.get_state(q).get_link_spheres()[0].detach().cpu().numpy()
        anchor_from_base = np.linalg.inv(np.array(problem["fixed_base_world_matrix"])) @ matrix(candidate["base_world_pose"])
        centers = spheres[:, :3] @ anchor_from_base[:3, :3].T + anchor_from_base[:3, 3]
        sphere_links = np.full(len(spheres), "unknown", dtype=object)
        for name in config["kinematics"]["collision_spheres"]:
            indices = robot.kinematics_config.get_sphere_index_from_link_name(name).cpu().numpy()
            sphere_links[indices] = name
    overlaps = []
    for obstacle in problem["statics"]:
        transform = matrix(obstacle["pose"])
        local = (centers - transform[:3, 3]) @ transform[:3, :3]
        delta = np.abs(local) - np.array(obstacle["dims"]) / 2
        distance = np.linalg.norm(np.maximum(delta, 0), axis=1) + np.minimum(delta.max(axis=1), 0)
        penetration = spheres[:, 3] - distance
        if penetration.max() <= 0:
            continue
        record = records[obstacle["name"]]
        overlaps.append({"geometry_id": record["id"], "name": record["name"],
                         "body": f"{record['model']}::{record['body']}", "ground": record["ground"],
                         "max_sphere_overlap_m": float(penetration.max()),
                         "sum_positive_sphere_overlaps_m": float(np.maximum(penetration, 0).sum()),
                         "robot_links": sorted(set(sphere_links[penetration > 0])) if sphere_links is not None else None,
                         "overlapping_spheres": int(np.sum(penetration > 0))})
    result = {"scope": "sphere_vs_obb_overlap_diagnostic_not_contact_permission",
              "overlaps": sorted(overlaps, key=lambda item: -item["sum_positive_sphere_overlaps_m"])}
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
