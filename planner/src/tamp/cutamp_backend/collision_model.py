"""Fit a cuRobo sphere model from exported Drake distance-query geometry.

Uses cuRobo's own sphere fitter, not a replacement collision/optimization
algorithm. Approximation errors are recorded; construction is not a safety
certificate. The existing Drake edge and runtime checks must remain in force.
"""

import argparse
from collections import defaultdict
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import torch
import trimesh

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.geom.sphere_fit import SphereFitType, fit_spheres_to_mesh
from curobo.rollout.cost.self_collision_cost import SelfCollisionCost, SelfCollisionCostConfig
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig


def distance_mesh(shape):
    """Match Drake pairwise signed-distance convex-hull semantics for meshes."""
    kind = shape["kind"]
    if kind == "box":
        return trimesh.creation.box(extents=shape["size_m"])
    if kind == "sphere":
        return trimesh.creation.icosphere(subdivisions=2, radius=shape["radius_m"])
    if kind == "cylinder":
        return trimesh.creation.cylinder(radius=shape["radius_m"], height=shape["length_m"], sections=48)
    if kind in {"mesh", "convex"}:
        path = Path(shape["path"])
        if hashlib.sha256(path.read_bytes()).hexdigest() != shape["sha256"]:
            raise ValueError(f"Proximity mesh changed after export: {path}")
        mesh = trimesh.load(path, force="mesh")
        mesh.apply_scale(shape["scale"])
        return mesh.convex_hull
    raise ValueError(f"No validated distance mesh conversion for {kind}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--kinematics", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--spheres-per-link", type=int, default=24)
    parser.add_argument("--surface-radius-m", type=float, default=0.005)
    args = parser.parse_args()
    if args.spheres_per_link < 1:
        parser.error("spheres-per-link must be positive")
    if not np.isfinite(args.surface_radius_m) or args.surface_radius_m <= 0:
        parser.error("surface-radius-m must be positive and finite")
    output = args.output_root.resolve()
    output.mkdir(parents=True, exist_ok=False)
    document = json.loads(args.geometry.read_text())
    reference = json.loads(args.kinematics.read_text())
    np.random.seed(500)
    link_meshes = defaultdict(list)
    for record in document["geometries"]:
        if not record["robot"]:
            continue
        mesh = distance_mesh(record["shape"])
        mesh.apply_transform(record["body_from_geometry"])
        link_meshes[record["body"]].append(mesh)
    spheres, fitting = {}, []
    for name, meshes in sorted(link_meshes.items()):
        mesh = trimesh.util.concatenate(meshes)
        centers, radii = fit_spheres_to_mesh(
            mesh, args.spheres_per_link, surface_sphere_radius=args.surface_radius_m,
            fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE, voxelize_method="subdivide",
        )
        if centers is None or not len(centers):
            raise ValueError(f"Sphere fitting returned no geometry for {name}")
        centers, radii = np.asarray(centers), np.asarray(radii).reshape(-1)
        if not np.isfinite(centers).all() or not np.isfinite(radii).all() or np.any(radii <= 0):
            raise ValueError(f"Invalid fitted spheres for {name}")
        spheres[name] = [{"center": center.tolist(), "radius": float(radius)}
                         for center, radius in zip(centers, radii, strict=True)]
        surface = np.concatenate([mesh.vertices, mesh.sample(2000)])
        residual = (np.linalg.norm(surface[:, None, :] - centers[None, :, :], axis=-1) - radii).min(axis=1)
        fitting.append({"link": name, "num_spheres": len(centers),
                        "surface_fraction_covered": float(np.mean(residual <= 0)),
                        "max_sampled_surface_gap_m": float(residual.max())})
        print(json.dumps(fitting[-1]), flush=True)
    total_spheres = sum(len(items) for items in spheres.values())
    if total_spheres >= 1024:
        raise ValueError(f"Sphere count {total_spheres} exceeds the selected kernel probe budget")
    config = copy.deepcopy(reference["robot_config"])
    kin = config["kinematics"]
    ignored = defaultdict(list)
    for first, others in document["self_collision_ignore_fully_filtered"].items():
        for second in others:
            ignored[first].append(second)
            ignored[second].append(first)
    kin.update(collision_link_names=list(spheres), collision_spheres=spheres,
               collision_sphere_buffer=0.0, self_collision_buffer={},
               self_collision_ignore=dict(ignored))
    robot = CudaRobotModel(RobotConfig.from_dict(copy.deepcopy(config)).kinematics)
    permutation = [reference["joint_names"].index(name) for name in robot.joint_names]
    q = torch.tensor(reference["configurations"], device="cuda", dtype=torch.float32)[:, permutation]
    state = robot.get_state(q)
    actual_spheres = state.get_link_spheres().detach().cpu().numpy()
    expected_home = []
    for name, values in spheres.items():
        matrix = np.array(document["base_from_robot_body"][name])
        for value in values:
            center = matrix[:3, :3] @ np.array(value["center"]) + matrix[:3, 3]
            expected_home.append([*center, value["radius"]])
    sphere_error = float(np.max(np.abs(actual_spheres[0] - np.array(expected_home))))
    args_tensor = TensorDeviceType()
    self_cost = SelfCollisionCost(SelfCollisionCostConfig(
        args_tensor.to_device([1.0]), args_tensor, return_loss=True,
        self_collision_kin_config=robot.get_self_collision_config(),
    ))
    costs = self_cost(state.get_link_spheres()[:, None].contiguous()).detach().cpu().tolist()
    ranges, offset = {}, 0
    for name, values in spheres.items():
        ranges[name] = slice(offset, offset + len(values))
        offset += len(values)
    home_collisions = []
    names = list(spheres)
    for i, first in enumerate(names):
        a = actual_spheres[0, ranges[first]]
        for second in names[i + 1:]:
            if second in ignored[first]:
                continue
            b = actual_spheres[0, ranges[second]]
            overlap = (a[:, None, 3] + b[None, :, 3]
                       - np.linalg.norm(a[:, None, :3] - b[None, :, :3], axis=-1)).max()
            if overlap > 0:
                home_collisions.append({"bodies": [first, second], "max_sphere_overlap_m": float(overlap)})
    result = {"scope": "sphere_model_construction_not_collision_certification",
              "sphere_fitter": "curobo.geom.sphere_fit.fit_spheres_to_mesh",
              "sphere_fit_type": "VOXEL_VOLUME_SAMPLE_SURFACE", "voxelize_method": "subdivide",
              "surface_radius_m": args.surface_radius_m,
              "seed": 500, "num_spheres": total_spheres, "num_links": len(spheres),
              "max_home_sphere_transform_error_m": sphere_error,
              "self_collision_costs": costs, "fitting": fitting,
              "self_collision_experimental_kernel": robot.get_self_collision_config().experimental_kernel,
              "home_sphere_collisions": sorted(home_collisions, key=lambda item: -item["max_sphere_overlap_m"]),
              "reproducibility_note": "NumPy seed set; upstream trimesh surface fallback can use its own RNG. Retain fitted artifact.",
              "kinematic_transform_passed": sphere_error < 1e-5,
              "collision_model_validated": False,
              "source_geometry_sha256": hashlib.sha256(args.geometry.read_bytes()).hexdigest()}
    (output / "robot_config.json").write_text(json.dumps(config, indent=2) + "\n")
    (output / "sphere_fit_report.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "fitting"}), flush=True)
    if not result["kinematic_transform_passed"]:
        raise RuntimeError("Drake/cuRobo sphere frame mismatch")


if __name__ == "__main__":
    main()
