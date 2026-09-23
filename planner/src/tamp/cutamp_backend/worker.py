"""Run official cuTAMP optimization on Zerith/SceneSmith diagnostic problems.

The default fixes the base; the explicit mobile mode optimizes world XY along
with arm configurations. Neither mode is the complete skill backend. The
supplied whole-robot sphere model is unvalidated; no result is executed and no
physical success or safety certificate is claimed.
"""

import argparse
import copy
import dataclasses
import hashlib
import inspect
import json
from pathlib import Path

import numpy as np
import roma
import torch
import trimesh

from .collision_model import distance_mesh
from .static_bounds import static_oriented_bounds
from .stationary_contacts import StationaryTargetWorldCost
from .stationary_support import StationarySupportWorldCost, PlanarSupportWorldCost
from .planar_kinematics import PlanarTranslationKinematics
from .base_cost import BaseDomainCost
from planner.src.tamp.cutamp_base_domain import base_domain_halfspaces
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.geom.types import Cuboid
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from cutamp.config import TAMPConfiguration
from cutamp.constraint_checker import ConstraintChecker
from cutamp.cost_function import CostFunction
from cutamp.cost_reduction import CostReducer
from cutamp.envs import TAMPEnvironment
from cutamp.optimize_plan import ParticleOptimizer
from cutamp.particle_initialization import ParticleInitializer
from cutamp.robots import RobotContainer
from cutamp.rollout import RolloutFunction
from cutamp.tamp_domain import Holding, MoveFree, Pick
from cutamp.tamp_world import TAMPWorld
from cutamp.utils.shapes import sample_greedy_surface_spheres
from cutamp.utils.timer import TorchTimer
from cutamp.utils.common import action_6dof_to_mat4x4, pose_list_to_mat4x4
from cutamp.utils.visualizer import MockVisualizer


def pose_list(matrix):
    return Pose.from_matrix(torch.as_tensor(matrix, dtype=torch.float32, device="cuda")[None]).to_list()


def seed_calibrated_grasps(initializer, calibration, target_spec, body_from_center,
                           *, base_samples=None, world_from_anchor=None,
                           lateral_range_m=(-0.02, 0.02)):
    """Fill the official pick cache with calibrated poses and real cuRobo IK.

    This is explicitly a modified domain initializer, not the official generic
    grasp sampler. Pose conventions match the runtime's target-body-relative
    TCP calibration. Its +/-2 cm lateral domain matches SceneSmithPickDomain.
    No IK failure is relabelled successful or removed from the final checks.
    """
    if (calibration["target_model_name"] != target_spec["model_instance_name"]
            or calibration["target_body_name"] != target_spec["body_name"]):
        raise ValueError("Grasp calibration target differs from the exported body")
    world = initializer.world
    count = initializer.config.num_particles
    calibrated = calibration["grasp_pose_in_target"]
    body_from_tcp = pose_list_to_mat4x4([
        *calibrated["translation_m"], *calibrated["quaternion_wxyz"]
    ]).to(world.tensor_args.device).repeat(count, 1, 1)
    low, high = lateral_range_m
    if not np.isfinite([low, high]).all() or not -0.02 <= low < high <= 0.02:
        raise ValueError("Grasp subdomain cannot expand the registered +/-2 cm envelope")
    offsets = torch.rand(count, device=world.tensor_args.device) * (high - low) + low
    body_from_tcp[:, 1, 3] += offsets
    center_from_body = torch.as_tensor(np.linalg.inv(body_from_center),
                                      device=world.tensor_args.device, dtype=torch.float32)
    center_from_tcp = center_from_body @ body_from_tcp
    grasps = torch.cat((center_from_tcp[:, :3, 3],
                        roma.rotmat_to_euler("XYZ", center_from_tcp[:, :3, :3])), dim=-1)
    roundtrip_error = float((action_6dof_to_mat4x4(grasps) - center_from_tcp).abs().max())
    if roundtrip_error > 2e-6:
        raise ValueError(f"Official 6-DOF grasp conversion failed: {roundtrip_error}")
    target = world.get_object(target_spec["observation_name"])
    base_from_center = pose_list_to_mat4x4(target.pose).to(world.tensor_args.device)
    base_from_ee = base_from_center @ center_from_tcp @ world.tool_from_ee
    if base_samples is not None:
        anchor = torch.as_tensor(world_from_anchor, device=world.tensor_args.device, dtype=torch.float32)
        delta_world = torch.cat((base_samples - anchor[:2, 3], torch.zeros_like(base_samples[:, :1])), dim=-1)
        # The real IK solver has only the arm joints. Express each desired TCP
        # relative to its sampled base; do not ask arm IK to move the base.
        base_from_ee = base_from_ee.clone()
        base_from_ee[:, :3, 3] -= delta_world @ anchor[:3, :3]
    ik_result = world.ik_solver.solve_batch(Pose.from_matrix(base_from_ee), seed_config=None)
    initializer.pick_cache[target.name] = {"sampled_grasps": grasps, "ik_result": ik_result}
    return {"kind": "modified_calibrated_domain_via_official_pick_cache_and_curobo_ik",
            "grasp_lateral_offsets_m": offsets.detach().cpu().tolist(),
            "ik_success_count": int(ik_result.success.sum()),
            "ik_solve_time_s": float(ik_result.solve_time),
            "pose_conversion_max_component_error": roundtrip_error,
            "grasp_gradient_optimized": False}


def exported_base_bounds(document, yaw):
    """Return broad XY bounds; BaseDomainCost enforces the actual convex hull."""
    base_domain_halfspaces(document["base_candidates_xyz_yaw"], yaw)
    candidates = np.asarray(document["base_candidates_xyz_yaw"])
    return np.stack((candidates[:, :2].min(axis=0), candidates[:, :2].max(axis=0)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--kinematics", type=Path, required=True)
    parser.add_argument("--robot-config", type=Path, required=True)
    parser.add_argument("--multipliers", type=Path, required=True)
    parser.add_argument("--tolerances", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--num-particles", type=int, default=64)
    parser.add_argument("--num-opt-steps", type=int, default=200)
    parser.add_argument("--conf-lr", type=float, default=0.01)
    parser.add_argument("--seed", type=int, default=500)
    parser.add_argument("--grasp-calibration", type=Path,
                        help="Explicit modified calibrated-domain initialization; default is upstream sampling")
    parser.add_argument("--exact-stationary-target-contact", action="store_true",
                        help="Use the matching passing Drake audit for the constant target/world constraint")
    parser.add_argument("--optimize-base-xy", action="store_true",
                        help="Optimize world XY and arm joints together at the exported domain's fixed heading")
    parser.add_argument("--exact-stationary-robot-support", action="store_true",
                        help="Use matching exact wheel/floor permissions only for an unchanged observed base")
    parser.add_argument("--exact-planar-robot-support", action="store_true",
                        help="Use exact support only over an audited horizontal-floor domain")
    args = parser.parse_args()
    if args.exact_planar_robot_support and not args.optimize_base_xy:
        parser.error("Planar support requires --optimize-base-xy")
    if args.num_particles <= 0 or args.num_opt_steps <= 0:
        parser.error("Particle and optimization-step counts must be positive")
    if not np.isfinite(args.conf_lr) or args.conf_lr <= 0:
        parser.error("conf-lr must be positive and finite")
    if args.exact_stationary_robot_support and args.optimize_base_xy:
        parser.error("A moving base cannot use the stationary robot support audit")
    if not 0 <= args.seed < 2**32:
        parser.error("seed must be in [0, 2**32)")
    output = args.output_root.resolve()
    output.mkdir(parents=True, exist_ok=False)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    document = json.loads(args.geometry.read_text())
    reference = json.loads(args.kinematics.read_text())
    robot_config = json.loads(args.robot_config.read_text())
    tensor_args = TensorDeviceType()
    kin_model = CudaRobotModel(RobotConfig.from_dict(copy.deepcopy(robot_config)).kinematics)
    permutation = [reference["joint_names"].index(name) for name in kin_model.joint_names]
    q_init = tensor_args.to_device(reference["configurations"][0])[permutation]
    if "observed_robot_base_world_matrix" in document:
        if document["observation_id"] != reference.get("observation_id"):
            raise ValueError("Geometry and kinematics snapshots have different timestamps")
        if robot_config != reference["robot_config"]:
            raise ValueError("Observed kinematics and collision robot configs differ")
        world_from_base = np.asarray(document["observed_robot_base_world_matrix"], dtype=float)
        if (world_from_base.shape != (4, 4) or not np.isfinite(world_from_base).all()
                or not np.allclose(world_from_base[3], [0, 0, 0, 1])
                or not np.allclose(world_from_base[:3, :3].T @ world_from_base[:3, :3], np.eye(3))
                or not np.isclose(np.linalg.det(world_from_base[:3, :3]), 1)):
            raise ValueError("Invalid observed base rigid transform")
        actual_fk = kin_model.get_state(q_init[None]).ee_pose.get_matrix()[0].detach().cpu().numpy()
        expected_fk = np.asarray(reference["base_from_tcp"][0])
        if not np.allclose(actual_fk, expected_fk, atol=1e-5, rtol=0):
            raise ValueError("Observed Drake/CUDA forward kinematics disagree")
        yaw = np.arctan2(world_from_base[1, 0], world_from_base[0, 0])
    else:
        options = document["resolved_experiment"]["user_config"]["policy_options"]
        yaw = np.deg2rad(options["calibrated_base_yaw_deg"])
        world_from_base = np.array([[np.cos(yaw), -np.sin(yaw), 0, 0],
                                    [np.sin(yaw), np.cos(yaw), 0, 0],
                                    [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        world_from_base[:3, 3] = options["calibrated_base_xyz_m"]
    base_from_world = np.linalg.inv(world_from_base)
    observed = document["observed_bodies"]
    if len(observed) != 1:
        raise ValueError("This interface probe requires exactly one observed movable target")
    target_spec = observed[0]
    target_meshes, target_boxes, statics, target_pose = [], [], [], None
    for record in document["geometries"]:
        if record["robot"]:
            continue
        mesh = distance_mesh(record["shape"])
        if record["model"] == target_spec["model_instance_name"] and record["body"] == target_spec["body_name"]:
            body_from_geometry = np.array(record["body_from_geometry"])
            if record["shape"]["kind"] != "box" or not np.allclose(body_from_geometry[:3, :3], np.eye(3)):
                raise ValueError("The upstream 6-DOF grasp sampler needs an exactly representable cuboid")
            mesh.apply_transform(body_from_geometry)
            target_meshes.append(mesh)
            target_boxes.append(mesh.bounds.copy())
            target_pose = base_from_world @ np.array(record["world_from_geometry"]) @ np.linalg.inv(body_from_geometry)
        else:
            box_transform, box_extents = static_oriented_bounds(mesh)
            matrix = base_from_world @ np.array(record["world_from_geometry"]) @ box_transform
            statics.append(Cuboid(name=f"drake_{record['id']}", pose=pose_list(matrix),
                                   dims=box_extents.tolist()))
    if not target_meshes:
        raise ValueError("No target proximity geometry found")
    target_mesh = trimesh.util.concatenate(target_meshes)
    bounds = target_mesh.bounds
    dimensions = bounds[1] - bounds[0]
    # Verify that the four actual collision boxes tile a single cuboid exactly;
    # do not silently replace a non-cuboid movable by its bounding box.
    volume = sum(float(np.prod(box[1] - box[0])) for box in target_boxes)
    for index, first in enumerate(target_boxes):
        for second in target_boxes[index + 1:]:
            overlap = np.maximum(0, np.minimum(first[1], second[1]) - np.maximum(first[0], second[0]))
            if np.prod(overlap) > 1e-12:
                raise ValueError("Target component boxes overlap; exact cuboid tiling is unproven")
    if not np.isclose(volume, np.prod(dimensions), rtol=1e-6, atol=1e-12):
        raise ValueError("Target collision boxes do not fill their bounding cuboid")
    body_from_center = np.eye(4)
    body_from_center[:3, 3] = bounds.mean(axis=0)
    target = Cuboid(name=target_spec["observation_name"], pose=pose_list(target_pose @ body_from_center),
                    dims=dimensions.tolist())
    env = TAMPEnvironment("zerith_fixed_base_scene_probe", [target], statics,
                          {"Movable": [target]}, frozenset({Holding.ground(target.name)}))
    # Robot gripper spheres use the actual TCP frame, not Panda's tool convention.
    base_from_tcp = np.array(reference["base_from_tcp"][0])
    tcp_from_base = np.linalg.inv(base_from_tcp)
    gripper_spheres = []
    for name, values in robot_config["kinematics"]["collision_spheres"].items():
        if name not in {"left_end_effector_link", "left_jaw_left_finger_link", "left_jaw_right_finger_link"}:
            continue
        matrix = tcp_from_base @ np.array(document["base_from_robot_body"][name])
        for value in values:
            center = matrix[:3, :3] @ value["center"] + matrix[:3, 3]
            gripper_spheres.append([*center, value["radius"]])
    robot = RobotContainer("zerith_dual_left", kin_model,
                           kin_model.kinematics_config.joint_limits.position,
                           tensor_args.to_device(gripper_spheres), torch.eye(4, device="cuda"))
    ik_solver = IKSolver(IKSolverConfig.load_from_robot_config(
        RobotConfig.from_dict(copy.deepcopy(robot_config)), num_seeds=12,
        self_collision_check=True, self_collision_opt=False, use_cuda_graph=False))
    world = TAMPWorld(env, tensor_args, robot, q_init, ik_solver=ik_solver,
                      sphere_seed=args.seed)
    movable_spheres_local = world.get_collision_spheres(target).detach().cpu().tolist()
    movable_spheres_sha256 = hashlib.sha256(json.dumps(
        movable_spheres_local, separators=(",", ":")).encode()).hexdigest()
    sphere_source_sha256 = {
        name: hashlib.sha256(Path(inspect.getfile(function)).read_bytes()).hexdigest()
        for name, function in (("tamp_world", TAMPWorld),
                               ("sphere_sampler", sample_greedy_surface_spheres),
                               ("trimesh_surface_sampler", trimesh.sample.sample_surface))
    }
    config = TAMPConfiguration(robot=robot.name, grasp_dof=6, random_init=args.grasp_calibration is None,
                               num_particles=args.num_particles, num_opt_steps=args.num_opt_steps,
                               enable_visualizer=False, lr=0.007, conf_lr=args.conf_lr)
    skeleton = [MoveFree.ground({"q_start": "q0", "traj": "t0", "q_end": "q_pick"}),
                Pick.ground({"obj": target.name, "grasp": "g0", "q": "q_pick"})]
    initializer = ParticleInitializer(world, config)
    base_bounds = exported_base_bounds(document, yaw) if args.optimize_base_xy else None
    base_samples = None
    if base_bounds is not None:
        anchors = tensor_args.to_device(document["base_candidates_xyz_yaw"])[:, :2]
        indices = torch.randint(len(anchors), (args.num_particles, 2), device="cuda")
        weight = torch.rand((args.num_particles, 1), device="cuda")
        base_samples = anchors[indices[:, 0]] * weight + anchors[indices[:, 1]] * (1 - weight)
    initialization = {"kind": "official_random_configuration_and_6dof_grasp_sampler",
                      "grasp_gradient_optimized": False}
    if args.grasp_calibration is not None:
        initialization = seed_calibrated_grasps(
            initializer, json.loads(args.grasp_calibration.read_text()), target_spec, body_from_center,
            base_samples=base_samples, world_from_anchor=world_from_base,
            lateral_range_m=document.get("grasp_lateral_range_m", (-0.02, 0.02)))
    particles = initializer(skeleton)
    if particles is None:
        raise RuntimeError("Official particle initialization failed")
    if args.optimize_base_xy:
        mobile = PlanarTranslationKinematics(kin_model, tensor_args.to_device(world_from_base),
                                            tensor_args.to_device(base_bounds))
        for name in ("q0", "q_pick"):
            # Lift actual initialized arm configurations into the new numeric
            # representation. No symbolic operators or grasp samples change.
            xy = (base_samples if name == "q_pick" else
                  tensor_args.to_device(world_from_base[:2, 3]).expand(args.num_particles, -1))
            particles[name] = torch.cat((xy, particles[name]), dim=-1)
        world.robot_container = dataclasses.replace(robot, kin_model=mobile, joint_limits=mobile.joint_limits)
        world.q_init = particles["q0"][0].clone()
        initialization["base_representation"] = "world_xy_m_then_actual_arm_joints"
        initialization["base_seed_world_xy_m"] = base_samples.detach().cpu().tolist()
        initialization["base_seed_may_be_outside_domain"] = False
        initialization["arm_ik_conditioned_on_sampled_base"] = args.grasp_calibration is not None
    scope = ("mobile_xy_optimizer_probe_not_skill_backend_acceptance" if args.optimize_base_xy
             else "fixed_base_interface_probe_not_skill_backend_acceptance")
    input_paths = {name: getattr(args, name) for name in
                   ("geometry", "kinematics", "robot_config", "multipliers", "tolerances")}
    if args.grasp_calibration is not None:
        input_paths["grasp_calibration"] = args.grasp_calibration
    (output / "problem.json").write_text(json.dumps({
        "scope": scope,
        "seed": args.seed,
        "inputs": {name: {"path": str(path.resolve()),
                           "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                   for name, path in input_paths.items()},
        "config": dataclasses.asdict(config), "skeleton": [str(op) for op in skeleton],
        "initialization": initialization,
        "exact_stationary_target_contact": args.exact_stationary_target_contact,
        "exact_stationary_robot_support": args.exact_stationary_robot_support,
        "exact_planar_robot_support": args.exact_planar_robot_support,
        "base_xy_bounds_world_m": base_bounds.tolist() if base_bounds is not None else None,
        "fixed_base_world_matrix": world_from_base.tolist(),
        "movable_pose": target.pose, "movable_dims": target.dims,
        "movable_spheres_local": movable_spheres_local,
        "movable_spheres_sha256": movable_spheres_sha256,
        "movable_sphere_seed_derivation": "sha256(worker_seed:object_name) first 64 bits",
        "movable_sphere_source_sha256": sphere_source_sha256,
        "statics": [{"name": obstacle.name, "pose": obstacle.pose, "dims": obstacle.dims}
                    for obstacle in statics],
    }, indent=2) + "\n")
    torch.save({name: values.detach().cpu() for name, values in particles.items()}, output / "initial_particles.pt")
    rollout = RolloutFunction(skeleton, world, config)
    cost_fn = CostFunction(skeleton, world, config, allow_nonexperimental_self_collision=True)
    target_cost = None
    if args.exact_stationary_target_contact:
        cost_fn = StationaryTargetWorldCost(cost_fn, document, target.name)
        target_cost = cost_fn
    support_cost = None
    if args.exact_stationary_robot_support:
        cost_fn = StationarySupportWorldCost(cost_fn, world, document, statics)
        support_cost = cost_fn
    if args.exact_planar_robot_support:
        cost_fn = PlanarSupportWorldCost(cost_fn, world, document, statics)
        support_cost = cost_fn
    multipliers = json.loads(args.multipliers.read_text())
    tolerances = json.loads(args.tolerances.read_text())
    if args.optimize_base_xy:
        halfspaces = tensor_args.to_device(base_domain_halfspaces(document["base_candidates_xyz_yaw"], yaw))
        cost_fn = BaseDomainCost(cost_fn, halfspaces, rollout.conf_params.index("q_pick"))
        multipliers["BaseDomain"] = {"outside_m": 1.0}
        tolerances["BaseDomain"] = {"outside_m": 0.0}
    reducer = CostReducer(multipliers)
    checker = ConstraintChecker(tolerances)
    initial = reducer.hard_costs(cost_fn(rollout(particles))).detach().clone()
    timer = TorchTimer()
    timer.start("start_optimization")
    found, metrics, timed_out = ParticleOptimizer(config, reducer, checker)(
        {"plan_skeleton": skeleton, "particles": particles, "rollout_fn": rollout,
         "cost_fn": cost_fn, "heuristic": 0.0}, timer, MockVisualizer())
    optimization_time = timer.stop("start_optimization")
    with torch.no_grad():
        costs = cost_fn(rollout(particles))
        final = reducer.hard_costs(costs)
        mask = checker.get_mask(costs, verbose=False)
        eligible = torch.nonzero(mask).flatten()
        selected = int(eligible[reducer.soft_costs(costs)[eligible].argmin()]) if len(eligible) else None
        masks = checker.get_full_mask(costs)
        constraint_summary = {group: {name: {
            "minimum": float(values.min()), "maximum": float(values.max()),
            "mean": float(values.mean()), "satisfying_elements": int(masks[group][name].sum()),
        } for name, values in entry["values"].items()}
            for group, entry in costs.items() if entry["type"] == "constraint"}
    optimizer_path = Path(inspect.getfile(ParticleOptimizer)).resolve()
    result = {"scope": scope,
              "seed": args.seed,
              "movable_spheres_sha256": movable_spheres_sha256,
              "physical_rollout_executed": False, "collision_model_validated": False,
              "base_pose_optimized": args.optimize_base_xy, "fixed_base_world_pose": pose_list(world_from_base),
              "base_xy_bounds_world_m": base_bounds.tolist() if base_bounds is not None else None,
              "initialization": initialization,
              "exact_stationary_target_contact": args.exact_stationary_target_contact,
              "exact_stationary_robot_support": args.exact_stationary_robot_support,
        "exact_planar_robot_support": args.exact_planar_robot_support,
              "original_approximate_robot_world_cost": (
                  float(support_cost.original_robot_world.mean()) if support_cost is not None else None),
              "original_approximate_movable_world_cost": (
                  float(target_cost.original_movable_world.mean())
                  if args.exact_stationary_target_contact else None),
              "found_solution_in_approximate_model": found, "timed_out": timed_out,
              "num_particles": args.num_particles, "num_opt_steps": len(metrics["loss"]),
              "optimization_time": optimization_time,
              "initial_constraint_cost": float(initial.mean()), "final_constraint_cost": float(final.mean()),
              "num_satisfying_particles": int(mask.sum()), "selected_particle": selected,
              "selected_arm_joint_names": list(kin_model.joint_names),
              "selected_arm_joint_positions": (
                  particles["q_pick"][selected, 2 if args.optimize_base_xy else 0:].detach().cpu().tolist()
                  if selected is not None else None),
              "selected_base_world_pose": (
                  [*particles["q_pick"][selected, :2].detach().cpu().tolist(), *pose_list(world_from_base)[2:]]
                  if selected is not None and args.optimize_base_xy
                  else pose_list(world_from_base) if selected is not None else None),
              "optimized_base_xy_range_world_m": (
                  [particles["q_pick"][:, :2].detach().amin(dim=0).cpu().tolist(),
                   particles["q_pick"][:, :2].detach().amax(dim=0).cpu().tolist()]
                  if args.optimize_base_xy else None),
              "selected_grasp_lateral_offset_m": (
                  initialization["grasp_lateral_offsets_m"][selected]
                  if selected is not None and args.grasp_calibration is not None else None),
              "constraint_summary": constraint_summary,
              "selected_initial_constraint_cost": float(initial[selected]) if selected is not None else None,
              "selected_final_constraint_cost": float(final[selected]) if selected is not None else None,
              "optimizer_source": str(optimizer_path),
              "optimizer_sha256": hashlib.sha256(optimizer_path.read_bytes()).hexdigest(),
              "config": dataclasses.asdict(config), "official_metrics": metrics,
              "static_geometry_count": len(statics), "timer": timer.get_summaries()}
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    candidates = []
    for index in range(args.num_particles):
        q = particles["q_pick"][index].detach().cpu().tolist()
        base_pose = pose_list(world_from_base)
        candidates.append({
            "particle_index": index, "optimizer_feasible": bool(mask[index]),
            "hard_constraint_cost": float(final[index]),
            "constraint_diagnostics": {
                group: {name: {"values": values[index].detach().cpu().reshape(-1).tolist(),
                               "tolerance": checker._get_tol(group, name),
                               "satisfied": bool(masks[group][name][index].all())}
                        for name, values in entry["values"].items()}
                for group, entry in costs.items() if entry["type"] == "constraint"},
            "arm_joint_names": list(kin_model.joint_names),
            "arm_joint_positions": q[2:] if args.optimize_base_xy else q,
            "base_world_pose": [*q[:2], *base_pose[2:]] if args.optimize_base_xy else base_pose,
            "grasp_lateral_offset_m": (initialization["grasp_lateral_offsets_m"][index]
                                       if args.grasp_calibration is not None else None),
        })
    (output / "candidate_assignments.json").write_text(json.dumps({
        "scope": "optimizer_candidates_require_exact_postchecks_not_execution_authorization",
        "candidates": candidates}, indent=2) + "\n")
    torch.save({name: values.detach().cpu() for name, values in particles.items()}, output / "particles.pt")
    print(json.dumps({key: value for key, value in result.items() if key not in {"official_metrics", "config", "timer"}}))


if __name__ == "__main__":
    main()
