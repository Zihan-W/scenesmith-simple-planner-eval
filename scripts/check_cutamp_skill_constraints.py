"""Check a retained GPU assignment through the shared SceneSmith skill domain.

The default is a geometric postcheck including navigation, approach, unchanged
grasp endpoint, contacts and lift. Explicit --execute uses the existing shared
skills and records physics, with fresh checks between skills. Neither mode
replaces the supplied grasp IK or constitutes the complete online D backend.
"""

import argparse
import dataclasses
import json
from pathlib import Path

from cutamp_observed_resolve import resolve_observed_pick
from examples.online_manipulation.tamp_planner import Subgoal
from examples.online_manipulation.tamp_scenesmith import SceneSmithPickDomain
from examples.online_manipulation.tamp_geometry import _bind_effect, _parameter_value
from examples.online_manipulation.tamp_hierarchy import ParameterizedSkillAction, picklift_registry
from examples.online_manipulation.tamp_scenesmith_online import SceneSmithSkillExecutor, SceneSmithWorldObserver
from src.online_manipulation import Pose, make_env
from src.online_manipulation.adapters.description import drake_pose
from src.online_manipulation.experiment import load_experiment


def execute_checked_candidate(env, experiment, domain, observation, reset_info,
                              repo, output, nav, pick, *, fresh_resolve=None):
    """Execute with fresh checks and optional observed-state grasp re-solving."""
    executor = SceneSmithSkillExecutor(
        env=env, experiment=experiment, repository_root=repo, output_root=output,
        observation=observation, reset_info=reset_info)
    observer = SceneSmithWorldObserver(executor, domain.target_name, output)
    registry = picklift_registry()
    state = {"base_height_m": domain.config.robot_adapter.base_config.base_height_m}
    outcomes = []
    env.start_recording()
    try:
        for name, assignment, bindings in (
                ("NavigateToPick", nav, {"base_pose": "b0"}),
                ("PickLift", pick, {"grasp_pose": "g0", "approach_pose": "a0"})):
            domain.observation = executor.observation
            state = observer.geometry_state(executor.observation, state)
            if name == "PickLift" and fresh_resolve is not None:
                world = observer.observe(executor.observation)
                assignment = resolve_observed_pick(
                    domain, world, experiment, repo, output / "fresh_pick_resolve", **fresh_resolve)
                if assignment is None:
                    outcomes.append({"skill": name, "executed": False,
                                     "reason": "fresh_cutamp_no_exact_feasible_assignment",
                                     "observation_id": world.observation_id})
                    break
            feasible, reason, details = domain.check(Subgoal(name, {"target": domain.target_name}), assignment, state)
            if not feasible:
                outcomes.append({"skill": name, "executed": False, "reason": reason,
                                 "fresh_geometry_check": details})
                break
            spec = registry[name]
            arguments = {"object": domain.target_name}
            values = {variable: _parameter_value(parameter, assignment, details)
                      for parameter, variable in bindings.items()}
            action = ParameterizedSkillAction(
                name, arguments, {**assignment, **values, "checks": details},
                tuple(_bind_effect(effect, arguments) for effect in spec.add_effects),
                spec.supports_geometric_conditioning, bindings)
            (output / f"action_{len(outcomes)}.json").write_text(json.dumps(dataclasses.asdict(action), indent=2) + "\n")
            outcome = executor.execute(action)
            world = observer.observe(outcome.observation)
            verified = all(effect in world.facts for effect in action.expected_effects)
            images = observer.capture_images(outcome.observation)
            outcomes.append({"skill": name, "executed": True, "success": outcome.success,
                             "verified": verified, "reason": outcome.reason,
                             "geometric_conditioning": dict(outcome.geometric_conditioning),
                             "world": {"objects": dict(world.objects), "robot": dict(world.robot),
                                       "observation_id": world.observation_id,
                                       "facts": [dataclasses.asdict(fact) for fact in sorted(world.facts, key=repr)]},
                             "images": images})
            if not outcome.success or not verified or outcome.episode_finished:
                break
    finally:
        env.save_recording(output / "simulation.html")
        (output / "execution_outcomes.json").write_text(json.dumps(outcomes, indent=2) + "\n")
    return {"success": bool(executor.observation.task.get("success", False)),
            "task": dict(executor.observation.task), "outcomes": outcomes,
            "recording": str(output / "simulation.html")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--scene-root", type=Path, required=True)
    parser.add_argument("--experiment", type=Path, required=True)
    parser.add_argument("--probe-root", type=Path, required=True)
    parser.add_argument("--particle-index", type=int, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--approach-segments", type=int, default=1)
    parser.add_argument("--seed", type=int, default=500)
    parser.add_argument("--execute", action="store_true",
                        help="Execute through shared skills with mandatory recording and fresh inter-skill checks")
    parser.add_argument("--resolve-after-navigation", action="store_true",
                        help="Re-solve the remaining fixed PickLift skeleton from the actual observed state")
    parser.add_argument("--gpu-python", type=Path)
    parser.add_argument("--cutamp-root", type=Path)
    args = parser.parse_args()
    if args.resolve_after_navigation and not (args.execute and args.gpu_python and args.cutamp_root):
        parser.error("--resolve-after-navigation requires --execute, --gpu-python and --cutamp-root")
    candidates = json.loads((args.probe_root / "candidate_assignments.json").read_text())["candidates"]
    candidate = next(item for item in candidates if item["particle_index"] == args.particle_index)
    if not candidate["optimizer_feasible"]:
        raise ValueError("Particle does not pass the optimizer's feasibility checks")
    output, repo = args.output_root.resolve(), args.repository_root.resolve()
    output.mkdir(parents=True, exist_ok=False)
    experiment = load_experiment(args.experiment, repository_root=repo,
                                 cache_root=output / "scene_cache", scene_root=args.scene_root,
                                 trust_factories=True, meshcat=args.execute)
    env = make_env(experiment.environment_config)
    observation, reset_info = env.reset(seed=args.seed)
    options = experiment.resolved_config["user_config"]["policy_options"]
    domain = SceneSmithPickDomain(
        environment_config=experiment.environment_config, observation=observation,
        calibration_path=repo / options["expert_calibration"],
        pick_home_path=repo / "experiments/inputs/pick_lift/pick_home.json",
        open_width_m=options.get("expert_policy_overrides", {}).get("open_width_m"),
        lift_distance_m=options.get("expert_policy_overrides", {}).get("lift_distance_m"))
    arm_names = domain.config.robot_adapter.spec.arm_groups["left"]
    if set(arm_names) != set(candidate["arm_joint_names"]):
        raise ValueError("Particle and runtime arm joint names differ")
    q = [candidate["arm_joint_positions"][candidate["arm_joint_names"].index(name)] for name in arm_names]
    base = candidate["base_world_pose"]
    pose = Pose(tuple(base[:3]), tuple(base[3:]))
    yaw = drake_pose(pose).rotation().ToRollPitchYaw().yaw_angle()
    start = observation.base["base_link_pose"]
    state = {"base_height_m": domain.config.robot_adapter.base_config.base_height_m,
             "base_pose": Pose(tuple(start["translation_m"]), tuple(start["quaternion_wxyz"]))}
    nav = {"base_x_m": base[0], "base_y_m": base[1], "base_yaw_rad": yaw}
    pick = {"target": domain.target_name, "arm": "left",
            "grasp_lateral_offset_m": candidate["grasp_lateral_offset_m"],
            "grasp_arm_joint_positions": q, "approach_segments": args.approach_segments,
            "approach_height_offset_m": 0.0}
    checks = []
    for name, assignment in (("NavigateToPick", nav), ("PickLift", pick)):
        skill = Subgoal(name, {"target": domain.target_name})
        feasible, reason, details = domain.check(skill, assignment, state)
        if name == "PickLift" and feasible:
            if details["ik"]["grasp_pose_in_target"]["arm_joint_positions"] != q:
                raise AssertionError("The shared domain replaced the supplied GPU grasp configuration")
        checks.append({"skill": name, "feasible": feasible, "reason": reason, "details": details})
        if not feasible:
            break
        state = domain.predict(skill, assignment, state)
    result = {"scope": "shared_domain_postcheck_not_physical_execution_or_cli_backend_acceptance",
              "particle_index": args.particle_index, "probe_root": str(args.probe_root.resolve()),
              "physical_rollout_executed": False, "seed": args.seed,
              "feasible": len(checks) == 2 and all(check["feasible"] for check in checks),
              "grasp_configuration_replaced_by_ik": False, "candidate": candidate,
              "approach_segments": args.approach_segments, "checks": checks}
    if args.execute and result["feasible"]:
        (output / "planner_config.json").write_text(json.dumps({
            "scope": "retained_cutamp_particle_execution_probe_not_full_D_mode",
            "skill_planner": "explicit_fixed_skeleton", "geometry_backend": "real_cutamp_retained_particle",
            "seed": args.seed, "particle_index": args.particle_index,
            "resolve_after_navigation": args.resolve_after_navigation,
            "source_probe": str(args.probe_root.resolve()), "record_html": True}, indent=2) + "\n")
        (output / "resolved_experiment.json").write_text(json.dumps(experiment.resolved_config, indent=2) + "\n")
        result["scope"] = "retained_cutamp_particle_shared_runtime_probe_not_full_D_mode"
        result["execution"] = execute_checked_candidate(
            env, experiment, domain, observation, reset_info, repo, output, nav, pick,
            fresh_resolve=({"source_probe": args.probe_root, "gpu_python": args.gpu_python,
                            "cutamp_root": args.cutamp_root} if args.resolve_after_navigation else None))
        result["physical_rollout_executed"] = any(item["executed"] for item in result["execution"]["outcomes"])
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key not in {"checks", "candidate"}}))


if __name__ == "__main__":
    main()
