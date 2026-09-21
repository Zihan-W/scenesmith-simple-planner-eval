"""One SceneSmith VLM-subgoal -> constrained-skill -> BT -> simulation pipeline."""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import os
from collections.abc import Mapping
from pathlib import Path

from PIL import Image

from planner.src.bt.core import Node, to_dict, to_mdsl, to_mermaid
from planner.src.bt.generation import OpenAICompatibleChatClient
from planner.src.bt.runtime import make_policy
from planner.src.bt.visualization import render_viewer
from planner.src.tamp.planner import (
    ConstraintGrounder, GoalPredicate, SymbolicOperator, SymbolicVlmProposer,
    parse_goal_proposal, parse_proposal, plan_with_reprompting, refine_goals,
)
from planner.src.tamp.scenesmith import SceneSmithPickDomain
from planner.src.tamp.program import SketchGrounder
from planner.src.tamp.semantic import ModelSettings
from simulation.src.io.experiment import FactoryContext, load_experiment
from simulation.src.runtime.factory import make_env


SKILL_DESCRIPTIONS = {
    "NavigateToPick": "Move the mobile base to a collision-free, arm-reachable pick pose near the target.",
    "PickLift": "Pick the target with the left arm, lift it, and hold it stably.",
}
PREDICATES = {
    "at_pick_pose": "The mobile base is parked at an arm-reachable pose for this target.",
    "holding": "The robot has lifted this target and holds it stably.",
}
OPERATORS = (
    SymbolicOperator("NavigateToPick", frozenset({"observed"}),
                     frozenset({"at_pick_pose"})),
    SymbolicOperator("PickLift", frozenset({"observed", "at_pick_pose"}),
                     frozenset({"holding"})),
)


class RecordedProposer:
    def __init__(self, path: Path):
        self.content = path.read_text(encoding="utf-8")

    def propose(self, *, goal, world, feedback):
        del goal, feedback
        schema = json.loads(self.content).get("schema")
        if schema == "scenesmith.tamp.goals.v2":
            goals = parse_goal_proposal(self.content, frozenset(PREDICATES))
            known = frozenset(GoalPredicate("observed", name)
                              for name in world.get("objects", {}))
            return refine_goals(goals, OPERATORS, known)
        return parse_proposal(self.content, frozenset(SKILL_DESCRIPTIONS))


def _json(value):
    if dataclasses.is_dataclass(value):
        return {field.name: _json(getattr(value, field.name))
                for field in dataclasses.fields(value)}
    if isinstance(value, Mapping):
        return {str(key): _json(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json(item) for item in value]
    return value


def _write(path: Path, value):
    path.write_text(json.dumps(_json(value), ensure_ascii=False, indent=2,
                               default=str) + "\n", encoding="utf-8")


def _tree(steps) -> Node:
    actions = []
    for step in steps:
        if step.skill == "NavigateToPick":
            actions.append(Node("action", "NavigateTo",
                                tuple(step.checks["navigation_goal"])))
            actions.append(Node("condition", "BaseParkedAtPickPose"))
        elif step.skill == "PickLift":
            actions.append(Node("action", "ExecutePickLift"))
        else:
            raise ValueError(f"No BT binding for {step.skill}")
    return Node("root", children=(Node("selector", children=(
        Node("condition", "PickLiftSucceeded"),
        Node("sequence", children=tuple(actions)),
    )),))


def _candidate(value: str):
    parts = tuple(float(part) for part in value.split(","))
    if len(parts) != 3 or not all(math.isfinite(part) for part in parts):
        raise argparse.ArgumentTypeError("Expected base x,y,yaw_rad")
    return parts


def _joint_pick_plan(step):
    checks = step.checks.get("ik", {})
    waypoints = checks.get("lift_waypoints_world", {})
    if not waypoints.get("success"):
        return None
    plan = {
        "arm_joint_names": waypoints["arm_joint_names"],
        "staging_joint_positions": checks["staging_pose_in_target"]["arm_joint_positions"],
        "grasp_joint_positions": checks["grasp_pose_in_target"]["arm_joint_positions"],
        "lift_waypoints": [item["joint_positions"]
                           for item in waypoints["waypoints"]],
    }
    if "approach_waypoints_world" in checks:
        plan["approach_waypoints"] = [
            point["joint_positions"] for point in checks["approach_waypoints_world"]]
    return plan


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", required=True, type=Path)
    parser.add_argument("--repository-root", required=True, type=Path)
    parser.add_argument("--scene-root", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--goal", required=True)
    parser.add_argument("--base-candidate", action="append", required=True,
                        type=_candidate, dest="base_candidates")
    parser.add_argument("--seed", type=int, default=500)
    parser.add_argument("--model", help="Debug override for both configured legacy model stages")
    parser.add_argument("--config", type=Path, help="TAMP model configuration")
    parser.add_argument("--base-url", default=os.environ.get("OPENAI_BASE_URL"))
    parser.add_argument("--api-key-env", default="OPENAI_API_KEY")
    parser.add_argument("--recorded-proposal", type=Path)
    parser.add_argument("--recorded-program", type=Path)
    parser.add_argument("--max-samples", type=int, default=16)
    parser.add_argument("--max-proposals", type=int, default=3)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--max-steps", type=int)
    args = parser.parse_args(argv)
    if not args.recorded_proposal and (not args.base_url or not os.environ.get(args.api_key_env)):
        parser.error("Live planning requires OPENAI_BASE_URL and the API key environment variable")
    if args.recorded_proposal and args.max_proposals != 3:
        parser.error("Recorded proposals use the default attempt limit")
    output = args.output_root.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    repo = args.repository_root.resolve()
    config_path = args.config or repo / "experiments/tamp_hierarchical_config.json"
    settings = json.loads(config_path.read_text(encoding="utf-8"))
    subgoal_settings = ModelSettings(**settings["subgoal_model"])
    skill_settings = ModelSettings(**settings["skill_model"])
    if args.model:
        subgoal_settings = dataclasses.replace(subgoal_settings, model=args.model)
        skill_settings = dataclasses.replace(skill_settings, model=args.model)
    experiment = load_experiment(args.experiment, repository_root=repo,
                                 cache_root=output / "scene_cache", scene_root=args.scene_root,
                                 trust_factories=True)
    observation, reset_info = make_env(experiment.environment_config).reset(args.seed)
    image_dir = output / "observations"
    image_dir.mkdir()
    images = []
    for name in ("head_camera", "left_wrist_camera"):
        camera = observation.sensors.get(name)
        if camera is None or camera.rgb is None:
            continue
        path = image_dir / f"{name}_rgb.png"
        Image.fromarray(camera.rgb).save(path)
        images.append({"camera": name, "path": str(path),
                       "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    world = {"base": observation.base,
             "objects": {name: dataclasses.asdict(item.pose)
                         for name, item in observation.objects.items()},
             "task": dict(observation.task), "reset": reset_info,
             "images": [{key: item[key] for key in ("camera", "sha256")}
                        for item in images]}
    _write(output / "world_snapshot.json", world)
    if args.recorded_proposal:
        proposer = RecordedProposer(args.recorded_proposal)
        proposals = 1
    else:
        client = OpenAICompatibleChatClient(
            base_url=args.base_url, api_key=os.environ[args.api_key_env])
        proposer = SymbolicVlmProposer(client=client, **dataclasses.asdict(subgoal_settings),
                                      skill_descriptions=SKILL_DESCRIPTIONS,
                                      predicates=PREDICATES, operators=OPERATORS,
                                      images=tuple(images))
        proposals = args.max_proposals
    domain = SceneSmithPickDomain(
        environment_config=experiment.environment_config,
        observation=observation,
        calibration_path=repo / "experiments/inputs/pick_lift/pick_lift_calibration.json",
        pick_home_path=repo / "experiments/inputs/pick_lift/pick_home.json",
        base_candidates=tuple(args.base_candidates),
        lift_distance_m=experiment.resolved_config["user_config"]["policy_options"].get(
            "expert_policy_overrides", {}).get("lift_distance_m"),
    )
    rollouts = []

    def rollout(steps):
        candidate_dir = output / "rollouts" / f"sample_{len(rollouts) + 1:03d}"
        candidate_dir.mkdir(parents=True)
        tree_data = to_dict(_tree(steps))
        bt_path = candidate_dir / "bt.json"
        _write(bt_path, tree_data)
        (candidate_dir / "bt.html").write_text(
            render_viewer(tree_data), encoding="utf-8")
        try:
            options = dict(experiment.resolved_config["user_config"]["policy_options"])
            options["bt_json_input"] = str(bt_path)
            pick_steps = [step for step in steps if step.skill == "PickLift"]
            if len(pick_steps) == 1:
                options["expert_grasp_lateral_offset_m"] = pick_steps[0].parameters.get(
                    "grasp_lateral_offset_m", 0.0)
                joint_plan = _joint_pick_plan(pick_steps[0])
                if joint_plan is not None:
                    options["tamp_joint_skill_plan"] = joint_plan
            options["tamp_generation"] = {
                "mode": "tamp",
                "model_called": args.recorded_proposal is None,
                "model": None if args.recorded_proposal else subgoal_settings.model,
                "candidate": str(candidate_dir),
            }
            candidate_experiment = json.loads(args.experiment.read_text(
                encoding="utf-8"))
            candidate_experiment["policy_options"] = options
            _write(candidate_dir / "experiment.json", candidate_experiment)
            policy = make_policy(FactoryContext(
                experiment.environment_config,
                experiment.environment_config.robot_adapter.spec,
                options, repo))
            run_options = dict(experiment.run_options)
            run_options["seeds"] = (args.seed,)
            if args.max_steps is not None:
                run_options["max_steps"] = args.max_steps
            simulation = dataclasses.replace(experiment, policy=policy,
                                             run_options=run_options)
            episodes = simulation.run(candidate_dir / "simulation")
            summary = episodes[0].summary
            accepted = bool(summary["success"])
            reason = "physical_success" if accepted else summary["termination_reason"]
            details = {"summary_json": episodes[0].artifact_paths.get("summary_json"),
                       "task": summary.get("task", {})}
        except Exception as error:
            accepted, reason = False, "simulation_exception"
            details = {"type": type(error).__name__, "message": str(error)}
        rollouts.append({"candidate": str(candidate_dir), "success": accepted,
                         "reason": reason, "details": details})
        return accepted, reason, details

    if args.recorded_program:
        program_content = args.recorded_program.read_text(encoding="utf-8")
        grounder = SketchGrounder(
            domain, world=world, recorded_program=program_content,
            max_samples=args.max_samples, seed=args.seed,
            rollout=rollout if args.execute else None)
    elif args.recorded_proposal:
        grounder = ConstraintGrounder(domain, rollout=rollout if args.execute else None)
    else:
        grounder = SketchGrounder(
            domain, world=world, client=client, **dataclasses.asdict(skill_settings),
            images=tuple(images), max_samples=args.max_samples, seed=args.seed,
            rollout=rollout if args.execute else None)
    result = plan_with_reprompting(
        proposer=proposer, grounder=grounder,
        goal=args.goal, world=world,
        initial_state={"base_height_m": experiment.environment_config.robot_adapter.base_config.base_height_m},
        max_proposals=proposals,
    )
    _write(output / "tamp_plan.json", {"schema": "scenesmith.tamp.plan.v1",
                                       "goal": args.goal,
                                       "model": None if args.recorded_proposal else subgoal_settings.model,
                                       "model_settings": {"subgoal_model": dataclasses.asdict(subgoal_settings),
                                                          "skill_model": dataclasses.asdict(skill_settings)},
                                       "model_attempts": getattr(proposer, "model_attempts", []),
                                       "source": "recorded" if args.recorded_proposal else "vlm",
                                       "result": result, "rollouts": rollouts,
                                       "programs": getattr(grounder, "programs", [])})
    if not result.success:
        print(f"No feasible grounding; see {output / 'tamp_plan.json'}")
        return 2
    tree = _tree(result.steps)
    tree_data = to_dict(tree)
    bt = output / "bt"
    bt.mkdir()
    _write(bt / "generated_bt.json", tree_data)
    _write(bt / "runtime_bindings.json", {
        "pick_lift": [step.parameters for step in result.steps
                      if step.skill == "PickLift"],
        "pick_lift_joint_skill_plans": [plan for step in result.steps
                                        if step.skill == "PickLift"
                                        if (plan := _joint_pick_plan(step)) is not None],
        "navigation": [step.parameters for step in result.steps
                       if step.skill == "NavigateToPick"],
    })
    (bt / "generated_bt.mdsl").write_text(to_mdsl(tree), encoding="utf-8")
    (bt / "generated_bt.mmd").write_text(to_mermaid(tree), encoding="utf-8")
    (bt / "generated_bt.html").write_text(render_viewer(tree_data), encoding="utf-8")
    if args.execute:
        print(json.dumps({"physical_success": True,
                          "rollout": rollouts[-1]["candidate"]}))
    print(f"TAMP artifacts: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
