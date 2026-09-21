"""Bounded live planning audit using unchanged registered SceneSmith skills.

Only this harness and new run artifacts are added. Symbolic counterfactuals are
explicitly separate from physical execution; no injected facts enter execution.
"""

import argparse
import dataclasses
import hashlib
import json
import os
from pathlib import Path
import random
import time

from planner.src.bt.generation import OpenAICompatibleChatClient
from planner.src.tamp.ccsp import Proc3sCCSPSolver
from planner.src.tamp.failures import ProgramFailure
from planner.src.tamp.hierarchy import PredicateGoal, picklift_registry, program_identity
from planner.src.tamp.online import IncrementalTampRunner, JsonlTrace, RecoveryLimits
from planner.src.tamp.proc3s import PRoC3SProgramGenerator, PRoC3SGenerationFailure
from planner.src.tamp.scenesmith import SceneSmithPickDomain
from planner.src.tamp.scenesmith_online import SceneSmithSkillExecutor, SceneSmithWorldObserver
from planner.src.tamp.semantic import SemanticSubgoalPlanner, SemanticModelError, ModelSettings
from simulation.src import make_env
from simulation.src.io.experiment import load_experiment


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, default=str) + "\n")


def source_hashes(repo):
    """Protect all existing implementations, prompts, configs and robot models."""
    result = {}
    for folder in ("simulation", "planner", "experiments", "models"):
        for path in sorted((repo / folder).rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts:
                result[str(path.relative_to(repo))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


class RecordedClient:
    """Record original model text without recording credentials or HTTP headers."""

    def __init__(self, client, trace):
        self.client, self.trace = client, trace
        self.case = "initialization"

    def complete(self, **kwargs):
        self.trace({"event": "raw_model_request", "case": self.case, **kwargs})
        start = time.perf_counter()
        response = self.client.complete(**kwargs)
        self.trace({"event": "raw_model_response", "case": self.case,
                    "elapsed_s": time.perf_counter() - start,
                    "response": dataclasses.asdict(response)})
        return response


def counterfactuals(client, settings, registry, world, images, output, target, repeats):
    """Test reasoning over explicit facts, not visual reachability estimation."""
    results = []
    at_pick = PredicateGoal("at_pick_pose", (target,))
    holding = PredicateGoal("holding", (target,))
    ready = dataclasses.replace(world, facts=world.facts | {at_pick})
    done = dataclasses.replace(ready, facts=(ready.facts - {PredicateGoal("gripper_empty", ())}) | {holding})
    cases = (
        ("observed_not_at_pick", world, (holding,), ["NavigateToPick", "PickLift"], False),
        ("injected_at_pick", ready, (holding,), ["PickLift"], True),
        ("navigation_goal_only", world, (at_pick,), ["NavigateToPick"], False),
        ("injected_already_holding", done, (holding,), [], True),
    )
    trace = JsonlTrace(output / "reasoning_trace.jsonl")
    for repetition in range(repeats):
        ready_program = None
        ready_generator = None
        for name, state, goals, expected, injected in cases:
            client.case = f"symbolic/{name}/{repetition}"
            generator = PRoC3SProgramGenerator(client, ModelSettings(**settings["skill_model"]), registry, trace=trace)
            item = {"case": name, "repetition": repetition, "injected_symbolic_facts": injected,
                    "physical_execution": False, "input_facts": [dataclasses.asdict(f) for f in sorted(state.facts)],
                    "goals": [dataclasses.asdict(f) for f in goals], "expected_skills": expected}
            try:
                program = generator.generate(state, goals)
                skills = [step.skill for step in program.steps]
                item.update(program=dataclasses.asdict(program), actual_skills=skills, passed=skills == expected)
                if name == "injected_at_pick":
                    ready_program = program
                    ready_generator = generator
            except PRoC3SGenerationFailure as error:
                item.update(passed=False, error=str(error))
            item["model_calls"] = generator.calls
            results.append(item)
            save(output / "reasoning_results.json", results)
            print(json.dumps({"phase": "symbolic", "case": name, "repetition": repetition,
                              "passed": item["passed"], "skills": item.get("actual_skills")}), flush=True)
        if ready_program is not None:
            client.case = f"symbolic/injected_grasp_failure/{repetition}"
            generator = ready_generator
            calls_before = generator.calls
            feedback = ProgramFailure("PickLift", ("ik",), (target,), True, True).as_feedback()
            item = {"case": "injected_grasp_failure", "repetition": repetition,
                    "injected_symbolic_facts": True, "injected_failure": True, "physical_execution": False,
                    "feedback": feedback, "before": dataclasses.asdict(ready_program)}
            try:
                program = generator.generate(ready, (holding,), feedback=(feedback,),
                                             excluded_programs=frozenset({program_identity(ready_program)}))
                skills = [step.skill for step in program.steps]
                item.update(program=dataclasses.asdict(program), actual_skills=skills,
                            passed=skills == ["NavigateToPick", "PickLift"])
            except PRoC3SGenerationFailure as error:
                item.update(passed=False, error=str(error))
            item["model_calls"] = generator.calls - calls_before
            results.append(item)
            save(output / "reasoning_results.json", results)
            print(json.dumps({"phase": "symbolic_repair", "repetition": repetition, "passed": item["passed"]}), flush=True)
    visual_results = []
    for repetition in range(repeats):
        for name, task, expected in (
            ("pick", "Pick up the red object and hold it.", holding),
            ("navigate_only", "Move to a suitable picking position for the red object, but do not grasp or lift it.", at_pick),
        ):
            client.case = f"visual/{name}/{repetition}"
            semantic = SemanticSubgoalPlanner(client, ModelSettings(**settings["subgoal_model"]), trace=trace)
            item = {"case": name, "repetition": repetition, "task": task,
                    "image_count": len(images), "simulator_labels_provided": True}
            try:
                goals = semantic.propose(task=task,
                    world={"objects": world.objects, "facts": [dataclasses.asdict(f) for f in sorted(world.facts)]},
                    predicate_arity={"observed": 1, "at_pick_pose": 1, "holding": 1, "gripper_empty": 0}, images=images)
                item.update(goals=[dataclasses.asdict(f) for f in goals],
                            passed=expected in goals and (name != "navigate_only" or holding not in goals))
            except SemanticModelError as error:
                item.update(passed=False, error=str(error))
            item["model_calls"] = semantic.calls
            visual_results.append(item)
            save(output / "visual_results.json", visual_results)
            print(json.dumps({"phase": "visual", "case": name, "repetition": repetition, "passed": item["passed"]}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--scene-root", type=Path, required=True)
    parser.add_argument("--experiment", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=500)
    parser.add_argument("--max-samples", type=int, default=16)
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()
    repo, output = args.repository_root.resolve(), args.output_root.resolve()
    output.mkdir(parents=True, exist_ok=False)
    before = source_hashes(repo)
    save(output / "protected_sources_before.json", before)
    settings = json.loads((repo / "experiments/tamp_hierarchical_config.json").read_text())
    settings["proc3s_ccsp"] = {"max_samples": args.max_samples}
    settings["recovery"].update(semantic_replans=0, skill_replans=1, geometry_retries=1, max_skill_executions=6)
    save(output / "audit_config.json", {"arguments": vars(args), "settings": settings,
          "scope": "bounded_validation_no_skill_changes", "counterfactuals_are_not_physical_trials": True})
    experiment = load_experiment(args.experiment, repository_root=repo,
        cache_root=output / "scene_cache", scene_root=args.scene_root, trust_factories=True, meshcat=True)
    save(output / "resolved_experiment.json", experiment.resolved_config)
    env = make_env(experiment.environment_config)
    observation, reset_info = env.reset(seed=args.seed)
    task = experiment.environment_config.task_factory()
    target = getattr(task, "task", task).config.target_observation_name
    registry = picklift_registry()
    save(output / "skill_registry.json", [dataclasses.asdict(s) for s in registry])
    runtime_output = output / "runtime"
    runtime_output.mkdir()
    executor = SceneSmithSkillExecutor(env=env, experiment=experiment, repository_root=repo,
        output_root=runtime_output, observation=observation, reset_info=reset_info,
        max_skill_steps=settings["max_skill_steps"], model_called=True, registry=registry)
    observer = SceneSmithWorldObserver(executor, target, runtime_output,
        scene_metadata=Path(experiment.resolved_config["provenance"]["scene_manifest"]).with_name("scene_metadata.json"))
    world = observer.observe(observation)
    images = observer.capture_images(observation)
    save(output / "initial_world.json", dataclasses.asdict(world))
    client = RecordedClient(OpenAICompatibleChatClient(base_url=os.environ["OPENAI_BASE_URL"],
        api_key=os.environ["OPENAI_API_KEY"]), JsonlTrace(output / "raw_model_io.jsonl"))
    try:
        counterfactuals(client, settings, registry, world, images, output, target, args.repeats)
        trace_file = JsonlTrace(runtime_output / "tamp_trace.jsonl")

        def trace(event):
            trace_file(event)
            if event["event"] in {"skill_skeleton", "semantic_subgoal", "ccsp_solved", "recovery_action", "execution_outcome", "online_result"}:
                print(json.dumps({"phase": "live", **event}, default=str), flush=True)

        rng = random.Random(args.seed)

        def solver_factory(current_world):
            del current_world
            overrides = experiment.resolved_config["user_config"]["policy_options"].get("expert_policy_overrides", {})
            domain = SceneSmithPickDomain(environment_config=experiment.environment_config,
                observation=executor.observation,
                calibration_path=repo / "experiments/inputs/pick_lift/pick_lift_calibration.json",
                pick_home_path=repo / "experiments/inputs/pick_lift/pick_home.json",
                open_width_m=overrides.get("open_width_m"), lift_distance_m=overrides.get("lift_distance_m"))
            return Proc3sCCSPSolver(registry, domain, trace=trace, rng=rng, max_samples=args.max_samples)

        client.case = "live_end_to_end"
        runner = IncrementalTampRunner(
            semantic=SemanticSubgoalPlanner(client, ModelSettings(**settings["subgoal_model"]), trace=trace),
            registry=registry, solver_factory=solver_factory, executor=executor, observer=observer,
            trace=trace, limits=RecoveryLimits(**settings["recovery"]),
            program_generator=PRoC3SProgramGenerator(client, ModelSettings(**settings["skill_model"]), registry, trace=trace))
        env.start_recording()
        start = time.perf_counter()
        try:
            result = runner.run(task="Pick up the red object and hold it.",
                task_goals=(PredicateGoal("holding", (target,)),), initial_observation=observation,
                initial_geometry_state=observer.geometry_state(observation, {
                    "base_height_m": experiment.environment_config.robot_adapter.base_config.base_height_m}),
                predicate_arity={"observed": 1, "at_pick_pose": 1, "holding": 1, "gripper_empty": 0}, images=images)
            save(output / "live_result.json", {"success": result.success, "reason": result.reason,
                "metrics": dict(result.metrics), "final_task": dict(executor.observation.task),
                "wall_time_s": time.perf_counter() - start, "seed": args.seed,
                "final_facts": [dataclasses.asdict(f) for f in sorted(result.world.facts)],
                "recording": str(output / "simulation.html")})
            print(json.dumps({"phase": "complete", "success": result.success, "reason": result.reason}), flush=True)
        finally:
            env.save_recording(output / "simulation.html")
    finally:
        after = source_hashes(repo)
        changed = sorted(k for k in set(before) | set(after) if before.get(k) != after.get(k))
        save(output / "protected_sources_after.json", after)
        save(output / "source_integrity.json", {"checked_files": len(before), "changed": changed, "unchanged": not changed})
        if changed:
            raise RuntimeError(f"Protected source files changed during audit: {changed}")


if __name__ == "__main__":
    main()
