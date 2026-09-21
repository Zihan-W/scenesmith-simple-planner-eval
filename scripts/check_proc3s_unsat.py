"""Live LLM + real SceneSmith checks with an explicit all-grasps fault injection.

Acceptance concerns unsat feedback and skeleton revision, not task completion.
The wrapper always runs the real geometry check and then forbids every PickLift.
No physical rollout is executed; original constraint results remain in the trace.
"""

import argparse
import dataclasses
import json
import os
from pathlib import Path

from planner.src.bt.generation import OpenAICompatibleChatClient
from planner.src.tamp.ccsp import Proc3sCCSPSolver, PRoC3SProgramUnsat
from planner.src.tamp.hierarchy import (
    PredicateGoal, WorldState, picklift_registry, program_identity,
)
from planner.src.tamp.online import JsonlTrace
from planner.src.tamp.proc3s import PRoC3SProgramGenerator
from planner.src.tamp.scenesmith import SceneSmithPickDomain
from planner.src.tamp.semantic import ModelSettings
from simulation.src import Pose, make_env
from simulation.src.io.experiment import load_experiment


class AllGraspsForbidden:
    """Test-only constraint, never installed into a production solver domain."""

    def __init__(self, domain, trace):
        self.domain, self.trace = domain, trace
        self.pick_checks = 0

    def sample_candidate(self, *args):
        return self.domain.sample_candidate(*args)

    def parameter_control_keys(self, parameter):
        return self.domain.parameter_control_keys(parameter)

    def predict(self, *args):
        return self.domain.predict(*args)

    def check(self, skill, candidate, state):
        feasible, constraint, details = self.domain.check(skill, candidate, state)
        if skill.skill != "PickLift":
            return feasible, constraint, details
        self.pick_checks += 1
        self.trace({"event": "injected_grasp_failure", "original_feasible": feasible,
                    "original_constraint": constraint, "original_details": details})
        return False, "grasp_validity", {
            "fault_injection": "all_grasps_forbidden", "target": self.domain.target_name,
            "original_constraint": constraint, "original_details": details,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--scene-root", type=Path, required=True)
    parser.add_argument("--experiment", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=500)
    parser.add_argument("--max-samples", type=int, default=3)
    args = parser.parse_args()
    output, repo = args.output_root.resolve(), args.repository_root.resolve()
    output.mkdir(parents=True, exist_ok=False)
    trace = JsonlTrace(output / "trace.jsonl")
    experiment = load_experiment(args.experiment, repository_root=repo,
                                 cache_root=output / "scene_cache", scene_root=args.scene_root,
                                 trust_factories=True)
    env = make_env(experiment.environment_config)
    observation, _ = env.reset(seed=args.seed)
    config = json.loads((repo / "experiments/tamp_hierarchical_config.json").read_text())
    domain = SceneSmithPickDomain(
        environment_config=experiment.environment_config, observation=observation,
        calibration_path=repo / "experiments/inputs/pick_lift/pick_lift_calibration.json",
        pick_home_path=repo / "experiments/inputs/pick_lift/pick_home.json",
    )
    fault_domain = AllGraspsForbidden(domain, trace)
    registry = picklift_registry()
    world = WorldState(
        {name: dataclasses.asdict(item.pose) for name, item in observation.objects.items()},
        frozenset({PredicateGoal("observed", (name,)) for name in observation.objects}
                  | {PredicateGoal("gripper_empty", ())}),
        observation_id=f"{observation.time_s:.6f}",
    )
    client = OpenAICompatibleChatClient(base_url=os.environ["OPENAI_BASE_URL"],
                                        api_key=os.environ["OPENAI_API_KEY"])
    generator = PRoC3SProgramGenerator(client, ModelSettings(**config["skill_model"]),
                                      registry, trace=trace)
    goal = (PredicateGoal("holding", (domain.target_name,)),)
    first = generator.generate(world, goal)
    pose = observation.base["base_link_pose"]
    state = {"base_height_m": experiment.environment_config.robot_adapter.base_config.base_height_m,
             "base_pose": Pose(tuple(pose["translation_m"]), tuple(pose["quaternion_wxyz"]))}
    (output / "configuration.json").write_text(json.dumps({
        "seed": args.seed, "max_samples": args.max_samples, "fault": "all_grasps_forbidden",
        "model_settings": config["skill_model"], "resolved_experiment": experiment.resolved_config,
        "physical_rollout_executed": False,
    }, indent=2, default=str) + "\n")
    try:
        Proc3sCCSPSolver(registry, fault_domain, max_samples=args.max_samples,
                         seed=args.seed, trace=trace).solve(world, first, state)
    except PRoC3SProgramUnsat as failure:
        feedback = failure.program_feedback
        trace({"event": "program_unsat_feedback", **feedback})
    else:
        raise AssertionError("The deliberately impossible grasp constraint was ignored")
    if not fault_domain.pick_checks:
        raise AssertionError("No grasp was checked; this is not the intended acceptance case")
    revised = generator.generate(world, goal, feedback=(feedback,),
                                  excluded_programs=frozenset({program_identity(first)}))
    changed = program_identity(first) != program_identity(revised)
    result = {"acceptance": "live_scenesmith_fault_injected_unsat_revision", "success": changed,
              "fault_injection": "all_grasps_forbidden", "physical_rollout_executed": False,
              "llm_calls": generator.calls, "real_pick_checks": fault_domain.pick_checks,
              "feedback": feedback, "first_program": dataclasses.asdict(first),
              "revised_program": dataclasses.asdict(revised)}
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"success": changed, "llm_calls": generator.calls,
                      "real_pick_checks": fault_domain.pick_checks, "output_root": str(output)}))
    if not changed:
        raise AssertionError("The model did not change the failed skeleton")


if __name__ == "__main__":
    main()
