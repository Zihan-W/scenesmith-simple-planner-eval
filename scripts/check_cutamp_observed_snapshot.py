"""Export or postcheck a real recorded observation without replaying physics.

This offline harness does not restore the simulator or claim physical success.
It uses only recorded poses/joints, the actual scene query and unchanged shared
skill checks. Optimizer execution is a separate explicit CUDA worker command.
"""

import argparse
import json
from pathlib import Path
from types import SimpleNamespace

from cutamp_observed_resolve import postcheck_candidates
from planner.src.tamp.cutamp_problem import ContinuousProblemBuilder
from planner.src.tamp.hierarchy import SkillProgram, SkillStep, WorldState
from planner.src.tamp.scenesmith import SceneSmithPickDomain
from simulation.src import Pose
from simulation.src.io.experiment import load_experiment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("export", "postcheck"))
    for name in ("repository-root", "scene-root", "experiment", "outcomes", "output-root"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--kinematics-template", type=Path)
    parser.add_argument("--robot-template", type=Path)
    parser.add_argument("--candidates", type=Path)
    parser.add_argument("--outcome-index", type=int, default=-1,
                        help="Index among outcomes containing a world observation; default is the latest")
    args = parser.parse_args()
    source = json.loads(args.outcomes.read_text())
    recorded = [item["world"] for item in source if "world" in item][args.outcome_index]
    world = WorldState(recorded["objects"], frozenset(), recorded["observation_id"], recorded["robot"])
    observation = SimpleNamespace(
        time_s=float(world.observation_id), base={"base_link_pose": world.robot["base_link_pose"]},
        robot=SimpleNamespace(joint_names=world.robot["joint_names"], q=world.robot["q"]),
        objects={name: SimpleNamespace(pose=Pose(tuple(item["translation_m"]),
                                                tuple(item["quaternion_wxyz"])))
                 for name, item in world.objects.items()})
    args.output_root.mkdir(parents=True, exist_ok=False)
    experiment = load_experiment(args.experiment, repository_root=args.repository_root,
                                 cache_root=args.output_root / "scene_cache", scene_root=args.scene_root,
                                 trust_factories=True)
    options = experiment.resolved_config["user_config"]["policy_options"]
    domain = SceneSmithPickDomain(
        environment_config=experiment.environment_config, observation=observation,
        calibration_path=args.repository_root / options["expert_calibration"],
        pick_home_path=args.repository_root / "experiments/inputs/pick_lift/pick_home.json",
        open_width_m=options.get("expert_policy_overrides", {}).get("open_width_m"),
        lift_distance_m=options.get("expert_policy_overrides", {}).get("lift_distance_m"))
    if args.mode == "export":
        if args.kinematics_template is None or args.robot_template is None:
            parser.error("export requires both templates")
        builder = ContinuousProblemBuilder(domain, experiment.resolved_config,
                                            args.kinematics_template, args.robot_template)
        program = SkillProgram((SkillStep("PickLift", {"object": domain.target_name},
                                         {"grasp_pose": "g0", "approach_pose": "a0"}),))
        result = builder.build(program, world, args.output_root / "problem")
    else:
        if args.candidates is None:
            parser.error("postcheck requires --candidates")
        candidates = json.loads(args.candidates.read_text())["candidates"]
        assignment, checks = postcheck_candidates(domain, world, candidates, args.output_root)
        result = {"feasible": assignment is not None, "checks": checks}
    result.update(scope="recorded_observation_offline_probe_not_physical_replay",
                  observation_id=world.observation_id, source=str(args.outcomes.resolve()),
                  physical_success=False)
    (args.output_root / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "checks"}))


if __name__ == "__main__":
    main()
