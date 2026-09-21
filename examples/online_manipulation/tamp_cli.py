"""Unified BT baseline and hierarchical/legacy TAMP command entrypoint."""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
from pathlib import Path

from examples.online_manipulation.bt_generation import OpenAICompatibleChatClient
from examples.online_manipulation.tamp_geometry import SamplingSolver
from examples.online_manipulation.tamp_hierarchy import (
    PredicateGoal, parse_semantic_goals, picklift_registry,
)
from examples.online_manipulation.tamp_online import (
    IncrementalTampRunner, JsonlTrace, RecoveryLimits,
)
from examples.online_manipulation.tamp_scenesmith import SceneSmithPickDomain
from examples.online_manipulation.tamp_scenesmith_online import (
    SceneSmithSkillExecutor, SceneSmithWorldObserver,
)
from examples.online_manipulation.tamp_semantic import ModelSettings, SemanticSubgoalPlanner
from src.online_manipulation.experiment import load_experiment
from src.online_manipulation.factory import make_env


class RecordedSemantic:
    """Deterministic v3 test input; never counted as an online VLM request."""

    calls = 0

    def __init__(self, content: str):
        self.content = content

    def propose(self, *, task, world, predicate_arity, feedback, images):
        del task, feedback, images
        return parse_semantic_goals(
            self.content, predicate_arity=predicate_arity,
            known_objects=frozenset(world["objects"]),
        )


def _execution_options(run_options, seed):
    """Apply an explicit common CLI seed without changing baseline defaults."""
    options = dict(run_options)
    if seed is not None:
        options["seeds"] = (seed,)
    return options


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner", choices=("bt", "tamp"), default="tamp")
    parser.add_argument("--tamp-mode", choices=("hierarchical", "legacy-vlm-domain"),
                        default="hierarchical")
    parser.add_argument("--geometry-backend", choices=("sampling", "proc3s", "cutamp"),
                        default="sampling")
    parser.add_argument("--skill-planner", choices=("strips", "proc3s"), default="strips")
    parser.add_argument("--request", type=Path, help="BT baseline generation request")
    parser.add_argument("--model-response", type=Path,
                        help="Recorded BT response for deterministic baseline tests")
    parser.add_argument("--execute", action="store_true",
                        help="Execute the generated BT baseline in SceneSmith")
    parser.add_argument("--record-html", action="store_true",
                        help="Retain a standalone Meshcat replay of actual simulation")
    parser.add_argument("--experiment", type=Path)
    parser.add_argument("--repository-root", type=Path)
    parser.add_argument("--scene-root", type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--task")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--seed", type=int,
                        help="Shared explicit seed; BT defaults to experiment seeds, TAMP to 500")
    parser.add_argument("--recorded-subgoals", type=Path)
    parser.add_argument("--recorded-proposal", type=Path)
    parser.add_argument("--recorded-program", type=Path)
    parser.add_argument("--base-candidate", action="append", help="Legacy/debug only")
    parser.add_argument("--base-url", default=os.environ.get("OPENAI_BASE_URL"))
    parser.add_argument("--api-key-env", default="OPENAI_API_KEY")
    args = parser.parse_args(argv)

    if args.planner == "bt":
        if args.request is None:
            parser.error("BT baseline requires --request")
        from examples.online_manipulation.bt_generation import main as bt_main
        command = ["--request", str(args.request), "--output-dir", str(args.output_root)]
        if args.base_url:
            command += ["--base-url", args.base_url]
        command += ["--api-key-env", args.api_key_env]
        if args.model_response:
            command += ["--model-response", str(args.model_response)]
        bt_main(command)
        if not args.execute:
            return 0
        if not all((args.experiment, args.repository_root, args.scene_root)):
            parser.error("Executing BT requires --experiment, --repository-root and --scene-root")
        from examples.online_manipulation.bt_runtime import make_policy
        from src.online_manipulation.experiment import FactoryContext
        output = args.output_root.resolve()
        repo = args.repository_root.resolve()
        experiment = load_experiment(
            args.experiment, repository_root=repo,
            cache_root=output / "scene_cache", scene_root=args.scene_root,
            trust_factories=True,
            meshcat=args.record_html,
        )
        options = dict(experiment.resolved_config["user_config"]["policy_options"])
        options["bt_json_input"] = str(output / "generated_bt.json")
        policy = make_policy(FactoryContext(
            experiment.environment_config,
            experiment.environment_config.robot_adapter.spec,
            options, repo,
        ))
        run_options = _execution_options(experiment.run_options, args.seed)
        if args.record_html:
            run_options["record_html"] = True
        episodes = dataclasses.replace(experiment, policy=policy,
                                       run_options=run_options).run(output / "simulation")
        summary = episodes[0].summary
        (output / "result.json").write_text(json.dumps({
            "planner": "bt", "success": bool(summary["success"]),
            "seeds": list(run_options["seeds"]),
            "reason": summary["termination_reason"],
            "summary_json": episodes[0].artifact_paths.get("summary_json"),
        }, indent=2) + "\n")
        return 0 if summary["success"] else 2

    if not all((args.experiment, args.repository_root, args.scene_root, args.task)):
        parser.error("TAMP requires --experiment, --repository-root, --scene-root and --task")
    if args.tamp_mode == "legacy-vlm-domain":
        if args.record_html:
            parser.error("--record-html currently requires BT or hierarchical TAMP mode")
        if not args.base_candidate:
            parser.error("Legacy model-domain ablation requires --base-candidate")
        from examples.online_manipulation.tamp_pipeline import main as legacy_main
        command = ["--experiment", str(args.experiment),
                   "--repository-root", str(args.repository_root),
                   "--scene-root", str(args.scene_root),
                   "--output-root", str(args.output_root), "--goal", args.task,
                   "--execute"]
        for candidate in args.base_candidate:
            command += ["--base-candidate", candidate]
        if args.recorded_proposal:
            command += ["--recorded-proposal", str(args.recorded_proposal)]
        if args.recorded_program:
            command += ["--recorded-program", str(args.recorded_program)]
        if args.base_url:
            command += ["--base-url", args.base_url]
        command += ["--api-key-env", args.api_key_env]
        if args.config:
            command += ["--config", str(args.config)]
        if args.seed is not None:
            command += ["--seed", str(args.seed)]
        return legacy_main(command)

    if args.geometry_backend == "cutamp":
        parser.error("The fixed-skeleton cuTAMP adapter for Zerith is not implemented; "
                     "use --geometry-backend sampling or proc3s")
    output = args.output_root.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    repo = args.repository_root.resolve()
    config_path = args.config or repo / "experiments/tamp_hierarchical_config.json"
    settings = json.loads(config_path.read_text(encoding="utf-8"))
    (output / "planner_config.json").write_text(json.dumps({
        "settings": settings, "task": args.task, "seed": 500 if args.seed is None else args.seed,
        "skill_planner": args.skill_planner, "geometry_backend": args.geometry_backend,
        "record_html": args.record_html,
    }, indent=2, ensure_ascii=False) + "\n")
    experiment = load_experiment(
        args.experiment, repository_root=repo,
        cache_root=output / "scene_cache", scene_root=args.scene_root,
        trust_factories=True,
        meshcat=args.record_html,
    )
    trace = JsonlTrace(output / "tamp_trace.jsonl")
    (output / "resolved_experiment.json").write_text(
        json.dumps(experiment.resolved_config, indent=2, default=str) + "\n")
    client = None
    if not args.recorded_subgoals or args.skill_planner == "proc3s":
        key = os.environ.get(args.api_key_env)
        if not args.base_url or not key:
            parser.error("Live hierarchical TAMP requires OPENAI_BASE_URL and API key")
        client = OpenAICompatibleChatClient(base_url=args.base_url, api_key=key)
    if args.recorded_subgoals:
        semantic = RecordedSemantic(args.recorded_subgoals.read_text(encoding="utf-8"))
    else:
        semantic = SemanticSubgoalPlanner(client, ModelSettings(**settings["subgoal_model"]),
                                         trace=trace)
    env = make_env(experiment.environment_config)
    seed = 500 if args.seed is None else args.seed
    observation, reset_info = env.reset(seed=seed)
    task = experiment.environment_config.task_factory()
    task = getattr(task, "task", task)
    target = task.config.target_observation_name
    registry = picklift_registry()
    from examples.online_manipulation.tamp_skill_planning import StripsProgramGenerator
    if args.skill_planner == "proc3s":
        from examples.online_manipulation.tamp_proc3s import PRoC3SProgramGenerator
        program_generator = PRoC3SProgramGenerator(
            client, ModelSettings(**settings["skill_model"]), registry, trace=trace)
    else:
        program_generator = StripsProgramGenerator(registry)
    executor = SceneSmithSkillExecutor(
        env=env, experiment=experiment, repository_root=repo,
        output_root=output, observation=observation, reset_info=reset_info,
        max_skill_steps=int(settings["max_skill_steps"]),
        model_called=args.recorded_subgoals is None,
        registry=registry,
    )
    scene_manifest = experiment.resolved_config["provenance"].get("scene_manifest")
    observer = SceneSmithWorldObserver(
        executor, target, output,
        scene_metadata=Path(scene_manifest).with_name("scene_metadata.json")
        if scene_manifest else None,
    )
    images = observer.capture_images(observation)
    debug_candidates = ()
    if args.base_candidate:
        debug_candidates = tuple(tuple(float(part) for part in value.split(","))
                                 for value in args.base_candidate)

    import random
    ccsp_rng = random.Random(seed)

    def solver_factory(world):
        del world
        domain = SceneSmithPickDomain(
            environment_config=experiment.environment_config,
            observation=executor.observation,
            calibration_path=repo / "experiments/inputs/pick_lift/pick_lift_calibration.json",
            pick_home_path=repo / "experiments/inputs/pick_lift/pick_home.json",
            base_candidates=debug_candidates,
            open_width_m=experiment.resolved_config["user_config"]["policy_options"].get(
                "expert_policy_overrides", {}).get("open_width_m"),
            lift_distance_m=experiment.resolved_config["user_config"]["policy_options"].get(
                "expert_policy_overrides", {}).get("lift_distance_m"),
        )
        if args.geometry_backend == "proc3s":
            from examples.online_manipulation.tamp_ccsp import Proc3sCCSPSolver
            return Proc3sCCSPSolver(registry, domain, trace=trace, rng=ccsp_rng,
                                    **settings.get("proc3s_ccsp", {}))
        return SamplingSolver(registry, domain, trace=trace, **settings["sampling"])

    runner = IncrementalTampRunner(
        semantic=semantic, registry=registry, solver_factory=solver_factory,
        executor=executor, observer=observer,
        trace=trace,
        limits=RecoveryLimits(**settings["recovery"]),
        program_generator=program_generator,
    )
    if args.record_html:
        env.start_recording()
    try:
        result = runner.run(
            task=args.task,
            task_goals=(PredicateGoal("holding", (target,)),),
            initial_observation=observation,
            initial_geometry_state={"base_height_m":
                experiment.environment_config.robot_adapter.base_config.base_height_m},
            predicate_arity={"observed": 1, "at_pick_pose": 1,
                             "holding": 1, "gripper_empty": 0},
            images=images,
        )
    finally:
        if args.record_html:
            env.save_recording(output / "simulation.html")
    (output / "result.json").write_text(json.dumps({
        "planner": "tamp", "tamp_mode": "hierarchical",
        "geometry_backend": args.geometry_backend,
        "skill_planner": args.skill_planner,
        "recording_html": str(output / "simulation.html") if args.record_html else None,
        "seed": seed,
        "model_source": "recorded" if args.recorded_subgoals else "live_vlm",
        "success": result.success, "reason": result.reason,
        "metrics": dict(result.metrics),
        "final_facts": [dataclasses.asdict(fact) for fact in result.world.facts],
    }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"success": result.success, "reason": result.reason,
                      "result": str(output / "result.json")}))
    return 0 if result.success else 2


if __name__ == "__main__":
    raise SystemExit(main())
