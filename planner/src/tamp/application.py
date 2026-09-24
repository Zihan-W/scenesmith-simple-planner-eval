"""Single assembly path for the CLI and scene-validation harness."""
import dataclasses
import random
from pathlib import Path

from .ccsp import Proc3sCCSPSolver
from .online import IncrementalTampRunner, RecoveryLimits
from .proc3s import PRoC3SProgramGenerator
from .scenesmith import SceneSmithPickDomain
from .semantic import ModelSettings
from .skill_planning import StripsProgramGenerator
from .task_domain import PICK_LIFT_DOMAIN


def create_session(*, env, experiment, repository_root, output_root, task,
                   settings, client, reset_info, skill_planner='proc3s', geometry_backend='proc3s',
                   seed=500, cutamp_settings=None, semantic=None, trace=None,
                   model_called=False, deadline_monotonic_s=None, base_candidates=()):
    """Prepare a plan/execute session from an already reset simulation.

    The caller owns the environment, model client, recording and process-level
    supervision. This function never resets or steps the environment. Model
    calls start at session.plan(); client may be a live or strict replay client.
    Current production task goals and bindings come from PICK_LIFT_DOMAIN.
    """
    import time

    from .online import JsonlTrace
    from .scenesmith_online import SceneSmithSkillExecutor, SceneSmithWorldObserver
    from .semantic import SemanticSubgoalPlanner

    if geometry_backend not in {'proc3s', 'cutamp'}:
        raise ValueError(f'Unknown geometry backend: {geometry_backend}')
    if (geometry_backend == 'cutamp') != (cutamp_settings is not None):
        raise ValueError('cutamp_settings is required exactly for the cuTAMP backend')
    if skill_planner not in {'strips', 'proc3s'}:
        raise ValueError(f'Unknown skill planner: {skill_planner}')
    if client is None and (semantic is None or skill_planner == 'proc3s'):
        raise ValueError('A model client is required for semantic/PRoC3S generation')
    limits = RecoveryLimits(**settings['recovery'])
    deadline = (time.perf_counter() + limits.max_wall_time_s
                if deadline_monotonic_s is None else deadline_monotonic_s)
    output = Path(output_root)
    output.mkdir(parents=True, exist_ok=True)
    if trace is None:
        trace = JsonlTrace(output / 'tamp_trace.jsonl')
    observation = env.observation
    task_instance = experiment.environment_config.task_factory()
    target = getattr(task_instance, 'task', task_instance).config.target_observation_name
    registry = PICK_LIFT_DOMAIN.registry()
    executor = SceneSmithSkillExecutor(env=env, experiment=experiment,
        repository_root=Path(repository_root), output_root=output,
        observation=observation, reset_info=reset_info,
        max_skill_steps=int(settings['max_skill_steps']), model_called=model_called,
        registry=registry, grasp_compensation=settings.get('grasp_compensation'))
    scene_manifest = experiment.resolved_config['provenance'].get('scene_manifest')
    observer = SceneSmithWorldObserver(executor, target, output,
        scene_metadata=Path(scene_manifest).with_name('scene_metadata.json')
        if scene_manifest else None)
    if semantic is None:
        semantic = SemanticSubgoalPlanner(client, ModelSettings(**settings['subgoal_model']), trace=trace)
    runner = build_runner(semantic=semantic, client=client, settings=settings,
        registry=registry, executor=executor, observer=observer, env=env,
        experiment=experiment, repository_root=repository_root, output_root=output,
        trace=trace, skill_planner=skill_planner, geometry_backend=geometry_backend,
        seed=seed, cutamp_settings=cutamp_settings, base_candidates=base_candidates)
    return runner.start(task=task, task_goals=PICK_LIFT_DOMAIN.goals(target),
        initial_observation=observation,
        initial_geometry_state={'base_height_m':
            experiment.environment_config.robot_adapter.base_config.base_height_m},
        predicate_arity=registry.predicate_arity, images=observer.capture_images(observation),
        deadline_monotonic_s=deadline)


class SnapshotSolver:
    """Bind all geometry backends to the same observation-owned physical state."""
    def __init__(self, solver, snapshot):
        self.solver, self.snapshot = solver, snapshot
        self.capabilities = solver.capabilities

    def set_deadline(self, deadline_monotonic_s):
        """Forward the run budget without changing the snapshot binding."""
        if hasattr(self.solver, "set_deadline"):
            self.solver.set_deadline(deadline_monotonic_s)

    def solve(self, world, program, initial_state, **kwargs):
        self.snapshot.require_world(world)
        if initial_state.get("snapshot_token", self.snapshot.token) != self.snapshot.token:
            raise ValueError("Geometry state belongs to a different planning snapshot")
        return self.solver.solve(world, program,
                                 self.snapshot.geometry_state(initial_state), **kwargs)


def build_runner(*, semantic, client, settings, registry, executor, observer,
                 env, experiment, repository_root, output_root, trace,
                 skill_planner, geometry_backend, seed, cutamp_settings=None,
                 base_candidates=()):
    """Construct one runner; all budgets and task inputs come from caller config."""
    repo, output = Path(repository_root), Path(output_root)
    if geometry_backend not in {"proc3s", "cutamp"}:
        raise ValueError(f"Unknown geometry backend: {geometry_backend}")
    if skill_planner not in {"strips", "proc3s"}:
        raise ValueError(f"Unknown skill planner: {skill_planner}")
    generator = (PRoC3SProgramGenerator(client, ModelSettings(**settings["skill_model"]),
                                      registry, trace=trace)
                 if skill_planner == "proc3s" else StripsProgramGenerator(registry))
    rng, solve_count = random.Random(seed), 0

    def solver_factory(world):
        nonlocal solve_count
        solve_count += 1
        overrides = experiment.resolved_config["user_config"]["policy_options"].get(
            "expert_policy_overrides", {})
        domain = SceneSmithPickDomain(
            environment_config=experiment.environment_config,
            observation=executor.observation, planning_query=env.get_planning_query(),
            calibration_path=repo / "experiments/inputs/pick_lift/pick_lift_calibration.json",
            pick_home_path=repo / "experiments/inputs/pick_lift/pick_home.json",
            base_candidates=base_candidates, open_width_m=overrides.get("open_width_m"),
            lift_distance_m=overrides.get("lift_distance_m"))
        domain.snapshot.require_world(world)
        if geometry_backend == "cutamp":
            from .cutamp import CuTAMPSolver
            if cutamp_settings is None:
                raise ValueError("cuTAMP requires explicit settings")
            solver = CuTAMPSolver(registry, domain, settings=dataclasses.replace(
                cutamp_settings, seed=(cutamp_settings.seed + solve_count - 1) % 2**32),
                resolved_config=experiment.resolved_config, repository_root=repo,
                output_root=output / "cutamp" / f"solve_{solve_count:03d}", trace=trace)
        else:
            solver = Proc3sCCSPSolver(registry, domain, trace=trace, rng=rng,
                                      **settings.get("proc3s_ccsp", {}))
        trace({"event": "solver_capabilities", "backend": geometry_backend,
               "snapshot_token": domain.snapshot.token,
               "capabilities": dataclasses.asdict(solver.capabilities)})
        return SnapshotSolver(solver, domain.snapshot)

    return IncrementalTampRunner(semantic=semantic, registry=registry,
        solver_factory=solver_factory, executor=executor, observer=observer,
        trace=trace, limits=RecoveryLimits(**settings["recovery"]),
        program_generator=generator)
