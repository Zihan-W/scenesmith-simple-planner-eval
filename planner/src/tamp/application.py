"""Single assembly path for the CLI and scene-validation harness."""
import dataclasses
import random
from pathlib import Path

from .ccsp import Proc3sCCSPSolver
from .geometry import SamplingSolver
from .online import IncrementalTampRunner, RecoveryLimits
from .proc3s import PRoC3SProgramGenerator
from .scenesmith import SceneSmithPickDomain
from .semantic import ModelSettings
from .skill_planning import StripsProgramGenerator
from .task_domain import PICK_LIFT_DOMAIN


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
    if geometry_backend not in {"sampling", "proc3s", "cutamp"}:
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
        elif geometry_backend == "proc3s":
            solver = Proc3sCCSPSolver(registry, domain, trace=trace, rng=rng,
                                      **settings.get("proc3s_ccsp", {}))
        else:
            solver = SamplingSolver(registry, domain, trace=trace, **settings["sampling"])
        trace({"event": "solver_capabilities", "backend": geometry_backend,
               "snapshot_token": domain.snapshot.token,
               "capabilities": dataclasses.asdict(solver.capabilities)})
        return SnapshotSolver(solver, domain.snapshot)

    return IncrementalTampRunner(semantic=semantic, registry=registry,
        solver_factory=solver_factory, executor=executor, observer=observer,
        trace=trace, limits=RecoveryLimits(**settings["recovery"]),
        program_generator=generator)
