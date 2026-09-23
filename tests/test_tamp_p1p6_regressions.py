"""Bounded recovery, cuTAMP preconditions and whole-run deadline contracts."""

import json
from pathlib import Path
import subprocess
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from planner.src.bt.generation import GenerationError, OpenAICompatibleChatClient
from planner.src.tamp.cutamp import CuTAMPSettings, CuTAMPSolver, run_cutamp_worker
from planner.src.tamp.geometry import (
    GeometricUnsat, GeometryBackendError, GeometryDeadlineExceeded,
    SolverCapabilities,
)
from planner.src.tamp.failures import ProgramFailure
from planner.src.tamp.hierarchy import (
    ConstraintResult, PredicateGoal, SkillProgram, SkillStep, WorldState,
    picklift_registry,
)
from planner.src.tamp.online import (
    IncrementalTampRunner, RecoveryLimits,
)


class RegressionTests(unittest.TestCase):
    def test_default_run_budget_is_twenty_minutes(self):
        self.assertEqual(RecoveryLimits().max_wall_time_s, 1200.0)

    def test_model_transport_does_not_start_after_deadline(self):
        client = OpenAICompatibleChatClient(
            base_url="https://example.invalid/v1", api_key="fixture")
        client.set_deadline(time.perf_counter() - 1)
        with patch("planner.src.bt.generation.urlopen") as request:
            with self.assertRaises(GenerationError):
                client.complete(model="fixture", messages=[
                    {"role": "user", "content": "test"}])
            request.assert_not_called()

    def test_tilted_base_fails_as_declared_precondition_before_worker(self):
        registry = picklift_registry()
        world = WorldState({"red_cube": {}}, frozenset(), observation_id="parked")
        program = SkillProgram((SkillStep("NavigateToPick", {"object": "red_cube"},
                                         {"base_pose": "b"}),))

        def build(_program, _world, problem):
            problem.mkdir(parents=True)
            # Rotation about world Y; the base is slightly tilted after parking.
            (problem / "geometry.json").write_text(json.dumps({
                "observed_robot_base_world_matrix": [
                    [1.0, 0.0, 0.00016, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [-0.00016, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]}))
            return {"optimize_base_xy": True}

        with tempfile.TemporaryDirectory() as directory, patch(
                "planner.src.tamp.cutamp.ContinuousProblemBuilder",
                return_value=SimpleNamespace(build=build)), patch(
                "planner.src.tamp.cutamp.run_cutamp_worker") as worker:
            settings = CuTAMPSettings(
                gpu_python="/bin/false", cutamp_root=directory,
                kinematics_template=directory, robot_template=directory,
                multipliers=directory, tolerances=directory,
                grasp_calibration=directory)
            solver = CuTAMPSolver(
                registry, SimpleNamespace(target_name="red_cube"), settings=settings,
                resolved_config={}, repository_root=directory,
                output_root=Path(directory) / "solver")
            with self.assertRaises(GeometricUnsat) as caught:
                solver.solve(world, program, {})
            worker.assert_not_called()
            self.assertEqual(caught.exception.program_feedback["failure_source"],
                             "backend_precondition")
            self.assertFalse(caught.exception.retryable_search)
            self.assertEqual(solver.capabilities.base_anchor_requirement,
                             "upright_for_base_xy")

    def test_worker_nonzero_exit_preserves_log_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            settings = SimpleNamespace(
                gpu_python="/bin/false", cutamp_root=directory,
                multipliers=directory, tolerances=directory,
                grasp_calibration=directory, num_particles=1,
                num_opt_steps=1, conf_lr=0.001, seed=0)
            with patch("planner.src.tamp.cutamp_backend.integrity.verify_installation",
                       return_value={}), patch("planner.src.tamp.cutamp.subprocess.run",
                       side_effect=subprocess.CalledProcessError(7, ["worker"])):
                with self.assertRaises(GeometryBackendError) as caught:
                    run_cutamp_worker(settings, root, root, root / "optimizer",
                                      optimize_base_xy=True)
            self.assertEqual(caught.exception.returncode, 7)
            self.assertTrue(Path(caught.exception.log_path).exists())

    def test_worker_timeout_is_reported_as_wall_budget(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            settings = SimpleNamespace(
                gpu_python="/bin/false", cutamp_root=directory,
                multipliers=directory, tolerances=directory,
                grasp_calibration=directory, num_particles=1,
                num_opt_steps=1, conf_lr=0.001, seed=0)
            with patch("planner.src.tamp.cutamp_backend.integrity.verify_installation",
                       return_value={}), patch("planner.src.tamp.cutamp.subprocess.run",
                       side_effect=subprocess.TimeoutExpired(["worker"], 1)):
                with self.assertRaises(GeometryDeadlineExceeded):
                    run_cutamp_worker(settings, root, root, root / "optimizer",
                                      optimize_base_xy=True,
                                      deadline_monotonic_s=time.perf_counter() + 2)
            self.assertTrue((root / "optimizer.log").exists())

    def test_backend_process_error_returns_structured_run_result(self):
        from test_tamp_online import Executor, Observer, Semantic

        class FailedSolver:
            capabilities = SolverCapabilities()

            def solve(self, *args, **kwargs):
                raise GeometryBackendError(returncode=7, log_path="/tmp/optimizer.log")

        registry = picklift_registry()
        seen = PredicateGoal("observed", ("red_cube",))
        empty = PredicateGoal("gripper_empty", ())
        holding = PredicateGoal("holding", ("red_cube",))
        events = []
        runner = IncrementalTampRunner(
            semantic=Semantic(((holding,),)), registry=registry,
            solver_factory=lambda world: FailedSolver(),
            executor=Executor((seen, empty)),
            observer=Observer(("red_cube",)), trace=events.append,
        )
        result = runner.run(task="pick red_cube", task_goals=(holding,),
                            initial_observation=(seen, empty), initial_geometry_state={},
                            predicate_arity={"holding": 1})
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "geometry_backend_error")
        self.assertEqual(events[-2]["event"], "geometry_backend_error")
        self.assertEqual(events[-1]["event"], "final_result")

    def test_exact_and_approximate_evidence_are_both_retained(self):
        registry = picklift_registry()
        world = WorldState({"red_cube": {}}, frozenset(), observation_id="parked")
        program = SkillProgram((SkillStep("PickLift", {"object": "red_cube"},
                                         {"grasp_pose": "g", "approach_pose": "a"}),))
        particle = {"particle_index": 0, "optimizer_feasible": False,
                    "constraint_diagnostics": {"KinematicConstraint": {
                        "pos_err": {"satisfied": False}}}}
        failure = ConstraintResult(False, "pick_ik_grasp", "g", "rejected",
                                   ("red_cube",), {"program_step": 0})

        def build(_program, _world, problem):
            problem.mkdir(parents=True)
            return {"optimize_base_xy": False}

        def worker(_settings, _repo, _problem, output, **_kwargs):
            output.mkdir(parents=True)
            (output.parent / "candidate_selection.json").write_text(json.dumps({
                "stop_reason": "all_candidates_checked", "candidates_completed": 1,
                "count_exhausted": False}))
            (output / "candidate_assignments.json").write_text(
                json.dumps({"candidates": [particle]}))
            return {"timed_out": False, "num_opt_steps": 1,
                    "constraint_summary": {}}

        with tempfile.TemporaryDirectory() as directory, patch(
                "planner.src.tamp.cutamp.ContinuousProblemBuilder",
                return_value=SimpleNamespace(build=build)), patch(
                "planner.src.tamp.cutamp.run_cutamp_worker", side_effect=worker), patch(
                "planner.src.tamp.cutamp.postcheck_cutamp_candidates",
                return_value=(None, [{"optimizer_feasible": False,
                                      "approximate_exact_disagreement": False}], (failure,))):
            settings = CuTAMPSettings(
                gpu_python="/bin/false", cutamp_root=directory,
                kinematics_template=directory, robot_template=directory,
                multipliers=directory, tolerances=directory,
                grasp_calibration=directory)
            solver = CuTAMPSolver(
                registry, SimpleNamespace(target_name="red_cube"), settings=settings,
                resolved_config={}, repository_root=directory,
                output_root=Path(directory) / "solver")
            with self.assertRaises(GeometricUnsat) as caught:
                solver.solve(world, program, {})
            self.assertEqual(
                [item["failure_source"] for item in caught.exception.program_feedbacks],
                ["approximate_tolerance_unmet", "exact_postcheck_rejected"])

    def test_deadline_stops_before_any_skill_command(self):
        from planner.src.tamp.geometry import SamplingSolver
        from test_tamp_online import Domain, Executor, Observer, Semantic

        registry = picklift_registry()
        seen = PredicateGoal("observed", ("red_cube",))
        empty = PredicateGoal("gripper_empty", ())
        holding = PredicateGoal("holding", ("red_cube",))
        initial = (seen, empty)
        executor = Executor(initial)
        events = []
        runner = IncrementalTampRunner(
            semantic=Semantic(((holding,),)), registry=registry,
            solver_factory=lambda world: SamplingSolver(registry, Domain()),
            executor=executor, observer=Observer(("red_cube",)), trace=events.append,
            limits=RecoveryLimits(max_wall_time_s=1200),
        )
        result = runner.run(task="pick red_cube", task_goals=(holding,),
                            initial_observation=initial, initial_geometry_state={},
                            predicate_arity={"holding": 1},
                            deadline_monotonic_s=time.perf_counter() - 1)
        self.assertEqual(result.reason, "wall_time_budget_exhausted")
        self.assertEqual(executor.calls, [])
        self.assertEqual(events[-1]["event"], "final_result")

    def test_runner_passes_both_failure_streams_to_program_generator(self):
        from planner.src.tamp.geometry import SamplingSolver
        from test_tamp_online import Domain, Executor, Observer, Semantic

        registry = picklift_registry()
        seen = PredicateGoal("observed", ("red_cube",))
        empty = PredicateGoal("gripper_empty", ())
        parked = PredicateGoal("at_pick_pose", ("red_cube",))
        holding = PredicateGoal("holding", ("red_cube",))

        class Generator:
            calls = 0
            received = []

            def generate(self, world, goals, *, feedback=(), excluded_programs=frozenset()):
                self.calls += 1
                self.received.append(feedback)
                return SkillProgram((SkillStep("PickLift", {"object": "red_cube"},
                                               {"grasp_pose": "g", "approach_pose": "a"}),))

        solves = []

        class Solver(SamplingSolver):
            def solve(self, *args, **kwargs):
                solves.append(None)
                if len(solves) == 1:
                    error = GeometricUnsat((ConstraintResult(
                        False, "pick_ik_grasp", "g", "rejected", ("red_cube",)),))
                    error.program_feedbacks = (
                        ProgramFailure("", ("ik",), ("red_cube",),
                                       failure_source="approximate_tolerance_unmet",
                                       attribution_scope="global").as_feedback(),
                        ProgramFailure("PickLift", ("collision",), ("red_cube",),
                                       failure_source="exact_postcheck_rejected").as_feedback(),
                    )
                    raise error
                return super().solve(*args, **kwargs)

        generator = Generator()
        runner = IncrementalTampRunner(
            semantic=Semantic(((holding,),)), registry=registry,
            solver_factory=lambda world: Solver(registry, Domain()),
            executor=Executor((seen, empty, parked)),
            observer=Observer(("red_cube",)), trace=lambda event: None,
            program_generator=generator,
            limits=RecoveryLimits(geometry_retries=0, skill_replans=1),
        )
        result = runner.run(task="pick red_cube", task_goals=(holding,),
                            initial_observation=(seen, empty, parked),
                            initial_geometry_state={}, predicate_arity={"holding": 1})
        self.assertTrue(result.success)
        self.assertEqual(
            [item["failure_source"] for item in generator.received[1]],
            ["approximate_tolerance_unmet", "exact_postcheck_rejected"])


if __name__ == "__main__":
    unittest.main()
