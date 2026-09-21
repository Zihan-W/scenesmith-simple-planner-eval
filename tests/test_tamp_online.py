"""Five bounded hierarchical TAMP cases without a simulator dependency."""

import unittest

from examples.online_manipulation.tamp_geometry import SamplingSolver
from examples.online_manipulation.tamp_hierarchy import (
    PredicateGoal, SkillRegistry, SkillSpec, WorldState, picklift_registry,
)
from examples.online_manipulation.tamp_online import (
    IncrementalTampRunner, RecoveryLimits, SkillExecution,
)


class Observer:
    def __init__(self, objects):
        self.objects = {name: {} for name in objects}

    def observe(self, facts):
        return WorldState(self.objects, frozenset(facts))


class Semantic:
    def __init__(self, proposals):
        self.proposals = list(proposals)
        self.calls = 0
        self.feedback = []

    def propose(self, *, task, world, predicate_arity, feedback, images):
        self.calls += 1
        self.feedback.append(feedback)
        return self.proposals.pop(0)


class Domain:
    def __init__(self, *, first_geometry_failure=False, blocked_skills=()):
        self.first_geometry_failure = first_geometry_failure
        self.blocked_skills = set(blocked_skills)
        self.checked = []

    def samples(self, skill, state):
        if skill.skill == "NavigateToPick":
            yield {"base_x_m": 1.0, "base_y_m": 0.0, "base_yaw_rad": 0.0}
            yield {"base_x_m": 2.0, "base_y_m": 0.0, "base_yaw_rad": 0.0}
        elif skill.skill == "PickLift":
            yield {"target": "red_cube", "arm": "left", "grasp_lateral_offset_m": 0.0}
        else:
            yield {}

    def check(self, skill, candidate, state):
        self.checked.append((skill.skill, dict(candidate)))
        if skill.skill in self.blocked_skills:
            return False, "ik", {"message": "blocked"}
        if self.first_geometry_failure and candidate.get("base_x_m") == 1.0:
            return False, "corridor", {"message": "blocked"}
        return True, "valid", {}

    def predict(self, skill, candidate, state):
        if skill.skill == "NavigateToPick":
            state["base_pose"] = candidate["base_x_m"]
        return state


class Executor:
    def __init__(self, facts, *, fail_once_skill=None):
        self.facts = set(facts)
        self.fail_once_skill = fail_once_skill
        self.calls = []

    def execute(self, action):
        self.calls.append(action.skill_name)
        if action.skill_name == self.fail_once_skill:
            self.fail_once_skill = None
            return SkillExecution(frozenset(self.facts), False, "transient")
        self.facts.update(action.expected_effects)
        return SkillExecution(frozenset(self.facts), True, "done")


def run_case(registry, semantic, domain, initial, goal, *, fail_once_skill=None,
             limits=RecoveryLimits()):
    events = []
    executor = Executor(initial, fail_once_skill=fail_once_skill)
    runner = IncrementalTampRunner(
        semantic=semantic, registry=registry,
        solver_factory=lambda world: SamplingSolver(registry, domain, batch_size=2),
        executor=executor, observer=Observer(("red_cube",)),
        trace=events.append, limits=limits,
    )
    result = runner.run(
        task="pick red_cube", task_goals=(goal,),
        initial_observation=frozenset(initial),
        initial_geometry_state={"base_height_m": 0.18},
        predicate_arity={goal.predicate: len(goal.arguments),
                         "holding": 1, "reachable": 1},
    )
    return result, executor, events


class OnlineRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.seen = PredicateGoal("observed", ("red_cube",))
        self.empty = PredicateGoal("gripper_empty", ())
        self.holding = PredicateGoal("holding", ("red_cube",))

    def test_a_basic_pick_is_incremental(self):
        result, executor, events = run_case(
            picklift_registry(), Semantic(((self.holding,),)), Domain(),
            (self.seen, self.empty), self.holding,
        )
        self.assertTrue(result.success)
        self.assertEqual(executor.calls, ["NavigateToPick", "PickLift"])
        self.assertEqual(sum(e["event"] == "verifier_result" for e in events), 2)

    def test_b_geometric_failure_tries_alternative_automatically(self):
        domain = Domain(first_geometry_failure=True)
        result, executor, events = run_case(
            picklift_registry(), Semantic(((self.holding,),)), domain,
            (self.seen, self.empty), self.holding,
        )
        self.assertTrue(result.success)
        self.assertEqual(executor.calls, ["NavigateToPick", "PickLift"])
        self.assertTrue(any(e["event"] == "selected_parameters" and
                            e["constraint_failures"] for e in events))

    def test_c_execution_failure_retries_then_verifies(self):
        result, executor, events = run_case(
            picklift_registry(), Semantic(((self.holding,),)), Domain(),
            (self.seen, self.empty), self.holding, fail_once_skill="PickLift",
        )
        self.assertTrue(result.success)
        self.assertEqual(executor.calls[-2:], ["PickLift", "PickLift"])
        self.assertEqual(result.metrics["num_skill_retries"], 1)

    def test_d_unsat_skeleton_replans_to_other_skill(self):
        obj = ("$object",)
        registry = SkillRegistry((
            SkillSpec("DirectPick", ("object",), (), (PredicateGoal("observed", obj),),
                      (PredicateGoal("holding", obj),)),
            SkillSpec("Clear", ("object",), (), (PredicateGoal("observed", obj),),
                      (PredicateGoal("clear", obj),)),
            SkillSpec("AlternativePick", ("object",), (),
                      (PredicateGoal("clear", obj),),
                      (PredicateGoal("holding", obj),)),
        ))
        result, executor, events = run_case(
            registry, Semantic(((self.holding,),)),
            Domain(blocked_skills=("DirectPick",)), (self.seen,), self.holding,
        )
        self.assertTrue(result.success)
        self.assertEqual(executor.calls, ["Clear", "AlternativePick"])
        # Clear + DirectPick must also be checked before ruling out that
        # same skill in the changed state and selecting AlternativePick.
        # After Clear executes, refinement uses its measured state and checks
        # DirectPick once more instead of retaining stale infeasibility.
        self.assertEqual(result.metrics["num_skill_replans"], 3)

    def test_e_unreachable_semantic_goal_prompts_revised_goal(self):
        reachable = PredicateGoal("reachable", ("red_cube",))
        obj = ("$object",)
        registry = SkillRegistry((SkillSpec(
            "Inspect", ("object",), (), (PredicateGoal("observed", obj),),
            (PredicateGoal("reachable", obj),)),))
        semantic = Semantic(((self.holding,), (reachable,)))
        result, executor, events = run_case(
            registry, semantic, Domain(), (self.seen,), reachable,
            limits=RecoveryLimits(semantic_replans=1, skill_replans=0),
        )
        self.assertTrue(result.success)
        self.assertEqual(executor.calls, ["Inspect"])
        self.assertEqual(result.metrics["num_semantic_replans"], 1)
        self.assertTrue(semantic.feedback[-1])

    def test_unsat_replans_clear_then_same_pick_skill(self):
        obj = ("$object",)
        registry = SkillRegistry((
            SkillSpec("Pick", ("object",), (),
                      (PredicateGoal("observed", obj),),
                      (PredicateGoal("holding", obj),)),
            SkillSpec("Clear", ("object",), (),
                      (PredicateGoal("observed", obj),),
                      (PredicateGoal("clear", obj),)),
        ))

        class ObstructedDomain(Domain):
            def check(self, skill, candidate, state):
                if skill.skill == "Pick" and not state.get("clear"):
                    return False, "collision", {"object": "obstacle"}
                return True, "valid", {}

            def predict(self, skill, candidate, state):
                if skill.skill == "Clear":
                    state["clear"] = True
                return state

        class MeasuredObserver(Observer):
            def geometry_state(self, observation, previous):
                return {**previous, "clear":
                        PredicateGoal("clear", ("red_cube",)) in observation}

        events = []
        executor = Executor((self.seen,))
        runner = IncrementalTampRunner(
            semantic=Semantic(((self.holding,),)), registry=registry,
            solver_factory=lambda world: SamplingSolver(registry, ObstructedDomain()),
            executor=executor, observer=MeasuredObserver(("red_cube",)),
            trace=events.append,
        )
        result = runner.run(
            task="pick red_cube", task_goals=(self.holding,),
            initial_observation=(self.seen,), initial_geometry_state={},
            predicate_arity={"holding": 1},
        )
        self.assertTrue(result.success)
        self.assertEqual(executor.calls, ["Clear", "Pick"])
        self.assertEqual(result.metrics["num_skill_replans"], 1)
        replans = [e for e in events if e.get("level") == "skill_replan"]
        self.assertIn("obstacle", replans[0]["feedback"][0]["involved_objects"])

    def test_failed_execution_refreshes_geometry_before_reparameterization(self):
        seen_states = []

        class MeasuredObserver(Observer):
            def geometry_state(self, observation, previous):
                return {**previous, "measured_after_execution": True}

        class MeasuredDomain(Domain):
            def check(self, skill, candidate, state):
                seen_states.append((len(executor.calls), dict(state)))
                return super().check(skill, candidate, state)

        registry = picklift_registry()
        executor = Executor((self.seen, self.empty), fail_once_skill="NavigateToPick")
        runner = IncrementalTampRunner(
            semantic=Semantic(((self.holding,),)), registry=registry,
            solver_factory=lambda world: SamplingSolver(registry, MeasuredDomain()),
            executor=executor, observer=MeasuredObserver(("red_cube",)),
            trace=lambda event: None, limits=RecoveryLimits(skill_retries=0),
        )
        result = runner.run(
            task="pick red_cube", task_goals=(self.holding,),
            initial_observation=(self.seen, self.empty), initial_geometry_state={},
            predicate_arity={"holding": 1},
        )
        self.assertTrue(result.success)
        self.assertEqual(executor.calls, ["NavigateToPick", "NavigateToPick", "PickLift"])
        retry_states = [state for count, state in seen_states if count == 1]
        self.assertTrue(retry_states)
        self.assertTrue(all(s.get("measured_after_execution") for s in retry_states))
        self.assertEqual(result.metrics["num_geometry_retries"], 1)

    def test_retryable_failure_with_changed_robot_rechecks_geometry(self):
        events, solving_worlds = [], []
        executor = Executor((self.seen, self.empty), fail_once_skill="NavigateToPick")

        class MovingObserver(Observer):
            def observe(self, facts):
                return WorldState(self.objects, frozenset(facts),
                                  robot={"execution_count": len(executor.calls)})

        registry = picklift_registry()

        def solver_factory(world):
            solving_worlds.append(world.robot["execution_count"])
            return SamplingSolver(registry, Domain())

        runner = IncrementalTampRunner(
            semantic=Semantic(((self.holding,),)), registry=registry,
            solver_factory=solver_factory, executor=executor,
            observer=MovingObserver(("red_cube",)), trace=events.append,
            limits=RecoveryLimits(skill_retries=2),
        )
        result = runner.run(
            task="pick red_cube", task_goals=(self.holding,),
            initial_observation=(self.seen, self.empty), initial_geometry_state={},
            predicate_arity={"holding": 1},
        )
        self.assertTrue(result.success)
        self.assertEqual(executor.calls, ["NavigateToPick", "NavigateToPick", "PickLift"])
        self.assertEqual(solving_worlds[:2], [0, 1])
        self.assertEqual(result.metrics["num_skill_retries"], 0)
        self.assertEqual(result.metrics["num_geometry_retries"], 1)
        self.assertTrue(any(event.get("level") == "state_changed_geometry_recheck"
                            for event in events))

    def test_episode_end_verifies_final_observation_without_retry(self):
        for final_success in (False, True):
            with self.subTest(final_success=final_success):
                seen, empty, holding = self.seen, self.empty, self.holding

                class TerminalExecutor:
                    calls = 0

                    def execute(self, action):
                        self.calls += 1
                        if self.calls > 1:
                            raise AssertionError("Cannot step a finished episode")
                        facts = {seen, empty}
                        if final_success:
                            facts.add(holding)
                        return SkillExecution(facts, False, "episode_finished",
                                              episode_finished=True)

                registry = picklift_registry()
                executor = TerminalExecutor()
                runner = IncrementalTampRunner(
                    semantic=Semantic(((holding,),)), registry=registry,
                    solver_factory=lambda world: SamplingSolver(registry, Domain()),
                    executor=executor, observer=Observer(("red_cube",)),
                    trace=lambda event: None,
                )
                result = runner.run(
                    task="pick red_cube", task_goals=(holding,),
                    initial_observation=(seen, empty), initial_geometry_state={},
                    predicate_arity={"holding": 1},
                )
                self.assertEqual(result.success, final_success)
                self.assertEqual(executor.calls, 1)
                self.assertEqual(result.metrics["num_skill_retries"], 0)

    def test_reparameterization_excludes_same_sample_despite_changed_ik_solution(self):
        class ChangingIKDomain(Domain):
            checks = 0

            def samples(self, skill, state):
                for offset in (0.0, 0.003):
                    yield {"target": "red_cube", "arm": "left", "grasp_lateral_offset_m": offset}

            def check(self, skill, candidate, state):
                self.checks += 1
                return True, "valid", {"ik": {"grasp_pose_in_target": {"q": self.checks}}}

        class FailingOnceExecutor(Executor):
            offsets = []

            def execute(self, action):
                self.offsets.append(action.geometric_parameters["grasp_lateral_offset_m"])
                return super().execute(action)

        initial = (self.seen, self.empty, PredicateGoal("at_pick_pose", ("red_cube",)))
        executor = FailingOnceExecutor(initial, fail_once_skill="PickLift")
        domain = ChangingIKDomain()
        registry = picklift_registry()
        runner = IncrementalTampRunner(
            semantic=Semantic(((self.holding,),)), registry=registry,
            solver_factory=lambda world: SamplingSolver(registry, domain),
            executor=executor, observer=Observer(("red_cube",)),
            trace=lambda event: None, limits=RecoveryLimits(skill_retries=0),
        )
        result = runner.run(task="pick red_cube", task_goals=(self.holding,),
                            initial_observation=initial, initial_geometry_state={},
                            predicate_arity={"holding": 1})
        self.assertTrue(result.success)
        self.assertEqual(executor.offsets, [0.0, 0.003])

    def test_successful_semantic_horizon_does_not_spend_recovery_budget(self):
        parked = PredicateGoal("at_pick_pose", ("red_cube",))
        result, executor, events = run_case(
            picklift_registry(), Semantic(((parked,), (self.holding,))), Domain(),
            (self.seen, self.empty), self.holding,
            limits=RecoveryLimits(semantic_replans=0),
        )
        self.assertTrue(result.success)
        self.assertEqual(executor.calls, ["NavigateToPick", "PickLift"])
        self.assertEqual(result.metrics["num_semantic_replans"], 0)
        self.assertTrue(any(event["event"] == "semantic_continuation" for event in events))

    def test_deterministic_path_rejection_skips_skill_retry(self):
        class RejectedExecutor(Executor):
            def execute(self, action):
                outcome = super().execute(action)
                if not outcome.success:
                    return SkillExecution(outcome.observation, False, "joint_edge_rejected",
                                          retryable=False)
                return outcome

        initial = (self.seen, self.empty)
        executor = RejectedExecutor(initial, fail_once_skill="NavigateToPick")
        registry = picklift_registry()
        runner = IncrementalTampRunner(
            semantic=Semantic(((self.holding,),)), registry=registry,
            solver_factory=lambda world: SamplingSolver(registry, Domain()),
            executor=executor, observer=Observer(("red_cube",)), trace=lambda event: None,
            limits=RecoveryLimits(skill_retries=3),
        )
        result = runner.run(task="pick red_cube", task_goals=(self.holding,),
                            initial_observation=initial, initial_geometry_state={},
                            predicate_arity={"holding": 1})
        self.assertTrue(result.success)
        self.assertEqual(result.metrics["num_skill_retries"], 0)
        self.assertEqual(result.metrics["num_geometry_retries"], 1)

    def test_exhausted_model_retries_produce_terminal_trace(self):
        from examples.online_manipulation.tamp_semantic import SemanticModelError

        class FailedSemantic:
            calls = 3

            def propose(self, **kwargs):
                raise SemanticModelError("No valid semantic response after 3 attempts")

        result, executor, events = run_case(
            picklift_registry(), FailedSemantic(), Domain(),
            (self.seen, self.empty), self.holding,
        )
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "semantic_model_error")
        self.assertEqual(executor.calls, [])
        self.assertEqual(result.metrics["vlm_calls"], 3)
        self.assertEqual(events[-1]["event"], "final_result")

    def test_runtime_blocker_reaches_semantic_feedback_without_joint_numbers(self):
        import json

        initial = (self.seen, self.empty)
        details = {"joint_configuration": [123.456],
                   "edge": {"limiting_nonpenetration_pair": {"body_a": "drawer::link"}}}

        class BlockedExecutor:
            def execute(self, action):
                return SkillExecution(initial, False, "joint_edge_rejected", retryable=False,
                                      failure_details=details, involved_objects=("drawer",))

        registry = picklift_registry()
        semantic = Semantic(((self.holding,), (self.holding,)))
        events = []
        runner = IncrementalTampRunner(
            semantic=semantic, registry=registry,
            solver_factory=lambda world: SamplingSolver(registry, Domain()),
            executor=BlockedExecutor(), observer=Observer(("red_cube", "drawer")),
            trace=events.append,
            limits=RecoveryLimits(geometry_retries=0, skill_replans=0, semantic_replans=1),
        )
        result = runner.run(task="pick red_cube", task_goals=(self.holding,),
                            initial_observation=initial, initial_geometry_state={},
                            predicate_arity={"holding": 1})
        self.assertFalse(result.success)
        failure = semantic.feedback[1][0]["blocking_failures"][0]
        self.assertEqual(failure["involved_objects"], ["drawer", "red_cube"])
        self.assertNotIn("123.456", json.dumps(semantic.feedback))
        execution = next(event for event in events if event["event"] == "skill_execution")
        self.assertEqual(execution["failure_details"], details)

    def test_geometry_retry_receives_complete_low_level_failure(self):
        initial = (self.seen, self.empty)
        contexts = []
        registry = picklift_registry()

        class Solver(SamplingSolver):
            def solve(self, *args, **kwargs):
                contexts.append(kwargs.get("failure_context"))
                return super().solve(*args, **kwargs)

        class BlockedExecutor:
            def execute(self, action):
                return SkillExecution(initial, False, "joint_edge_rejected", retryable=False,
                                      failure_details={"joint_configuration": [123.456]},
                                      involved_objects=("drawer",))

        runner = IncrementalTampRunner(
            semantic=Semantic(((self.holding,),)), registry=registry,
            solver_factory=lambda world: Solver(registry, Domain()),
            executor=BlockedExecutor(), observer=Observer(("red_cube", "drawer")),
            trace=lambda event: None,
            limits=RecoveryLimits(geometry_retries=1, skill_replans=0, semantic_replans=0))
        runner.run(task="pick red_cube", task_goals=(self.holding,),
                   initial_observation=initial, initial_geometry_state={},
                   predicate_arity={"holding": 1})
        self.assertIsNone(contexts[0])
        self.assertEqual(contexts[1].details["joint_configuration"], [123.456])
        self.assertIn("geometric_parameters", contexts[1].details)
        self.assertIn("assignments", contexts[1].details)
        self.assertIn("drawer", contexts[1].involved_objects)


if __name__ == "__main__":
    unittest.main()
