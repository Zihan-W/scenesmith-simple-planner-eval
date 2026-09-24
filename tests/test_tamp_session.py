"""Plan/execute handoff contracts using the real runner and sampling solver."""

import dataclasses
import json
from pathlib import Path
import tempfile
import time
import unittest

from planner.src.tamp.ccsp import Proc3sCCSPSolver

from planner.src.tamp import StalePlanError
from planner.src.tamp.geometry import GeometryDeadlineExceeded
from planner.src.tamp.hierarchy import PredicateGoal, picklift_registry
from planner.src.tamp.online import IncrementalTampRunner, RecoveryLimits
from test_tamp_online import Domain, Executor, Observer, Semantic


class PlanningSessionTest(unittest.TestCase):
    def session(self, *, fail_once=None, deadline=None):
        registry = picklift_registry()
        facts = (PredicateGoal('observed', ('red_cube',)), PredicateGoal('gripper_empty', ()))
        goal = PredicateGoal('holding', ('red_cube',))
        executor = Executor(facts, fail_once_skill=fail_once)
        domain = Domain()
        events = []
        runner = IncrementalTampRunner(semantic=Semantic(((goal,),)), registry=registry,
            solver_factory=lambda world: Proc3sCCSPSolver(registry, domain, seed=1),
            executor=executor, observer=Observer(('red_cube',)), trace=events.append,
            limits=RecoveryLimits())
        session = runner.start(task='pick', task_goals=(goal,),
            initial_observation=frozenset(facts), initial_geometry_state={},
            predicate_arity=registry.predicate_arity,
            deadline_monotonic_s=deadline,
            state_token=lambda: (frozenset(executor.facts), tuple(executor.calls)))
        return session, executor, domain, events

    def test_planning_does_not_execute_and_only_next_skill_is_consumed(self):
        session, executor, domain, events = self.session()
        with session:
            first = session.plan()
            self.assertEqual(executor.calls, [])
            self.assertEqual(first.as_dict()['next_action']['skill_name'], 'NavigateToPick')
            checked = len(domain.checked)
            self.assertIs(session.plan(), first)
            self.assertEqual(len(domain.checked), checked)
            document = first.as_dict()
            document['next_action']['skill_name'] = 'Modified'
            with tempfile.TemporaryDirectory() as directory:
                path = first.write(Path(directory) / 'plan.json')
                self.assertEqual(json.loads(path.read_text()), first.as_dict())
                with self.assertRaises(FileExistsError):
                    first.write(path)
            session.execute(first)
            self.assertEqual(executor.calls, ['NavigateToPick'])
            with self.assertRaises(ValueError):
                session.execute(first)
            second = session.plan()
            self.assertEqual(second.as_dict()['next_action']['skill_name'], 'PickLift')
            self.assertEqual(executor.calls, ['NavigateToPick'])
            session.execute(second)
            self.assertIsNone(session.plan())
            self.assertTrue(session.result.success)
            self.assertEqual(session.result.metrics['skill_executions'], 2)
            self.assertEqual(sum(event['event'] == 'verifier_result' for event in events), 2)

    def test_foreign_or_reconstructed_handles_cannot_execute(self):
        first, executor, _, _ = self.session()
        second, other, _, _ = self.session()
        with first, second:
            ticket = first.plan()
            foreign = second.plan()
            for rejected in (foreign, dataclasses.replace(ticket)):
                with self.assertRaises(ValueError):
                    first.execute(rejected)
            self.assertEqual(executor.calls, [])
            self.assertEqual(other.calls, [])
            first.execute(ticket)
            self.assertEqual(executor.calls, ['NavigateToPick'])

    def test_external_state_change_invalidates_plan_without_motion(self):
        session, executor, _, _ = self.session()
        ticket = session.plan()
        executor.facts.add(PredicateGoal('holding', ('red_cube',)))
        with self.assertRaises(StalePlanError):
            session.execute(ticket)
        self.assertEqual(executor.calls, [])
        with self.assertRaises(RuntimeError):
            session.plan()

    def test_deadline_includes_waiting_for_explicit_execution(self):
        session, executor, _, _ = self.session(deadline=time.perf_counter() + 0.2)
        ticket = session.plan()
        self.assertIsNotNone(ticket)
        time.sleep(0.25)
        with self.assertRaises(GeometryDeadlineExceeded):
            session.execute(ticket)
        self.assertEqual(executor.calls, [])
        self.assertEqual(session.result.reason, 'wall_time_budget_exhausted')
        self.assertEqual(session.result.metrics['skill_executions'], 0)

    def test_close_cancels_without_executing(self):
        session, executor, _, _ = self.session()
        ticket = session.plan()
        session.close()
        with self.assertRaises(ValueError):
            session.execute(ticket)
        self.assertEqual(executor.calls, [])

    def test_explicit_execution_retains_original_failure_recovery(self):
        session, executor, _, _ = self.session(fail_once='PickLift')
        with session:
            while (ticket := session.plan()) is not None:
                session.execute(ticket)
        self.assertTrue(session.result.success)
        self.assertEqual(executor.calls, ['NavigateToPick', 'PickLift', 'PickLift'])
        self.assertEqual(session.result.metrics['num_skill_retries'], 1)

    def test_editing_returned_outcome_does_not_rewrite_recovery_evidence(self):
        session, executor, _, _ = self.session(fail_once='PickLift')
        with session:
            while (ticket := session.plan()) is not None:
                outcome = session.execute(ticket)
                if not outcome.success:
                    outcome.failure_details['recovery_preconditions'] = {
                        'allowed': False, 'blockers': ['caller-edited-inspection-copy']}
        self.assertTrue(session.result.success)
        self.assertEqual(executor.calls, ['NavigateToPick', 'PickLift', 'PickLift'])


class SceneSmithStateGuardTest(unittest.TestCase):
    def test_reset_to_identical_state_invalidates_old_observation(self):
        from simulation.src import make_env
        from simulation.src.recipes.mobile import make_config
        from planner.src.tamp.scenesmith_online import SceneSmithSkillExecutor

        env = make_env(make_config('wheel_dynamic'))
        observation, _ = env.reset(4)
        executor = object.__new__(SceneSmithSkillExecutor)
        executor.env = env
        executor.observation = observation
        executor.last_navigation_goal = None
        token = executor.planning_state_token()
        self.assertEqual(token, executor.planning_state_token())
        observation.task['external_edit'] = True
        self.assertNotEqual(token, executor.planning_state_token())
        env.reset(4)
        with self.assertRaises(StalePlanError):
            executor.planning_state_token()


if __name__ == '__main__':
    unittest.main()
