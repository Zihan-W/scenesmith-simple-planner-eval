"""Deterministic PRoC3S feedback contracts; physical acceptance is separate."""
import dataclasses
import json
import random
import unittest

from planner.src.bt.generation import ChatCompletion
from planner.src.tamp.ccsp import Proc3sCCSPSolver
from planner.src.tamp.hierarchy import PredicateGoal, WorldState, picklift_registry
from planner.src.tamp.online import IncrementalTampRunner, RecoveryLimits, SkillExecution
from planner.src.tamp.proc3s import PRoC3SProgramGenerator
from planner.src.tamp.semantic import ModelSettings


def document(skills):
    steps, domains = [], []
    for i, skill in enumerate(skills):
        parameters = {'base_pose': f'b{i}'} if skill == 'NavigateToPick' else {
            'grasp_pose': f'g{i}', 'approach_pose': f'a{i}'}
        samplers = {'base_pose': 'scene_base_pose', 'grasp_pose': 'calibrated_grasp_pose',
                    'approach_pose': 'calibrated_approach_pose'}
        steps.append({'skill': skill, 'arguments': {'object': 'cube'},
                      'continuous_variables': {k: '$'+v for k,v in parameters.items()}})
        domains.extend({'variable': v, 'sampler': samplers[k]} for k,v in parameters.items())
    return json.dumps({'schema': 'scenesmith.proc3s.program.v1', 'steps': steps, 'domains': domains})


class FeedbackClosedLoopTest(unittest.TestCase):
    def test_repair_then_verified_reposition_does_not_reuse_old_failure(self):
        registry = picklift_registry()
        holding = PredicateGoal('holding', ('cube',))
        initial = WorldState({'cube': {}}, frozenset({PredicateGoal('observed', ('cube',)),
            PredicateGoal('gripper_empty', ()), PredicateGoal('at_pick_pose', ('cube',))}),
            observation_id='0', robot={'base_x': 0.})
        requests, executions, events = [], [], []

        class Client:
            def complete(self, **kwargs):
                payload = json.loads(kwargs['messages'][1]['content'])
                requests.append(payload)
                sequence = [['PickLift'], ['NavigateToPick', 'PickLift'], ['PickLift']]
                return ChatCompletion(document(sequence[len(requests)-1]), 'contract-model', 'contract-response')

        class Semantic:
            calls = 0
            def propose(self, **kwargs):
                self.calls += 1
                return (holding,)

        class Domain:
            def sample_candidate(self, skill, state, rng):
                return {'base_x_m': 1., 'base_y_m': 0., 'base_yaw_rad': 0.} if skill.skill == 'NavigateToPick' else {
                    'grasp_lateral_offset_m': 0., 'approach_height_offset_m': 0.}
            def parameter_control_keys(self, parameter):
                return {'base_pose': ('base_x_m', 'base_y_m', 'base_yaw_rad'), 'grasp_pose': ('grasp_lateral_offset_m',),
                        'approach_pose': ('approach_height_offset_m',)}[parameter]
            def predict(self, skill, candidate, state):
                if skill.skill == 'NavigateToPick': state['base_x'] = candidate['base_x_m']
                return state
            def check(self, skill, candidate, state):
                ok = skill.skill == 'NavigateToPick' or state['base_x'] == 1.
                return ok, 'valid' if ok else 'ik', {'raw_joint': 123.456}

        class Observer:
            def observe(self, observation): return observation
            def geometry_state(self, observation, previous): return dict(observation.robot)

        class Executor:
            world = initial
            def execute(self, action):
                executions.append(action.skill_name)
                facts = self.world.facts | set(action.expected_effects)
                self.world = dataclasses.replace(self.world, facts=frozenset(facts),
                    observation_id=str(len(executions)), robot={'base_x': 1.})
                return SkillExecution(self.world, True, 'observed_effect')

        generator = PRoC3SProgramGenerator(Client(), ModelSettings('contract-model', max_attempts=1), registry, trace=events.append)
        runner = IncrementalTampRunner(semantic=Semantic(), registry=registry,
            solver_factory=lambda w: Proc3sCCSPSolver(registry, Domain(), max_samples=2, rng=random.Random(1), trace=events.append),
            executor=Executor(), observer=Observer(), trace=events.append,
            limits=RecoveryLimits(semantic_replans=0, skill_replans=1, geometry_retries=0, max_skill_executions=4),
            program_generator=generator)
        result = runner.run(task='lift cube', task_goals=(holding,), initial_observation=initial,
            initial_geometry_state={'base_x': 0.}, predicate_arity={'holding': 1})
        self.assertTrue(result.success)
        self.assertEqual(executions, ['NavigateToPick', 'PickLift'])
        self.assertEqual(result.metrics['num_skill_replans'], 1)
        self.assertEqual(requests[1]['constraint_feedback'][0]['failed_constraints'], ['ik'])
        self.assertTrue(requests[1]['constraint_feedback'][0]['search_budget_exhausted'])
        # Finite failures allow an approved bounded subdomain revision.
        self.assertEqual(requests[1]['excluded_skeletons'], [])
        self.assertIsNotNone(requests[1]['previous_program'])
        self.assertNotIn('123.456', json.dumps(requests))
        self.assertEqual(requests[2]['constraint_feedback'], [])
        self.assertEqual(requests[2]['excluded_skeletons'], [])
        self.assertIsNone(requests[2]['previous_program'])


if __name__ == '__main__':
    unittest.main()
