"""Stable contracts for shared assembly, snapshots, capabilities and packaging."""
import ast
import dataclasses
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

from planner.src.skills.driver import SkillDriver
from planner.src.skills.runtime import SingleSkillPolicy
from planner.src.bt.runtime import JsonBtPolicy, _certified_local_map
from planner.src.skills.navigation import certified_local_map
from planner.src.tamp.failures import ProgramFailure
from planner.src.tamp.hierarchy import SkillSpec, SkillRegistry, PredicateGoal, WorldState
from planner.src.tamp.proc3s import parse_proc3s_program, PROC3S_SCHEMA
from planner.src.tamp.ccsp import Proc3sCCSPSolver
from planner.src.tamp.cutamp import CuTAMPSolver
from planner.src.tamp.cutamp_backend.integrity import verify_installation, gpu_environment, ARTIFACT_FIELDS


class ArchitectureContractsTest(unittest.TestCase):
    def test_runtime_implementations_are_shared(self):
        self.assertIs(_certified_local_map, certified_local_map)
        for name in ('record_action_result', '_observed_gripper_width', '_clear_closure'):
            self.assertIs(getattr(JsonBtPolicy, name), getattr(SkillDriver, name))
            self.assertIs(getattr(SingleSkillPolicy, name), getattr(SkillDriver, name))
        self.assertIs(SingleSkillPolicy._picklift, SkillDriver._picklift)

    def test_library_has_no_scripts_dependency(self):
        root = Path(__file__).resolve().parents[1]
        for path in (root/'planner/src').rglob('*.py'):
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.ImportFrom):
                    self.assertFalse((node.module or '').startswith('scripts'), str(path))
                elif isinstance(node, ast.Import):
                    self.assertFalse(any(n.name.startswith('scripts') for n in node.names), str(path))

    def test_failure_scope_and_backend_capabilities(self):
        with self.assertRaisesRegex(ValueError, 'nonempty skill'):
            ProgramFailure('', ('other',), (), attribution_scope='skill')
        failure = ProgramFailure.from_feedback({'failed_constraints':['other']})
        self.assertEqual(failure.attribution_scope, 'global')
        self.assertFalse(Proc3sCCSPSolver.capabilities.retry_after_search_failure)
        self.assertTrue(CuTAMPSolver.capabilities.retry_after_search_failure)
        self.assertEqual(CuTAMPSolver.capabilities.failure_context_use, 'diagnostics_only')

    def test_new_skill_sampler_and_predicates_come_from_registry(self):
        registry = SkillRegistry((SkillSpec('Inspect', ('object',), ('view',),
            (PredicateGoal('observed', ('$object',)),),
            (PredicateGoal('inspected', ('$object',)),),
            domain_samplers={'view':'inspection_views'}),))
        world = WorldState({'box':{}}, frozenset({PredicateGoal('observed',('box',))}))
        document = {'schema':PROC3S_SCHEMA,'steps':[{'skill':'Inspect','arguments':{'object':'box'},
            'continuous_variables':{'view':'$v'}}], 'domains':[{'variable':'v','sampler':'inspection_views'}]}
        program = parse_proc3s_program(json.dumps(document), registry=registry, world=world,
            goals=(PredicateGoal('inspected',('box',)),))
        self.assertEqual(program.parameter_domains, {'v':'inspection_views'})
        self.assertEqual(registry.predicate_arity, {'observed':1,'inspected':1})
        class InspectionGeometry:
            def sample_candidate(self, skill, state, rng):
                return {"view": 1.0}
            def parameter_control_keys(self, parameter):
                return (parameter,)
            def check(self, skill, candidate, state):
                return True, "valid", {}
            def predict(self, skill, candidate, state):
                return state
        plan = Proc3sCCSPSolver(registry, InspectionGeometry(), max_samples=1).solve(
            world, program, {})
        self.assertEqual(plan.assignments, {"v":1.0})
        self.assertEqual(plan.actions[0].skill_name, "Inspect")

    def test_config_and_optimizer_drift_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); (root/'cutamp').mkdir()
            source=root/'cutamp/optimizer.py'; source.write_text('VERSION = 1\n')
            values={name:str(root/name) for name in ARTIFACT_FIELDS}
            for name in values: Path(values[name]).write_text('{}')
            manifest={'schema':'scenesmith.cutamp.integrity.v1',
                'artifacts':{k:hashlib.sha256(Path(v).read_bytes()).hexdigest() for k,v in values.items()},
                'optimizer_sources':{'cutamp/optimizer.py':hashlib.sha256(source.read_bytes()).hexdigest()},
                'gpu_environment':gpu_environment(sys.executable)}
            lock=root/'integrity.json'; lock.write_text(json.dumps(manifest))
            settings=SimpleNamespace(**values,cutamp_root=str(root),gpu_python=sys.executable,
                                     integrity_manifest=str(lock))
            verify_installation(settings)
            Path(values['tolerances']).write_text('{"changed":true}')
            with self.assertRaisesRegex(ValueError,'artifact hash mismatch: tolerances'):
                verify_installation(settings)
            Path(values['tolerances']).write_text('{}'); source.write_text('VERSION = 2\n')
            with self.assertRaisesRegex(ValueError,'optimizer source drift'):
                verify_installation(settings)

if __name__=='__main__':
    unittest.main()
