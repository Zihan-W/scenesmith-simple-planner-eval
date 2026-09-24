"""The installed TAMP package is callable outside the repository."""

import os
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from planner.src.tamp import TampRunRequest


class TampPackageEntryTest(unittest.TestCase):
    def invoke(self, *arguments):
        environment = dict(os.environ)
        for name in ('PYTHONPATH', 'OPENAI_API_KEY', 'OPENAI_BASE_URL',
                     'CUTAMP_ROOT', 'CUTAMP_PYTHON'):
            environment.pop(name, None)
        with tempfile.TemporaryDirectory() as directory:
            return subprocess.run(
                [sys.executable, '-m', 'planner.src.tamp', *arguments],
                cwd=directory, env=environment, capture_output=True, text=True)

    def test_package_help_exposes_both_geometry_backends(self):
        completed = self.invoke('--help')
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn('--geometry-backend {proc3s,cutamp}', completed.stdout)
        self.assertIn('--model-replay', completed.stdout)

    def test_invalid_backend_preserves_cli_failure_exit_status(self):
        for backend in ('unknown', 'sampling'):
            with self.subTest(backend=backend):
                completed = self.invoke('--geometry-backend', backend, '--output-root', 'unused')
                self.assertEqual(completed.returncode, 2)
                self.assertIn('invalid choice', completed.stderr)

    def test_cli_omitted_selectors_record_proc3s_before_scene_loading(self):
        environment = dict(os.environ)
        environment.pop('PYTHONPATH', None)
        repo = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / 'planner.json'
            config.write_text('{"recovery": {}}')
            completed = subprocess.run([
                sys.executable, '-m', 'planner.src.tamp',
                '--repository-root', str(repo), '--scene-root', str(root),
                '--experiment', str(root / 'intentionally-absent.json'),
                '--config', str(config), '--task', 'pick',
                '--output-root', str(root / 'output'), '--max-wall-time-s', '30',
            ], cwd=root, env=environment, capture_output=True, text=True)
            self.assertNotEqual(completed.returncode, 0)
            recorded = json.loads((root / 'output/planner_config.json').read_text())
            self.assertEqual(recorded['skill_planner'], 'proc3s')
            self.assertEqual(recorded['geometry_backend'], 'proc3s')
            self.assertIsNone(recorded['cutamp'])
            self.assertIn('intentionally-absent.json', completed.stderr)

    def test_import_is_lightweight_outside_repository(self):
        environment = dict(os.environ)
        environment.pop('PYTHONPATH', None)
        with tempfile.TemporaryDirectory() as directory:
            completed = subprocess.run([
                sys.executable, '-c',
                'import sys; from planner.src.tamp import TampRunRequest, run; '
                'assert callable(run); '
                'assert not any(n == "torch" or n.startswith("pydrake") for n in sys.modules)',
            ], cwd=directory, env=environment, capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_python_api_keeps_watchdog_and_rejects_output_reuse(self):
        environment = dict(os.environ)
        for name in ('PYTHONPATH', 'OPENAI_API_KEY', 'OPENAI_BASE_URL'):
            environment.pop(name, None)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / 'planner.json'
            config.write_text('{"recovery": {}}')
            code = '''
import json
from pathlib import Path
from planner.src.tamp import TampRunRequest, run
root = Path.cwd()
request = TampRunRequest(repository_root=root, scene_root=root,
    experiment=root / 'unused.json', task='pick', output_root=root / 'output',
    config=root / 'planner.json', max_wall_time_s=0.001,
    skill_planner='proc3s', geometry_backend='proc3s')
result = run(request)
assert result == json.loads((root / 'output/result.json').read_text())
assert result['success'] is False
assert result['reason'] == 'wall_time_budget_exhausted'
assert result['supervision']['timed_out'] is True
before = (root / 'output/result.json').read_bytes()
try:
    run(request)
except FileExistsError:
    pass
else:
    raise AssertionError('Existing result was not protected')
assert (root / 'output/result.json').read_bytes() == before
'''
            completed = subprocess.run([sys.executable, '-c', code], cwd=root,
                env=environment, capture_output=True, text=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertFalse((root / 'output/skill_steps.jsonl').exists())


class TampRequestTest(unittest.TestCase):
    def request(self, **options):
        return TampRunRequest(repository_root=Path('/repo'), scene_root=Path('/scene'),
                              experiment=Path('/experiment.json'), task='pick',
                              output_root=Path('/output'), **options)

    def test_defaults_and_explicit_backends_preserve_cli_selection(self):
        default = self.request()
        self.assertEqual((default.skill_planner, default.geometry_backend), ('proc3s', 'proc3s'))
        baseline = self.request(skill_planner='strips', geometry_backend='proc3s')
        self.assertEqual((baseline.skill_planner, baseline.geometry_backend), ('strips', 'proc3s'))
        for backend in ('proc3s', 'cutamp'):
            options = {'cutamp_config': Path('/gpu.json')} if backend == 'cutamp' else {}
            request = self.request(skill_planner='proc3s', geometry_backend=backend,
                                   seed=0, record_html=True, **options)
            arguments = request.cli_arguments()
            self.assertEqual(arguments[arguments.index('--geometry-backend') + 1], backend)
            self.assertEqual(arguments[arguments.index('--seed') + 1], '0')
            self.assertIn('--record-html', arguments)
            self.assertEqual('--cutamp-config' in arguments, backend == 'cutamp')

    def test_incompatible_inputs_fail_before_starting_simulation(self):
        for options in ({'geometry_backend': 'cutamp'},
                        {'cutamp_config': Path('/gpu.json')},
                        {'geometry_backend': 'unknown'}, {'geometry_backend': 'sampling'},
                        {'skill_planner': 'unknown'},
                        {'model_replay': Path('/record'), 'recorded_subgoals': Path('/goals')}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.request(**options)


if __name__ == '__main__':
    unittest.main()
