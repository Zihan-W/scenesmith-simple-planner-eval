"""Installed entry points and retained optional-baseline contracts."""

import ast
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from src.online_manipulation import load_experiment
from tools.iiwa.__main__ import baseline_commands

ROOT = Path(__file__).resolve().parents[1]


class DistributionTest(unittest.TestCase):
    """Exercise installation rather than relying on repository cwd/PYTHONPATH."""

    def test_installed_cli_and_public_api_outside_repository(self):
        environment = dict(os.environ)
        environment.pop("PYTHONPATH", None)
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            result = subprocess.run([
                str(Path(sys.executable).parent / "scene-eval"),
                str(ROOT / "experiments/minimal.json"), "--repository-root", str(ROOT),
                "--cache-root", str(run / "cache"), "--output-root", str(run / "output"),
            ], cwd=run, env=environment, capture_output=True, text=True, check=True)
            self.assertIn('"policy_steps": 10', result.stdout)
            self.assertTrue((run / "output/resolved_config.json").is_file())
            result = subprocess.run([
                sys.executable, "-B", str(ROOT / "examples/online_manipulation/public_api_client.py"),
                "--repository-root", str(ROOT),
            ], cwd=run, env=environment, capture_output=True, text=True, check=True)
            self.assertAlmostEqual(json.loads(result.stdout)["simulation_time_s"], 0.1)

    def test_retired_imports_and_bootstrap_do_not_return(self):
        retired = ("tools.legacy", "src.rrt", "src.shortcut", "src.zerith_online_env",
                   "src.online_manipulation.recipes.pick_environment",
                   "examples.online_manipulation.minimal_setup")
        for folder in ("src", "tools", "examples", "scripts"):
            for path in (ROOT / folder).rglob("*.py"):
                text = path.read_text()
                self.assertNotIn("sys.path.insert", text, path)
                for node in ast.walk(ast.parse(text)):
                    modules = ([node.module or ""] if isinstance(node, ast.ImportFrom)
                               else [a.name for a in node.names] if isinstance(node, ast.Import) else [])
                    for module in modules:
                        self.assertFalse(module.startswith(retired), (path, module))

    def test_configuration_errors_are_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text('{}')
            with self.assertRaisesRegex(ValueError, "Missing experiment profiles"):
                load_experiment(path, repository_root=ROOT, cache_root=Path(directory)/"cache")
            data = json.loads((ROOT / "experiments/minimal.json").read_text())
            data["run"]["misspelled_option"] = 1
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "Unknown run options"):
                load_experiment(path, repository_root=ROOT, cache_root=Path(directory)/"cache")

    def test_iiwa_pipeline_is_explicit_and_does_not_delete_previous_runs(self):
        commands = baseline_commands(Path('/input'), ROOT, Path('/results'))
        self.assertEqual(len(commands), 4)
        self.assertTrue(all(c[:3] == [sys.executable, '-B', '-m'] for c in commands))
        self.assertIn('--gripper-clearance', commands[2])
        self.assertIn('/results/final.dmd.yaml', commands[3])
        from tools.iiwa import __main__ as baseline
        with tempfile.TemporaryDirectory() as directory:
            args = ['iiwa-baseline', '--scene-root', directory, '--repository-root', str(ROOT),
                    '--output-root', directory]
            with mock.patch('sys.argv', args), mock.patch.object(baseline.subprocess, 'run') as execute:
                with self.assertRaisesRegex(FileNotFoundError, 'IIWA baseline requires'):
                    baseline.main()
            execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
