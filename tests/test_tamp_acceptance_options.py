"""Backend selection must reach the simulation CLI without leaking GPU settings."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest

from scripts.validate_cutamp_online import parse_args, trial_command


class AcceptanceBackendOptionsTest(unittest.TestCase):
    def options(self, *extra):
        return parse_args(['--scene-root', '/scene', '--config', '/planner.json',
                           '--output', '/run', '--seeds', '500',
                           '--validation-ref', 'refs/validation/test', *extra])

    def command(self, args, cutamp_config=None):
        return trial_command(args, Path('/repo'), Path('/experiment.json'),
                             Path('/run'), Path('/run/seed_500'), 500, cutamp_config)

    def test_proc3s_runs_without_gpu_arguments(self):
        args = self.options()
        command = self.command(args)
        self.assertEqual(command[command.index('--geometry-backend') + 1], 'proc3s')
        self.assertEqual(command[command.index('--skill-planner') + 1], 'proc3s')
        self.assertNotIn('--cutamp-config', command)
        self.assertEqual(command[command.index('--seed') + 1], '500')

    def test_explicit_cutamp_choice_preserves_dispatch(self):
        args = self.options('--geometry-backend', 'cutamp', '--archive-dir', '/archives',
                            '--cutamp-config', '/gpu.json')
        command = self.command(args, Path('/run/cutamp_seed_500.json'))
        self.assertEqual(command[command.index('--geometry-backend') + 1], 'cutamp')
        self.assertEqual(command[command.index('--skill-planner') + 1], 'proc3s')
        self.assertEqual(command[command.index('--cutamp-config') + 1],
                         '/run/cutamp_seed_500.json')

    def test_invalid_combinations_fail_before_creating_a_run(self):
        for flags in (('--geometry-backend', 'cutamp'),
                      ('--cutamp-config', '/gpu.json'), ('--archive-dir', '/archives'),
                      ('--geometry-backend', 'proc3s', '--cutamp-config', '/gpu.json'),
                      ('--geometry-backend', 'proc3s', '--archive-dir', '/archives'),
                      ('--geometry-backend', 'sampling')):
            with self.subTest(flags=flags), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as caught:
                    self.options(*flags)
                self.assertEqual(caught.exception.code, 2)

    def test_replay_choice_is_forwarded_to_both_backends(self):
        with tempfile.TemporaryDirectory() as directory:
            transcript = Path(directory) / 'model_transcript.jsonl'
            transcript.write_text('')
            seed_dir = Path(directory) / 'seed_500'
            seed_dir.mkdir()
            (seed_dir / transcript.name).write_text('')
            for backend in ('proc3s', 'cutamp'):
                gpu = ['--archive-dir', '/archives', '--cutamp-config', '/gpu.json'] if backend == 'cutamp' else []
                for replay, value, expected in (
                        ('--model-replay', transcript, transcript),
                        ('--replay-from', Path(directory), seed_dir / transcript.name)):
                    with self.subTest(backend=backend, replay=replay):
                        args = self.options('--geometry-backend', backend, *gpu, replay, str(value))
                        command = self.command(args, Path('/gpu.json') if gpu else None)
                        self.assertEqual(command[command.index('--model-replay') + 1], str(expected))


if __name__ == '__main__':
    unittest.main()
