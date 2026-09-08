"""Ensure the example composes public interfaces without selecting a robot."""

import ast
import unittest
from pathlib import Path
from unittest import mock

from examples.online_manipulation import run_online
from examples.online_manipulation.example_policies import MyPolicy


class GenericOnlineExampleTest(unittest.TestCase):
    """Check dependency injection and reuse of the artifact-producing runner."""

    def test_passes_caller_objects_and_options_without_interpretation(self):
        """A non-Zerith config reaches make_env unchanged, with no joint access."""
        config = object()
        env = object()
        policy = MyPolicy()
        results = (object(),)
        with (
            mock.patch.object(run_online, "make_env", return_value=env) as build,
            mock.patch.object(
                run_online, "run_episodes", return_value=results
            ) as episodes,
        ):
            actual = run_online.run(
                config, policy, output_root=Path("run"), seeds=(4, 8),
                max_steps=12, record_html=True, write_final_dmd=True,
            )
        self.assertIs(actual, results)
        build.assert_called_once_with(config)
        episodes.assert_called_once_with(
            env=env, policy=policy, output_root=Path("run"), seeds=(4, 8),
            max_steps=12, record_html=True, write_final_dmd=True,
        )

    def test_invalid_runner_arguments_fail_before_build(self):
        """Reject invalid episode requests before constructing simulator assets."""
        with mock.patch.object(run_online, "make_env") as build:
            for options in ({"max_steps": 0}, {"seeds": ()}):
                with self.assertRaises(ValueError):
                    run_online.run(
                        object(), MyPolicy(), output_root=Path("run"), **options
                    )
            with self.assertRaises(TypeError):
                run_online.run(object(), object(), output_root=Path("run"))
        build.assert_not_called()

    def test_cli_loads_environment_and_policy_independently(self):
        """The policy factory receives config, never a runtime or context."""
        config = object()
        policy = MyPolicy()
        env_factory = mock.Mock(return_value=config)
        policy_factory = mock.Mock(return_value=policy)
        args = [
            "run_online", "--env-factory", "user_env:make_config",
            "--policy-factory", "user_policy:make_policy",
            "--output-root", "output/example",
        ]
        with (
            mock.patch("sys.argv", args),
            mock.patch.object(
                run_online, "load_factory", side_effect=[env_factory, policy_factory]
            ),
            mock.patch.object(run_online, "run", return_value=[]) as execute,
        ):
            run_online.main()
        env_factory.assert_called_once_with()
        policy_factory.assert_called_once_with(config)
        self.assertEqual(execute.call_args.args, (config, policy))

    def test_launcher_only_imports_public_library_api(self):
        """Keep concrete setup, Drake access, and robot choices out of launcher."""
        tree = ast.parse(Path(run_online.__file__).read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                name = node.module or ""
                if name.startswith("src"):
                    self.assertEqual(name, "src.online_manipulation")
                self.assertFalse(name.startswith(("pydrake", "examples")))
            if isinstance(node, ast.Attribute):
                self.assertNotIn(node.attr, ("backend", "runtime", "context"))
        public_imports = {
            alias.name for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
            and node.module == "src.online_manipulation"
            for alias in node.names
        }
        self.assertEqual(public_imports, {
            "EnvironmentConfig", "EpisodeResult", "Policy", "make_env",
            "run_episodes",
        })


if __name__ == "__main__":
    unittest.main()
