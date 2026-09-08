"""Ensure the example composes public interfaces without selecting a robot."""

import ast
import unittest
from pathlib import Path
from unittest import mock

from src.online_manipulation import assembly as run_online
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
            mock.patch("src.online_manipulation.assembly.make_env", return_value=env) as build,
            mock.patch(
                "src.online_manipulation.assembly.run_episodes", return_value=results
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
        with mock.patch("src.online_manipulation.assembly.make_env") as build:
            for options in ({"max_steps": 0}, {"seeds": ()}):
                with self.assertRaises(ValueError):
                    run_online.run(
                        object(), MyPolicy(), output_root=Path("run"), **options
                    )
            with self.assertRaises(TypeError):
                run_online.run(object(), object(), output_root=Path("run"))
        build.assert_not_called()




if __name__ == "__main__":
    unittest.main()
