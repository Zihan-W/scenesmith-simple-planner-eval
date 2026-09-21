"""Common comparison controls must preserve baseline defaults."""

import unittest

from examples.online_manipulation.tamp_cli import _execution_options


class ComparisonOptionsTest(unittest.TestCase):
    def test_explicit_seed_overrides_bt_episode_without_mutating_experiment(self):
        options = {"seeds": (10, 20), "max_steps": 123}
        self.assertEqual(_execution_options(options, 7), {"seeds": (7,), "max_steps": 123})
        self.assertEqual(options["seeds"], (10, 20))

    def test_absent_seed_retains_all_original_baseline_episodes(self):
        options = {"seeds": (10, 20), "max_steps": 123}
        self.assertEqual(_execution_options(options, None), options)


if __name__ == "__main__":
    unittest.main()
