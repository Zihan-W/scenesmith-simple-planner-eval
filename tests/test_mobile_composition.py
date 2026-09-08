"""Real fixed-runtime composition tests without expert assets (P1)."""

import dataclasses
from pathlib import Path
import unittest

from src.online_manipulation.recipes.minimal import make_config
from src.online_manipulation import HoldAction, JointDeltaAction, NullTask, make_env


class CounterTask(NullTask):
    """Expose mutable task state to detect accidental cross-environment reuse."""

    def reset(self, env, rng):
        self.count = 0
        return super().reset(env, rng)

    def evaluate(self, env):
        self.count += 1
        return super().evaluate(env)

    def observe(self, env):
        return {"count": self.count}


class CompositionTest(unittest.TestCase):
    """Exercise actual Drake reset/step and independently owned Tasks."""

    def test_two_real_environments_have_independent_tasks(self):
        config = dataclasses.replace(
            make_config(Path(__file__).resolve().parents[1]), task=CounterTask()
        )
        first, second = make_env(config), make_env(config)
        a, _ = first.reset(seed=0)
        b, _ = second.reset(seed=0)
        self.assertEqual(a.robot.q, b.robot.q)
        a, *_ = first.step(HoldAction())
        self.assertEqual(a.task["count"], 1)
        self.assertEqual(second.observation.task["count"], 0)
        b, *_ = second.step(JointDeltaAction((b.robot.joint_names[0],), (0.001,)))
        self.assertEqual(b.task["count"], 1)
        self.assertAlmostEqual(a.time_s, 0.1)
        self.assertAlmostEqual(b.time_s, 0.1)


if __name__ == "__main__":
    unittest.main()
