"""Contact-policy classification contracts; not GPU/physical acceptance tests."""

import dataclasses
from types import SimpleNamespace

import unittest

from scripts.export_cutamp_geometry import stationary_target_world_check
from simulation.src.geometry.planning import CollisionPair
from simulation.src.tasks.tasks import PickLiftTask, PickLiftTaskConfig


def task():
    return PickLiftTask(PickLiftTaskConfig(
        target_observation_name="target", target_contact_body="target::body",
        gripper_contact_bodies=("robot::finger_a", "robot::finger_b"),
        support_contact_bodies=("table::body",),
        maximum_allowed_contact_penetration_m=0.0005,
        maximum_allowed_support_penetration_m=0.0035,
    ))


def audit(pairs, configured_task=None):
    """Isolate classification from the independently exercised Drake query."""
    query = SimpleNamespace(
        robot_adapter=SimpleNamespace(spec=SimpleNamespace(model_instance_name="robot")),
        configuration=lambda: (),
        collision_pairs=lambda configuration, **kwargs: tuple(pairs),
    )
    return stationary_target_world_check(query, configured_task or task())


def pair(distance, other="table::body"):
    return CollisionPair(distance, "target::body", other, "target_geom", "other_geom")


class StationaryTargetContactTest(unittest.TestCase):
    def test_only_configured_support_receives_support_penetration_budget(self):
        self.assertTrue(audit([pair(-0.000029)])["valid"])
        self.assertFalse(audit([pair(-0.0036)])["valid"])
        self.assertFalse(audit([pair(-0.000029, "wall::body")])["valid"])
        self.assertFalse(audit([pair(0.004, "wall::body")])["valid"])
        self.assertTrue(audit([pair(0.006, "wall::body")])["valid"])


    def test_distinct_support_bound_and_default_task_bound_are_preserved(self):
        configured = task()
        configured.config = dataclasses.replace(
            configured.config, maximum_allowed_support_penetration_m=None)
        result = audit([pair(-0.001)], configured)
        self.assertFalse(result["valid"])
        self.assertEqual(result["maximum_allowed_support_penetration_m"], 0.0005)


    def test_report_excludes_robot_pairs_and_rejects_unknown_task_policy(self):
        result = audit([pair(-1.0, "robot::finger_a")])
        self.assertEqual(result["pairs"], [])
        self.assertEqual(result["scope"], "stationary_target_static_world_only_not_robot_or_lift_certification")
        with self.assertRaisesRegex(TypeError, "PickLiftTask"):
            audit([], SimpleNamespace())


if __name__ == "__main__":
    unittest.main()
