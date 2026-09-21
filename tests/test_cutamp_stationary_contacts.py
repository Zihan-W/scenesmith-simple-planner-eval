"""Fail-closed snapshot-adapter contracts, not physical/GPU acceptance tests."""

import copy
import hashlib
import json
from types import SimpleNamespace
import unittest

try:
    import torch
except ModuleNotFoundError as error:
    if error.name != "torch":
        raise
    raise unittest.SkipTest("cuTAMP contact adapter tests require the isolated torch environment") from error

from scripts.cutamp_stationary_contacts import StationaryTargetWorldCost


class OfficialCostFixture:
    """Minimal callable for testing which already-computed costs are retained."""

    def __init__(self):
        self.world = SimpleNamespace(
            movables=[SimpleNamespace(name="target")], get_object_pose=lambda name: torch.eye(4))
        self.plan_skeleton = [
            SimpleNamespace(operator=SimpleNamespace(name="MoveFree"), values=("q0", "t0", "q1")),
            SimpleNamespace(operator=SimpleNamespace(name="Pick"), values=("target", "g0", "q1"))]
        self.costs = {"Collision": {"values": {
            "movable_to_world": torch.tensor([[0.08]]),
            "robot_to_world": torch.tensor([[0.2]]),
            "robot_to_movables": torch.tensor([[0.3]])}},
            "Motion": {"values": {"self_collision": torch.tensor([[0.4]])}}}

    def __call__(self, rollout):
        return copy.deepcopy(self.costs)


def geometry():
    task = {"target_observation_name": "target", "target_contact_body": "box::body",
            "support_contact_bodies": ["table::body"],
            "maximum_allowed_support_penetration_m": 0.0035,
            "maximum_allowed_contact_penetration_m": 0.0005}
    result = {"geometries": [], "resolved_experiment": {"task": task},
              "stationary_target_world_check": {
                  "valid": True, "geometry_evaluator": "Drake.ComputeSignedDistancePairwiseClosestPoints",
                  "target_body": "box::body", "support_bodies": ["table::body"],
                  "maximum_allowed_support_penetration_m": 0.0035,
                  "minimum_safety_clearance_m": 0.005, "influence_distance_m": 0.05,
                  "pairs": [{"body_a": "box::body", "body_b": "table::body", "distance_m": -0.000029}]}}
    result["stationary_target_world_check"]["geometry_task_sha256"] = hashlib.sha256(
        json.dumps({"geometries": [], "task": task}, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    return result


class StationaryContactCostTest(unittest.TestCase):
    def test_all_robot_costs_remain_unchanged(self):
        official = OfficialCostFixture()
        cost = StationaryTargetWorldCost(official, geometry(), "target")
        actual = cost({"obj_to_pose": {"target": torch.eye(4)[None, None]}})
        self.assertEqual(float(actual["Collision"]["values"]["movable_to_world"].sum()), 0.0)
        for key in ("robot_to_world", "robot_to_movables"):
            self.assertTrue(torch.equal(actual["Collision"]["values"][key], official.costs["Collision"]["values"][key]))
        self.assertTrue(torch.equal(actual["Motion"]["values"]["self_collision"],
                                    official.costs["Motion"]["values"]["self_collision"]))
        self.assertTrue(torch.equal(cost.original_movable_world,
                                    official.costs["Collision"]["values"]["movable_to_world"]))

    def test_changed_pose_or_movable_set_rejected(self):
        cost = StationaryTargetWorldCost(OfficialCostFixture(), geometry(), "target")
        moved = torch.eye(4)[None, None]
        moved[0, 0, 2, 3] = 1e-5
        with self.assertRaisesRegex(ValueError, "pose changed"):
            cost({"obj_to_pose": {"target": moved}})
        with self.assertRaisesRegex(ValueError, "movable set"):
            cost({"obj_to_pose": {"other": moved}})

    def test_stale_task_or_geometry_audit_rejected(self):
        stale = geometry()
        stale["resolved_experiment"]["task"]["maximum_allowed_support_penetration_m"] = 1.0
        with self.assertRaisesRegex(ValueError, "changed since"):
            StationaryTargetWorldCost(OfficialCostFixture(), stale, "target")
        stale = geometry()
        stale["geometries"].append({"robot": False, "shape": {"kind": "box"}})
        with self.assertRaisesRegex(ValueError, "changed since"):
            StationaryTargetWorldCost(OfficialCostFixture(), stale, "target")

    def test_invalid_distance_not_overridden_by_valid_flag(self):
        invalid = geometry()
        invalid["stationary_target_world_check"]["pairs"][0]["distance_m"] = -0.004
        with self.assertRaisesRegex(ValueError, "violates"):
            StationaryTargetWorldCost(OfficialCostFixture(), invalid, "target")

    def test_object_motion_skeleton_rejected(self):
        official = OfficialCostFixture()
        official.plan_skeleton[1].operator.name = "Place"
        with self.assertRaisesRegex(ValueError, "only MoveFree"):
            StationaryTargetWorldCost(official, geometry(), "target")


if __name__ == "__main__":
    unittest.main()
