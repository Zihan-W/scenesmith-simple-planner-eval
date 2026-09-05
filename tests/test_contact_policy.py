"""Tests for explicit unordered contact-pair policies."""

import unittest

from src.online_manipulation import (
    CarriedBody,
    FREE_MOTION_CONTACT_POLICY,
    PairContactPolicy,
    Pose,
)


class PairContactPolicyTest(unittest.TestCase):
    """Validate default rejection and canonical pair matching."""

    def test_free_motion_rejects_every_contact(self) -> None:
        self.assertFalse(FREE_MOTION_CONTACT_POLICY.permits("a", "b"))

    def test_explicit_pair_is_unordered(self) -> None:
        policy = PairContactPolicy.from_pairs(
            "grasp",
            [("robot::finger", "scene::object")],
        )
        self.assertTrue(
            policy.permits("scene::object", "robot::finger")
        )
        self.assertFalse(policy.permits("robot::wrist", "scene::object"))

    def test_carried_body_is_monitored_with_bounded_contact(self) -> None:
        pose = Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        carried = CarriedBody(
            body_name="scene::object",
            carrier_frame_name="tool",
            body_pose_world=pose,
            carrier_pose_world=pose,
        )
        policy = PairContactPolicy.from_pairs(
            "carrying",
            [("robot::finger", "scene::object")],
            carried_bodies=(carried,),
            maximum_allowed_penetration_m=1e-5,
        )
        self.assertEqual(policy.carried_bodies, (carried,))
        self.assertEqual(policy.monitored_bodies, {"scene::object"})
        self.assertEqual(policy.maximum_allowed_penetration_m, 1e-5)


if __name__ == "__main__":
    unittest.main()
