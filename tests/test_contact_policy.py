"""Tests for explicit unordered contact-pair policies."""

import unittest

from src.online_manipulation import (
    FREE_MOTION_CONTACT_POLICY,
    PairContactPolicy,
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


if __name__ == "__main__":
    unittest.main()
