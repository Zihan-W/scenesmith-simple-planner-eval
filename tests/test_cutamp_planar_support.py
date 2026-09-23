"""Reject unsupported changes to exact planar support substitution."""
import unittest

try:
    import torch
except ImportError:
    raise unittest.SkipTest("Planar support checks require the isolated torch environment")

from scripts.cutamp_stationary_support import PlanarSupportWorldCost


class PlanarSupportTests(unittest.TestCase):
    def test_requires_exact_domain_audit(self):
        with self.assertRaisesRegex(ValueError, "planar support audit"):
            PlanarSupportWorldCost(None, None, {}, [])

    def test_accepts_only_rigid_translation_of_support(self):
        cost = object.__new__(PlanarSupportWorldCost)
        cost.anchor = torch.eye(4)
        cost.indices = torch.tensor([0])
        cost.expected = torch.tensor([[0.0, 0.0, 0.1, 0.2]])
        rollout = {'confs': torch.tensor([[[0.3, 0.4, 0.0]]])}
        spheres = torch.tensor([[[[0.3, 0.4, 0.1, 0.2]]]])
        cost.validate_support(rollout, spheres)
        shifted = spheres.clone()
        shifted[..., 2] += 0.001
        with self.assertRaisesRegex(ValueError, "rigid planar translation"):
            cost.validate_support(rollout, shifted)
        enlarged = spheres.clone()
        enlarged[..., 3] += 0.001
        with self.assertRaisesRegex(ValueError, "rigid planar translation"):
            cost.validate_support(rollout, enlarged)


if __name__ == '__main__':
    unittest.main()
