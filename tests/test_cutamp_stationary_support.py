"""Focused tensor guards; actual CUDA/Drake validation remains an integration run."""

import unittest

try:
    import torch
except ImportError:
    raise unittest.SkipTest("Stationary support tensor tests require the CUDA environment")

from scripts.cutamp_stationary_support import StationarySupportWorldCost


class StationarySupportGuards(unittest.TestCase):
    def test_requires_audit(self):
        with self.assertRaisesRegex(ValueError, "passing exact"):
            StationarySupportWorldCost(None, None, {}, [])

    def test_rejects_changed_support_sphere(self):
        wrapper = object.__new__(StationarySupportWorldCost)
        wrapper.indices = torch.tensor([0])
        wrapper.expected = torch.tensor([[0., 0., 0., 0.1]])
        spheres = torch.tensor([[[[0.001, 0., 0., 0.1], [1., 0., 0., 0.1]]]])
        with self.assertRaisesRegex(ValueError, "Support spheres moved"):
            wrapper({"robot_spheres": spheres})

    def test_only_ground_support_pair_is_removed(self):
        wrapper = object.__new__(StationarySupportWorldCost)
        wrapper.indices = torch.tensor([0])
        wrapper.mask = torch.tensor([False, True])
        wrapper.expected = torch.tensor([[0., 0., 0., 0.1]])
        spheres = torch.tensor([[[[0., 0., 0., 0.1], [1., 0., 0., 0.1]]]])
        calls = []

        def other_cost(values):
            calls.append(("other", values.clone()))
            return torch.tensor([[2.]])

        def ground_cost(values):
            calls.append(("ground", values.clone()))
            return torch.tensor([[3.]])

        wrapper.other_cost, wrapper.ground_cost = other_cost, ground_cost
        wrapper.cost = lambda rollout: {
            "Collision": {"values": {"robot_to_world": torch.tensor([[8.]]),
                                     "robot_to_movables": torch.tensor([[9.]])}},
            "Motion": {"values": {"self_collision": torch.tensor([[4.]])}}}
        result = wrapper({"robot_spheres": spheres})
        self.assertEqual(result["Collision"]["values"]["robot_to_world"].item(), 5.)
        self.assertEqual(result["Collision"]["values"]["robot_to_movables"].item(), 9.)
        self.assertEqual(result["Motion"]["values"]["self_collision"].item(), 4.)
        self.assertTrue(torch.equal(calls[0][1], spheres))
        self.assertTrue(torch.equal(calls[1][1], spheres[..., 1:, :]))
        self.assertEqual(wrapper.original_robot_world.item(), 8.)


if __name__ == "__main__":
    unittest.main()
