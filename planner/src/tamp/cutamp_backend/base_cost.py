"""Differentiable constraint preserving the observed parking domain in world XY."""

import torch


class BaseDomainCost:
    """Constrain only the optimized parking configuration, not the initial base."""

    def __init__(self, cost, halfspaces, configuration_index):
        self.cost = cost
        self.halfspaces = halfspaces
        self.configuration_index = configuration_index

    def __call__(self, rollout):
        """Add the actual convex-domain violation to official cuTAMP costs."""
        costs = self.cost(rollout)
        xy = rollout["confs"][:, self.configuration_index, :2]
        distances = xy @ self.halfspaces[:, :2].T + self.halfspaces[:, 2]
        costs["BaseDomain"] = {"type": "constraint", "values": {
            "outside_m": torch.relu(distances).sum(dim=-1)}}
        return costs
