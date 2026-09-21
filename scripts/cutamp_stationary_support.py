"""Apply exact SceneSmith wheel/floor rules to a strictly stationary GPU base.

All spheres still collide with non-ground obstacles. Only audited, unchanged
support-link spheres use the original Drake wheel/floor constraint instead of
the GPU sphere approximation. No global floor exclusion or threshold change.
"""

import hashlib
import json
import math

import torch
from curobo.geom.types import WorldConfig
from cutamp.utils.collision import get_world_collision_cost


class StationarySupportWorldCost:
    """Replace only constant, exact-audited support/floor geometry pairs."""

    def __init__(self, cost, world, geometry, statics):
        audit = geometry.get("stationary_robot_support_check")
        if not audit or audit.get("valid") is not True:
            raise ValueError("A passing exact stationary support audit is required")
        if audit["geometry_evaluator"] != "Drake.ComputeSignedDistancePairClosestPoints":
            raise ValueError("Unexpected robot support evaluator")
        payload = {"geometries": geometry["geometries"], "support_pairs": audit["pairs"]}
        digest = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        if digest != audit["geometry_support_sha256"]:
            raise ValueError("Robot support geometry changed since the audit")
        records = {item["id"]: item for item in geometry["geometries"]}
        grounds = {key for key, item in records.items() if item["ground"]}
        if len(grounds) != 1:
            raise ValueError("Stationary support adapter currently requires exactly one ground geometry")
        covered = set()
        for pair in audit["pairs"]:
            distance, bound = pair["distance_m"], pair["maximum_allowed_penetration_m"]
            if not math.isfinite(distance) or not math.isfinite(bound) or bound < 0 or distance < -bound:
                raise ValueError("Support violates the existing exact contact allowance")
            item = records[pair["robot_geometry_id"]]
            if (not item["robot"] or pair["ground_geometry_id"] not in grounds
                    or item["body"] != pair["robot_link"]):
                raise ValueError("Invalid audited support geometry pair")
            covered.add(item["id"])
        links = {records[key]["body"] for key in covered}
        for item in records.values():
            if item["robot"] and item["body"] in links and item["id"] not in covered:
                raise ValueError("Support link has unaudited collision geometry")
        model = world.kin_model
        indices = torch.cat([model.kinematics_config.get_sphere_index_from_link_name(name)
                             for name in sorted(links)]).long()
        expected = model.get_state(world.q_init[None]).get_link_spheres()[0]
        mask = torch.ones(len(expected), dtype=torch.bool, device=expected.device)
        mask[indices] = False
        self.indices, self.mask = indices, mask
        self.expected = expected[indices].detach().clone()
        ground_names = {f"drake_{key}" for key in grounds}
        self.other_cost = get_world_collision_cost(
            WorldConfig(cuboid=[item for item in statics if item.name not in ground_names]), world.tensor_args,
            collision_activation_distance=world.collision_activation_distance)
        self.ground_cost = get_world_collision_cost(
            WorldConfig(cuboid=[item for item in statics if item.name in ground_names]), world.tensor_args,
            collision_activation_distance=world.collision_activation_distance)
        self.cost = cost
        self.original_robot_world = None

    def __call__(self, rollout):
        """Require unchanged support spheres before substituting their fixed cost."""
        spheres = rollout["robot_spheres"]
        actual = spheres[..., self.indices, :]
        if not torch.equal(actual, self.expected.expand_as(actual)):
            raise ValueError("Support spheres moved; stationary audit no longer applies")
        costs = self.cost(rollout)
        values = costs["Collision"]["values"]
        self.original_robot_world = values["robot_to_world"].detach().clone()
        values["robot_to_world"] = (self.other_cost(spheres.contiguous())
                                    + self.ground_cost(spheres[..., self.mask, :].contiguous()))
        return costs
