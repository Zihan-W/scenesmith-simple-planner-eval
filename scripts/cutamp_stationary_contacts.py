"""SceneSmith's exact stationary-target constraint alongside official GPU costs.

This deliberately restricted adapter is not an object-motion collision checker.
It replaces a constant sphere/OBB approximation only after the matching original
Drake snapshot passes its configured support-contact check. All robot collision
costs remain upstream; trajectories and lifted-object motion need later checks.
"""

import hashlib
import json
import math
from pathlib import Path

import torch


class StationaryTargetWorldCost:
    """Combine a verified constant Drake constraint with unchanged variable costs."""

    def __init__(self, official_cost, geometry, target_name):
        self.official_cost = official_cost
        world = official_cost.world
        skeleton = official_cost.plan_skeleton
        if [op.operator.name for op in skeleton] != ["MoveFree", "Pick"]:
            raise ValueError("Stationary target substitution supports only MoveFree -> Pick")
        if len(world.movables) != 1 or world.movables[0].name != target_name:
            raise ValueError("Stationary target substitution requires exactly one matching movable")
        if skeleton[1].values[0] != target_name:
            raise ValueError("Pick target differs from the audited object")
        audit = geometry.get("stationary_target_world_check")
        if not audit or audit.get("valid") is not True:
            raise ValueError("A passing original Drake stationary-target audit is required")
        if audit["geometry_evaluator"] != "Drake.ComputeSignedDistancePairwiseClosestPoints":
            raise ValueError("Unexpected stationary-target geometry evaluator")
        task = geometry["resolved_experiment"]["task"]
        payload = {"geometries": [record for record in geometry["geometries"] if not record["robot"]],
                   "task": task}
        fingerprint = hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        if audit.get("geometry_task_sha256") != fingerprint:
            raise ValueError("Geometry transforms/shapes or task changed since the Drake audit")
        if target_name != task["target_observation_name"]:
            raise ValueError("GPU target name differs from the audited observation")
        if audit["target_body"] != task["target_contact_body"]:
            raise ValueError("Contact audit target does not match the task")
        bound = task["maximum_allowed_support_penetration_m"]
        if bound is None:
            bound = task["maximum_allowed_contact_penetration_m"]
        if (audit["maximum_allowed_support_penetration_m"] != bound
                or audit["minimum_safety_clearance_m"] != 0.005
                or audit["influence_distance_m"] < 0.005
                or set(audit["support_bodies"]) != set(task["support_contact_bodies"])):
            raise ValueError("Contact audit limits differ from the existing task policy")
        for pair in audit["pairs"]:
            bodies = {pair["body_a"], pair["body_b"]}
            if audit["target_body"] not in bodies:
                raise ValueError("Audit contains a non-target pair")
            other = (bodies - {audit["target_body"]}).pop()
            support = other in task["support_contact_bodies"]
            distance = pair["distance_m"]
            required = -bound if support else 0.005
            if not math.isfinite(distance) or distance < required:
                raise ValueError("Original geometry violates the stationary target contact policy")
        # The exact distances and the GPU OBBs must describe the same source
        # assets. Reject a changed mesh instead of reusing a stale certificate.
        for record in geometry["geometries"]:
            shape = record["shape"]
            if not record["robot"] and shape["kind"] in {"mesh", "convex"}:
                if hashlib.sha256(Path(shape["path"]).read_bytes()).hexdigest() != shape["sha256"]:
                    raise ValueError(f"Audited source geometry changed: {record['name']}")
        self.target_name = target_name
        self.expected_pose = world.get_object_pose(target_name).detach().clone()
        self.audit = audit
        self.original_movable_world = None

    def __call__(self, rollout):
        """Fail if any target pose changes; otherwise evaluate all variable costs."""
        poses = rollout["obj_to_pose"]
        if set(poses) != {self.target_name}:
            raise ValueError("Rollout movable set differs from the audited snapshot")
        actual = poses[self.target_name]
        if not torch.equal(actual, self.expected_pose.expand_as(actual)):
            raise ValueError("Target pose changed; the stationary contact audit is no longer applicable")
        costs = self.official_cost(rollout)
        values = costs["Collision"]["values"]
        # Keep the approximate score for diagnostics; do not report it as the
        # exact physical penetration. The exact constraint is zero iff the
        # strictly checked, unchanged snapshot meets its existing policy.
        self.original_movable_world = values["movable_to_world"].detach().clone()
        values["movable_to_world"] = torch.zeros_like(values["movable_to_world"])
        return costs
