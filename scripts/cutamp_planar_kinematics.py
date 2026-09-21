"""Differentiable world-XY base translation around the actual cuRobo arm model.

The optimization coordinates are [base_x_world_m, base_y_world_m, arm_q...].
Base height and heading are fixed at the supplied anchor, matching the current
SceneSmith constant-heading geometric domain. These first two coordinates are
NOT wheel joints or velocity commands. Navigation corridor/dynamics checks
remain separate and mandatory before execution.
"""

import dataclasses

import torch


class PlanarTranslationKinematics:
    """Expose rigid base translation to cuTAMP's configuration gradients."""

    def __init__(self, arm_model, world_from_anchor, base_xy_bounds):
        if world_from_anchor.shape != (4, 4) or base_xy_bounds.shape != (2, 2):
            raise ValueError("Expected a 4x4 anchor and 2x2 lower/upper XY bounds")
        if not torch.isfinite(world_from_anchor).all() or not torch.isfinite(base_xy_bounds).all():
            raise ValueError("Anchor and optimization bounds must be finite")
        if not torch.all(base_xy_bounds[0] < base_xy_bounds[1]):
            raise ValueError("Base bounds must have positive extent")
        rotation = world_from_anchor[:3, :3]
        identity = torch.eye(3, device=rotation.device, dtype=rotation.dtype)
        if not torch.allclose(rotation.T @ rotation, identity, atol=1e-6, rtol=0):
            raise ValueError("The anchor must contain a rigid rotation")
        if not torch.allclose(rotation[:, 2], identity[:, 2], atol=1e-6, rtol=0):
            raise ValueError("Planar base optimization requires an upright anchor")
        self.arm_model = arm_model
        self.world_from_anchor = world_from_anchor.detach().clone()
        self.joint_names = ["base_x_world_m", "base_y_world_m", *arm_model.joint_names]
        self.joint_limits = torch.cat((base_xy_bounds,
                                      arm_model.kinematics_config.joint_limits.position), dim=1)

    def get_self_collision_config(self):
        """Rigid base translation leaves the actual self-collision model intact."""
        return self.arm_model.get_self_collision_config()

    def get_state(self, configuration):
        """Run actual arm CUDA FK, then translate all outputs in the anchor frame."""
        if configuration.ndim != 2 or configuration.shape[1] != len(self.joint_names):
            raise ValueError("Expected a batch of world-XY plus arm configurations")
        state = self.arm_model.get_state(configuration[:, 2:].contiguous())
        world_delta = torch.cat((configuration[:, :2] - self.world_from_anchor[:2, 3],
                                 torch.zeros_like(configuration[:, :1])), dim=-1)
        # Rows transform by R; equivalently column vectors use R.transpose().
        anchor_delta = world_delta @ self.world_from_anchor[:3, :3]
        spheres = state.get_link_spheres()
        translated_spheres = torch.cat((spheres[..., :3] + anchor_delta[:, None, :],
                                        spheres[..., 3:]), dim=-1)
        return dataclasses.replace(
            state, ee_position=state.ee_position + anchor_delta,
            links_position=(state.links_position + anchor_delta[:, None, :]
                            if state.links_position is not None else None),
            link_spheres_tensor=translated_spheres)
