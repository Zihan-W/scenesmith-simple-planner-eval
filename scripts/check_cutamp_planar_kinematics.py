"""Check the actual CUDA FK translation adapter and its world-XY derivatives."""

import argparse
import copy
import json
from pathlib import Path

import torch

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.robot import RobotConfig
from cutamp_planar_kinematics import PlanarTranslationKinematics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", type=Path, required=True)
    parser.add_argument("--problem", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    config = json.loads(args.robot_config.read_text())
    problem = json.loads(args.problem.read_text())
    original = CudaRobotModel(RobotConfig.from_dict(copy.deepcopy(config)).kinematics)
    anchor = torch.tensor(problem["fixed_base_world_matrix"], device="cuda", dtype=torch.float32)
    bounds = torch.tensor(problem["base_xy_bounds_world_m"], device="cuda", dtype=torch.float32)
    mobile = PlanarTranslationKinematics(original, anchor, bounds)
    arm = torch.zeros((1, len(original.joint_names)), device="cuda")
    state = original.get_state(arm)
    original_ee = state.ee_position.detach().clone()
    original_spheres = state.get_link_spheres().detach().clone()
    errors = []
    for delta_xy in ((0.0, 0.0), (0.01, 0.0), (0.0, -0.01), (0.013, -0.007)):
        delta = torch.tensor([[*delta_xy, 0.0]], device="cuda")
        config_xy = anchor[None, :2, 3] + delta[:, :2]
        state = mobile.get_state(torch.cat((config_xy, arm), dim=-1))
        shift = delta @ anchor[:3, :3]
        errors.append(float((state.ee_position - (original_ee + shift)).abs().max()))
        errors.append(float((state.get_link_spheres()[..., :3]
                             - (original_spheres[..., :3] + shift[:, None])).abs().max()))
        if not torch.equal(state.get_link_spheres()[..., 3], original_spheres[..., 3]):
            raise AssertionError("Base translation changed collision sphere radii")
    q = torch.cat((anchor[None, :2, 3], arm), dim=-1).clone().requires_grad_(True)
    direction = torch.tensor([0.7, -0.2, 0.4], device="cuda")
    loss = (mobile.get_state(q).ee_position * direction).sum()
    loss.backward()
    actual_gradient = q.grad[0, :2].detach().clone()
    expected_gradient = anchor[:2, :3] @ direction
    gradient_error = float((actual_gradient - expected_gradient).abs().max())
    finite_differences = []
    for axis in (0, 1):
        values = []
        for sign in (-1, 1):
            changed = q.detach().clone()
            changed[0, axis] += sign * 0.001
            values.append(float((mobile.get_state(changed).ee_position * direction).sum()))
        finite_differences.append((values[1] - values[0]) / 0.002)
    fd_error = float((torch.tensor(finite_differences, device="cuda") - actual_gradient).abs().max())
    report = {"scope": "actual_cuda_fk_and_base_gradient_not_navigation_or_physics",
              "maximum_translation_error_m": max(errors), "base_gradient_error": gradient_error,
              "base_gradient_finite_difference_error": fd_error,
              "base_gradient": actual_gradient.cpu().tolist(),
              "passed": max(errors) < 1e-6 and gradient_error < 1e-6 and fd_error < 1e-4}
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))
    if not report["passed"]:
        raise AssertionError("Mobile translation or gradient comparison failed")


if __name__ == "__main__":
    main()
