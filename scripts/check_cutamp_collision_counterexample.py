"""Compare retained cuRobo sphere models on one identical Drake-rejected state."""

import argparse
import copy
import hashlib
import json
from pathlib import Path

import torch

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.geom.types import Cuboid, WorldConfig
from curobo.rollout.cost.self_collision_cost import SelfCollisionCost, SelfCollisionCostConfig
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from cutamp.utils.collision import get_world_collision_cost


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe-root", type=Path, required=True)
    parser.add_argument("--robot-config", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    problem = json.loads((args.probe_root / "problem.json").read_text())
    selected = json.loads((args.probe_root / "result.json").read_text())
    if selected["selected_particle"] is None:
        raise ValueError("The retained probe has no selected particle")
    tensor_args = TensorDeviceType()
    world = WorldConfig(cuboid=[Cuboid(**record) for record in problem["statics"]])
    collision = get_world_collision_cost(world, tensor_args, collision_activation_distance=0.0)
    records = []
    for path in args.robot_config:
        config = json.loads(path.read_text())
        robot = CudaRobotModel(RobotConfig.from_dict(copy.deepcopy(config)).kinematics)
        names = selected["selected_arm_joint_names"]
        q = tensor_args.to_device([[selected["selected_arm_joint_positions"][names.index(name)]
                                   for name in robot.joint_names]])
        spheres = robot.get_state(q).get_link_spheres()[:, None].contiguous()
        self_cost = SelfCollisionCost(SelfCollisionCostConfig(
            tensor_args.to_device([1.0]), tensor_args, return_loss=True,
            self_collision_kin_config=robot.get_self_collision_config()))
        records.append({"robot_config": str(path.resolve()),
                        "robot_config_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                        "num_spheres": spheres.shape[-2],
                        "robot_world_cost": float(collision(spheres).sum()),
                        "robot_self_cost": float(self_cost(spheres).sum())})
    report = {"scope": "identical_rejected_state_collision_model_comparison_not_optimizer_acceptance",
              "probe_root": str(args.probe_root.resolve()), "selected_particle": selected["selected_particle"],
              "physical_rollout_executed": False, "models": records}
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
