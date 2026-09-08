"""Description-driven robot adapter with named groups and optional grippers."""

import numpy as np
from pydrake.all import Quaternion, RigidTransform

from src.online_manipulation.observations import Pose, RobotObservation, SpatialVelocity


def public_pose(transform):
    """Convert a Drake transform to the public world pose representation."""
    return Pose(
        tuple(transform.translation()),
        tuple(transform.rotation().ToQuaternion().wxyz()),
    )


def drake_pose(pose):
    """Convert a public pose, normalizing its nonzero quaternion."""
    q = np.asarray(pose.quaternion_wxyz)
    return RigidTransform(Quaternion(q / np.linalg.norm(q)), pose.translation_m)


class DescriptionRobotAdapter:
    """Implement the runtime contract from a robot description, not dimensions."""

    # A11 fixed baseline only. Mobile adapters must override model construction;
    # this adapter must never be labelled planar_kinematic or wheel_dynamic.
    base_mode = "fixed"

    def __init__(self, spec):
        self._spec = spec

    @property
    def spec(self):
        return self._spec

    def add_model(self, parser):
        """Register the model's package and load exactly one model instance."""
        parser.package_map().Add(
            self.spec.package_name, str(self.spec.model_path.parent.parent)
        )
        instances = parser.AddModels(str(self.spec.model_path))
        if len(instances) != 1:
            raise ValueError("A robot adapter must load exactly one model")
        return instances[0]

    def configure_model(self, plant, model_instance):
        """Configure the fixed baseline and named position actuators."""
        if self.base_mode != "fixed":
            raise ValueError(
                "Mobile adapters must implement non-welded base construction"
            )
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName(self.spec.base_link_name, model_instance),
            drake_pose(self.spec.base_pose),
        )
        self.add_actuators(plant, model_instance)

    def add_actuators(self, plant, model_instance):
        """Add position-servo actuator metadata shared by all runtime modes."""
        for spec in self.spec.controlled_joints:
            plant.AddJointActuator(
                f"{spec.name}_actuator",
                plant.GetJointByName(spec.name, model_instance),
                effort_limit=spec.effort_limit,
            )

    def initialize_state(self, plant, plant_context, model_instance):
        """Set episode initial joints once, before simulation starts."""
        q = plant.GetPositions(plant_context).copy()
        targets = dict(
            zip(self.spec.controlled_joint_names, self.spec.home_positions, strict=True)
        )
        targets.update(self.spec.locked_joint_positions)
        for name, value in targets.items():
            joint = plant.GetJointByName(name, model_instance)
            q[joint.position_start()] = value
        plant.SetPositions(plant_context, q)
        for name in self.spec.locked_joint_positions:
            plant.GetJointByName(name, model_instance).Lock(plant_context)

    def gripper_position_targets(self, width_m, name=None):
        """Map an explicitly declared parallel gripper; no gripper is required."""
        if name is not None and name not in self.spec.grippers:
            raise ValueError(f"Unknown gripper: {name}")
        gripper = self.spec.grippers[name] if name is not None else self.spec.gripper
        if gripper is None:
            raise ValueError("This robot has no default gripper")
        if not gripper.minimum_width_m <= width_m <= gripper.maximum_width_m:
            raise ValueError("Gripper width exceeds configured limits")
        travel = (gripper.maximum_width_m - width_m) / 2
        return dict(zip(gripper.joint_names, (-travel, travel), strict=True))

    def make_robot_observation(
        self, plant, plant_context, model_instance, controller_state
    ):
        """Read named actual state and frame poses from the same live plant."""
        q_all, v_all = plant.GetPositions(plant_context), plant.GetVelocities(
            plant_context
        )
        joints = [
            plant.GetJointByName(name, model_instance)
            for name in self.spec.controlled_joint_names
        ]
        q = tuple(q_all[j.position_start()] for j in joints)
        v = tuple(v_all[j.velocity_start()] for j in joints)
        frame = plant.GetFrameByName(self.spec.end_effector_frame_name, model_instance)
        velocity = frame.CalcSpatialVelocityInWorld(plant_context)
        values = dict(zip(self.spec.controlled_joint_names, q, strict=True))

        def width(gripper):
            return float(
                gripper.maximum_width_m
                - values[gripper.joint_names[1]]
                + values[gripper.joint_names[0]]
            )

        return RobotObservation(
            joint_names=self.spec.controlled_joint_names,
            q=q,
            v=v,
            **controller_state,
            end_effector_pose=public_pose(frame.CalcPoseInWorld(plant_context)),
            end_effector_twist=SpatialVelocity(
                tuple(velocity.rotational()), tuple(velocity.translational())
            ),
            gripper_width_m=width(self.spec.gripper) if self.spec.gripper else None,
            end_effectors={
                name: public_pose(
                    plant.GetFrameByName(frame_name, model_instance).CalcPoseInWorld(
                        plant_context
                    )
                )
                for name, frame_name in self.spec.end_effector_frames.items()
            },
            gripper_widths_m={
                name: width(gripper) for name, gripper in self.spec.grippers.items()
            },
            frame_poses_world={
                plant.get_body(index).name(): public_pose(
                    plant.get_body(index).EvalPoseInWorld(plant_context)
                )
                for index in plant.GetBodyIndices(model_instance)
            },
        )
