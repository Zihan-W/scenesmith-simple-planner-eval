"""Zerith dual-arm model description using the existing derived assets."""

import dataclasses
from pydrake.all import FixedOffsetFrame, RigidTransform

from src.online_manipulation.adapters.description import DescriptionRobotAdapter
from src.online_manipulation.adapters.zerith import (
    make_zerith_robot_spec,
    _joint_elements,
    _joint_spec,
)
from src.online_manipulation.specs import GripperSpec


def make_zerith_dual_spec(*, q_home_right=None, **kwargs):
    """Add the right arm with explicit mirrored simulation gains, not hardware claims."""
    left = make_zerith_robot_spec(**kwargs)
    xml = _joint_elements(left.model_path)
    right = tuple(
        _joint_spec(
            xml[j.name.replace("left_", "right_", 1)],
            kp=j.kp,
            kd=j.kd,
            effort_limit=j.effort_limit,
        )
        for j in left.controlled_joints
    )
    right_names = tuple(j.name for j in right)
    q_right = (
        tuple(q_home_right)
        if q_home_right is not None
        else tuple(0.0 for _ in right[:-2])
    )
    right_gripper = GripperSpec(
        right_names[-2:],
        left.gripper.minimum_width_m,
        left.gripper.maximum_width_m,
        ("right_jaw_left_finger_link", "right_jaw_right_finger_link"),
    )
    return dataclasses.replace(
        left,
        name="zerith_dual_arm",
        controlled_joints=left.controlled_joints + right,
        home_positions=left.home_positions + q_right + (0.0, 0.0),
        locked_joint_positions={
            n: q for n, q in left.locked_joint_positions.items() if n not in right_names
        },
        arm_groups={
            "left": left.controlled_joint_names[:-2],
            "right": right_names[:-2],
        },
        end_effector_frames={"left": "left_grasp_frame", "right": "right_grasp_frame"},
        grippers={"left": left.gripper, "right": right_gripper},
        safety_exempt_body_pairs=left.safety_exempt_body_pairs
        + (
            (
                f"{left.model_instance_name}::body_yaw_link",
                f"{left.model_instance_name}::right_shoulder_roll_link",
            ),
        ),
    )


class ZerithDualRobotAdapter(DescriptionRobotAdapter):
    """Named dual arms and grippers; fixed baseline before mobile extension."""

    def configure_model(self, plant, instance):
        super().configure_model(plant, instance)
        self.add_tcp_frames(plant, instance)

    def add_tcp_frames(self, plant, instance):
        """Use the same calibrated mechanical TCP offset on both hands."""
        for name, frame_name in self.spec.end_effector_frames.items():
            plant.AddFrame(
                FixedOffsetFrame(
                    frame_name,
                    plant.GetFrameByName(f"{name}_wrist_pitch_link", instance),
                    RigidTransform([0.176, 0, -0.002]),
                )
            )
