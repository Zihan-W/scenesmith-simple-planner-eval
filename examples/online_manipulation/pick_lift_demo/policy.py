"""External expert policy configuration; only this module reads IK artifacts."""

import dataclasses
import json
import os
from pathlib import Path

import numpy as np

from src.online_manipulation import (
    PickLiftPolicy,
    PickLiftPolicyConfig,
    PickLiftTask,
    Pose,
    ZerithEnvironmentConfig,
    make_zerith_robot_spec,
)


def build_policy(
    config: ZerithEnvironmentConfig,
    *,
    pick_artifact_root: Path,
    calibration_json: Path,
    pick_home_json: Path | None = None,
    pregrasp_json: Path | None = None,
    policy_overrides: dict | None = None,
) -> PickLiftPolicy:
    """Configure the expert for an existing environment description.

    The environment does not require these files. Only this expert needs
    PREGRASP and target-relative poses; another policy can ignore them.
    """
    if not isinstance(config.task, PickLiftTask):
        raise TypeError("This expert requires an environment with PickLiftTask")
    artifacts = Path(pick_artifact_root)
    home = json.loads(
        (
            Path(pick_home_json)
            if pick_home_json is not None
            else artifacts / "pick_home.json"
        ).read_text()
    )
    pregrasp = json.loads(
        (
            Path(pregrasp_json)
            if pregrasp_json is not None
            else artifacts / "pregrasp_ik.json"
        ).read_text()
    )
    calibration = json.loads(Path(calibration_json).read_text())
    task = config.task.config
    if (
        tuple(pregrasp["robot_base_xyz_m"]) != config.robot_xyz
        or pregrasp["robot_base_yaw_deg"] != config.robot_yaw_deg
        or home["daogui_joint_position_m"] != config.rail_position
        or pregrasp["daogui_joint_position_m"] != config.rail_position
        or calibration["rail_position_m"] != config.rail_position
        or tuple(home["q_pick_home"]) != config.q_home_left
        or f"{calibration['target_model_name']}::{calibration['target_body_name']}"
        != task.target_contact_body
    ):
        raise ValueError("Expert calibration does not match the environment")
    robot = make_zerith_robot_spec(
        robot_model_dir=config.robot_model_dir,
        robot_xyz=config.robot_xyz,
        robot_yaw_deg=config.robot_yaw_deg,
        rail_position=config.rail_position,
        q_home_left=config.q_home_left,
        cameras=config.cameras,
        locked_joint_position_overrides=config.locked_joint_position_overrides,
    )
    solution = pregrasp["solution"]
    axis = np.asarray(solution["grasp_axes_world"]["approach"], dtype=float)
    axis /= np.linalg.norm(axis)
    target = np.asarray(pregrasp["target_collision_center_xyz_m"])
    start = np.asarray(solution["actual_pregrasp_pose"]["translation_xyz_m"])
    distance = float(axis @ (target - start))
    poses = {
        name: Pose(
            tuple(calibration[name]["translation_m"]),
            tuple(calibration[name]["quaternion_wxyz"]),
        )
        for name in ("staging_pose_in_target", "grasp_pose_in_target")
    }
    policy_config = PickLiftPolicyConfig(
        target_observation_name=task.target_observation_name,
        arm_joint_names=tuple(
            name
            for name in robot.controlled_joint_names
            if name not in robot.gripper.joint_names
        ),
        pregrasp_joint_positions=tuple(home["q_pregrasp"]),
        end_effector_frame=robot.end_effector_frame_name,
        approach_axis_world=tuple(axis),
        finger_contact_bodies=task.gripper_contact_bodies,
        target_contact_body=task.target_contact_body,
        support_contact_bodies=task.support_contact_bodies,
        open_width_m=robot.gripper.maximum_width_m,
        closed_width_m=0.0,
        approach_distance_m=distance,
        lift_distance_m=0.1,
        cartesian_step_m=0.003,
        maximum_alignment_error_m=0.015,
        **poses,
    )
    return PickLiftPolicy(
        dataclasses.replace(policy_config, **(policy_overrides or {}))
    )


def make_policy(config: ZerithEnvironmentConfig) -> PickLiftPolicy:
    """Load only this policy's assets; never construct or reset an environment."""
    root = Path(__file__).resolve().parents[3]
    return build_policy(
        config,
        pick_artifact_root=Path(os.environ["PICK_ARTIFACT_ROOT"]),
        calibration_json=root / "models/zerith_pick_eval/pick_lift_calibration.json",
    )
