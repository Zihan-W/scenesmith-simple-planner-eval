"""PickLift environment configuration; no policy imports or IK-file dependency."""

import dataclasses
import json
import os
from pathlib import Path
from src.online_manipulation.recipes.settings import read_pick_settings

from src.online_manipulation import (
    ObservedBodySpec,
    PickLiftTask,
    PickLiftTaskConfig,
    ScenarioSpec,
    TimingConfig,
    VisualizationConfig,
    NullTask,
    ZerithEnvironmentConfig,
    make_zerith_robot_spec,
)


def make_config(
    *,
    repository_root: Path,
    scene_root: Path,
    pick_artifact_root: Path,
    scene_dmd: Path | None = None,
    scene_package_xml: Path | None = None,
    eval_package_xml: Path | None = None,
    robot_model_dir: Path | None = None,
    settings_path: Path | None = None,
    task_enabled: bool = True,
) -> ZerithEnvironmentConfig:
    """Build the same task/initial state for any independently supplied policy.

    Only the derived DMD is required from pick_artifact_root. Robot initial
    state and task bindings are tracked environment data, not expert outputs.
    """
    root = Path(repository_root).resolve()
    settings = read_pick_settings(
        (
            Path(settings_path)
            if settings_path is not None
            else root / "models/zerith_pick_eval/environment.json"
        )
    )
    task = settings["task"]
    initial = settings["initial_state"]
    control = settings["control"]
    bindings = settings["scene_bindings"]
    robot_dir = (
        Path(robot_model_dir)
        if robot_model_dir is not None
        else root / "models/zerith_drake"
    )
    robot = make_zerith_robot_spec(
        robot_model_dir=robot_dir,
        robot_xyz=initial["robot_xyz"],
        robot_yaw_deg=initial["robot_yaw_deg"],
        rail_position=initial["rail_position"],
        q_home_left=initial["q_home_left"],
    )
    task_config = PickLiftTaskConfig(
        target_observation_name=task["target_observation_name"],
        target_contact_body=(
            f"{bindings['target_model_name']}::{bindings['target_body_name']}"
        ),
        gripper_contact_bodies=tuple(
            f"{robot.model_instance_name}::{link}"
            for link in robot.gripper.contact_body_names
        ),
        support_contact_bodies=tuple(bindings["support_contact_bodies"]),
        required_lift_m=task["required_lift_m"],
        required_hold_s=task["required_hold_s"],
    )
    return ZerithEnvironmentConfig(
        scenario=ScenarioSpec(
            dmd_path=(
                Path(scene_dmd).resolve()
                if scene_dmd is not None
                else Path(pick_artifact_root).resolve() / "zerith_pick_eval.dmd.yaml"
            ),
            package_xmls=(
                (
                    Path(scene_package_xml).resolve()
                    if scene_package_xml is not None
                    else Path(scene_root).resolve() / "package.xml"
                ),
                (
                    Path(eval_package_xml).resolve()
                    if eval_package_xml is not None
                    else root / "models/zerith_pick_eval/package.xml"
                ),
            ),
            observed_bodies=(
                ObservedBodySpec(
                    observation_name=task["target_observation_name"],
                    model_instance_name=bindings["target_model_name"],
                    body_name=bindings["target_body_name"],
                    write_back=True,
                ),
            ),
        ),
        robot_model_dir=robot_dir,
        robot_xyz=initial["robot_xyz"],
        robot_yaw_deg=initial["robot_yaw_deg"],
        rail_position=initial["rail_position"],
        q_home_left=initial["q_home_left"],
        timing=TimingConfig(**control["timing"]),
        episode_duration=control["episode_duration"],
        max_joint_delta=control["max_joint_delta"],
        maximum_cartesian_joint_delta=control["maximum_cartesian_joint_delta"],
        task=PickLiftTask(task_config) if task_enabled else NullTask(),
        enable_planning_query=task_enabled,
    )
