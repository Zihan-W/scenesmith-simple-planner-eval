"""Explicit simulation reset used identically by CLI and validation harness."""
import dataclasses
import math
from simulation.src import Pose, make_env


def reset_scene(experiment, *, seed, shift_world_x_m=0.0):
    """Reset the configured scene; optional target shift is measured and recorded."""
    if not math.isfinite(shift_world_x_m):
        raise ValueError("Target shift must be finite")
    if shift_world_x_m == 0.0:
        env = make_env(experiment.environment_config)
        observation, reset_info = env.reset(seed=seed)
        return experiment, env, observation, reset_info, None
    task = experiment.environment_config.task_factory()
    target = getattr(task, "task", task).config.target_observation_name
    reference_env = make_env(experiment.environment_config)
    reference_observation, _ = reference_env.reset(seed=seed)
    original_pose = reference_observation.objects[target].pose
    if experiment.environment_config.scenario.pose_randomizations:
        raise ValueError("The scene must not randomize any object pose")
    shifted_pose = Pose(
        (original_pose.translation_m[0] + shift_world_x_m,
         original_pose.translation_m[1], original_pose.translation_m[2]),
        original_pose.quaternion_wxyz,
    )
    scenario = dataclasses.replace(
        experiment.environment_config.scenario,
        initial_object_poses={
            **experiment.environment_config.scenario.initial_object_poses,
            target: shifted_pose,
        },
    )
    experiment = dataclasses.replace(
        experiment,
        environment_config=dataclasses.replace(experiment.environment_config, scenario=scenario),
    )
    reference_base = reference_observation.base["base_link_pose"]
    del reference_env
    env = make_env(experiment.environment_config)
    observation, reset_info = env.reset(seed=seed)
    measured_pose = observation.objects[target].pose
    measured_base = observation.base["base_link_pose"]
    position_delta = [a - b for a, b in zip(
        measured_pose.translation_m, original_pose.translation_m, strict=True)]
    base_delta = [a - b for a, b in zip(
        measured_base["translation_m"], reference_base["translation_m"], strict=True)]
    if max(abs(a - b) for a, b in zip(
            position_delta, (shift_world_x_m, 0.0, 0.0), strict=True)) >= 5e-5:
        raise RuntimeError(f"Measured target shift differs from protocol: {position_delta}")
    if max(map(abs, base_delta)) >= 1e-8:
        raise RuntimeError(f"Initial robot base moved unexpectedly: {base_delta}")
    protocol = {
        "mode": "EXPLICIT_TARGET_WORLD_X_TRANSLATION",
        "requested_world_translation_m": [shift_world_x_m, 0.0, 0.0],
        "measured_world_translation_m": position_delta,
        "original_object_pose": dataclasses.asdict(original_pose),
        "shifted_object_pose": dataclasses.asdict(measured_pose),
        "original_base_link_pose": reference_base,
        "shifted_base_link_pose": measured_base,
        "scene_pose_randomizations": 0,
        "random_seed_fixed": True,
        "production_experiment_modified": False,
    }
    return experiment, env, observation, reset_info, protocol
