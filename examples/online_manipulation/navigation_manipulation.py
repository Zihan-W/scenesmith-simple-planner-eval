"""Navigate, park, then exercise both arms/grippers through public actions."""

import argparse
import dataclasses
import math
from pathlib import Path
import numpy as np

from src.online_manipulation import (
    BaseVelocityAction,
    GripperAction,
    HoldAction,
    JointPositionAction,
    NavigationGoal,
    Navigator,
    NullTask,
    Pose,
    RobotCommand,
    TaskEvaluation,
    build_navigation_map,
    build_planning_query,
    make_env,
    run_episodes,
)
from examples.online_manipulation.mobile_smoke import make_config


class NavigateExercisePolicy:
    """Own navigation/manipulation sequencing without accessing a Context."""

    def __init__(self, navigator, goal, joint_targets, gripper_targets):
        self.navigator, self.goal = navigator, goal
        self.joint_targets, self.gripper_targets = joint_targets, gripper_targets

    def reset(self, observation, info):
        self.navigator.reset()
        self.stage = "settle"
        self.start_time = observation.time_s
        self.parking_pose = None
        self.maximum_parking_drift_m = 0.0

    @property
    def stop_reason(self):
        return (
            self.navigator.status
            if self.navigator.status in ("blocked", "no_path", "timeout")
            else None
        )

    def act(self, observation):
        if self.stage == "settle":
            if observation.time_s - self.start_time < 2:
                return HoldAction()
            self.navigator.set_goal(self.goal, observation)
            self.stage = "navigate"
        if self.stage == "navigate":
            action = self.navigator.act(observation)
            if self.navigator.status != "arrived":
                return action
            self.navigator.release()
            self.parking_pose = np.array(observation.base["pose"]["translation_m"])
            self.stage = "manipulate"
        drift = np.linalg.norm(
            np.array(observation.base["pose"]["translation_m"])[:2]
            - self.parking_pose[:2]
        )
        self.maximum_parking_drift_m = max(self.maximum_parking_drift_m, float(drift))
        return RobotCommand(
            arms={
                side: JointPositionAction(tuple(targets), tuple(targets.values()))
                for side, targets in self.joint_targets.items()
            },
            grippers={
                name: GripperAction(width)
                for name, width in self.gripper_targets.items()
            },
            base=(
                BaseVelocityAction(
                    0,
                    0,
                    control_owner=self.navigator.config.control_owner,
                    release_control=True,
                )
                if observation.base["control_owner"] is not None
                else None
            ),
        )

    def diagnostics(self):
        return {
            "stage": self.stage,
            "navigation_status": self.navigator.status,
            "navigation_errors": getattr(self.navigator, "errors", {}),
            "parking_pose": (
                None if self.parking_pose is None else self.parking_pose.tolist()
            ),
            "maximum_parking_drift_m": self.maximum_parking_drift_m,
        }


class ParkedExerciseTask(NullTask):
    """Evaluate actual parking and both independent joint/gripper targets."""

    def __init__(self, goal, joint_targets, gripper_targets):
        self.goal, self.joint_targets, self.gripper_targets = (
            goal,
            joint_targets,
            gripper_targets,
        )

    def reset(self, env, rng):
        self.stable_since = None
        self.result = TaskEvaluation()
        return {"task_name": "parked_dual_exercise"}

    def observe(self, env):
        return {"task_name": "parked_dual_exercise", "success": self.result.success}

    def evaluate(self, env):
        obs = env.observation
        q = dict(zip(obs.robot.joint_names, obs.robot.q, strict=True))
        joint_error = max(
            abs(q[name] - value)
            for targets in self.joint_targets.values()
            for name, value in targets.items()
        )
        gripper_error = max(
            abs(obs.robot.gripper_widths_m[name] - width)
            for name, width in self.gripper_targets.items()
        )
        p = obs.base["pose"]
        position_error = float(
            np.linalg.norm(
                np.array(p["translation_m"][:2]) - self.goal.pose.translation_m[:2]
            )
        )
        w, x, y, z = p["quaternion_wxyz"]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        # This fixture's target yaw is explicitly zero, never inferred from commands.
        yaw_error = abs(math.atan2(math.sin(yaw), math.cos(yaw)))
        parked = (
            position_error <= 0.03
            and yaw_error <= math.radians(3)
            and np.linalg.norm(obs.base["linear_velocity_world_m_s"][:2]) <= 0.01
            and abs(obs.base["yaw_rate_rad_s"]) <= 0.02
        )
        valid = parked and joint_error < 0.01 and gripper_error < 0.004
        if not valid:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = obs.time_s
        success = (
            self.stable_since is not None
            and obs.time_s - self.stable_since >= 0.5 - 1e-9
        )
        self.result = TaskEvaluation(
            reward=float(success),
            terminated=success,
            success=success,
            reason="parked_dual_targets_held" if success else "running",
            metrics={
                "position_error_m": position_error,
                "yaw_error_rad": yaw_error,
                "joint_error_rad": joint_error,
                "gripper_error_m": gripper_error,
            },
        )
        return self.result

    def finalize(self, env):
        return {"success": self.result.success, "metrics": self.result.metrics}


def main():
    """Assemble reusable pieces; the public episode runner owns the step loop."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", required=True, choices=("planar_kinematic", "wheel_dynamic")
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--meshcat", action="store_true")
    args = parser.parse_args()
    config = make_config(args.mode, meshcat=args.meshcat, obstacle=True)
    spec = config.robot_adapter.spec
    home = list(spec.home_positions)
    for side in ("left", "right"):
        home[spec.controlled_joint_names.index(f"{side}_elbow_joint")] = 1.4
    adapter = type(config.robot_adapter)(
        dataclasses.replace(spec, home_positions=tuple(home)),
        config.robot_adapter.base_config,
    )
    goal = NavigationGoal(Pose((2.8, 0, 0), (1, 0, 0, 0)))
    joint_targets = {
        side: {spec.arm_groups[side][0]: -0.08} for side in ("left", "right")
    }
    widths = {"left": 0.060, "right": 0.055}
    config = dataclasses.replace(
        config,
        robot_adapter=adapter,
        episode_duration=180,
        task_factory=lambda: ParkedExerciseTask(goal, joint_targets, widths),
    )
    query = build_planning_query(
        scenario=config.scenario, robot_adapter=adapter, timing=config.timing
    )
    if not query.check_configuration(query.configuration()).valid:
        raise RuntimeError("Navigation posture is not collision safe")
    navigation_map = build_navigation_map(
        query,
        navigation_frame=adapter.navigation_frame_name,
        ground_body_names=config.scenario.ground_body_names,
    )
    policy = NavigateExercisePolicy(
        Navigator(navigation_map), goal, joint_targets, widths
    )
    results = run_episodes(
        env=make_env(config),
        policy=policy,
        seeds=(0,),
        max_steps=1750,
        output_root=args.output,
        record_html=args.meshcat,
    )
    print(
        {
            key: results[0].summary[key]
            for key in (
                "success",
                "termination_reason",
                "episode_time_s",
                "policy_steps",
                "task",
                "policy",
            )
        }
    )
    if not results[0].success:
        raise RuntimeError("Navigation and parked dual-arm exercise did not pass")


if __name__ == "__main__":
    main()
