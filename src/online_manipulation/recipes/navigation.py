"""Existing navigate-park-dual-exercise benchmark, not a general planner."""
import math
import numpy as np
from src.online_manipulation import (
    BaseVelocityAction, JointPositionAction, GripperAction, RobotCommand,
    HoldAction, NullTask, TaskEvaluation, NavigationGoal, Pose, Navigator,
    build_planning_query, build_navigation_map,
)

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



def make_task(options, spec):
    """Bind fixture targets to named robot capabilities."""
    for side in ("left", "right"):
        if side not in spec.arm_groups or side not in spec.grippers:
            raise ValueError("Parked exercise requires left/right arms and grippers")
    goal = NavigationGoal(Pose(tuple(options["goal_xyz_m"]), (1, 0, 0, 0)))
    targets = {side: {spec.arm_groups[side][0]: options["arm_target_rad"]}
               for side in ("left", "right")}
    return ParkedExerciseTask(goal, targets, options["gripper_widths_m"])


def make_policy(context):
    """Check the initial posture and build a static map through PlanningQuery."""
    config = context.environment_config
    task = config.task_factory()
    if not isinstance(task, ParkedExerciseTask):
        raise TypeError("Navigation exercise policy requires ParkedExerciseTask")
    if not hasattr(config.robot_adapter, "base_config"):
        raise ValueError("Navigation exercise requires a mobile RobotAdapter")
    query = build_planning_query(scenario=config.scenario,
                                robot_adapter=config.robot_adapter, timing=config.timing)
    if not query.check_configuration(query.configuration()).valid:
        raise ValueError("Navigation posture is not collision safe")
    navigation_map = build_navigation_map(
        query, navigation_frame=config.robot_adapter.navigation_frame_name,
        ground_body_names=config.scenario.ground_body_names,
        ground_geometries=config.scenario.ground_geometries)
    return NavigateExercisePolicy(Navigator(navigation_map), task.goal,
                                  task.joint_targets, task.gripper_targets)
