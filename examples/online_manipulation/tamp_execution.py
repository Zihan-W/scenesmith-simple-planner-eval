"""Minimal TAMP query-to-online-execution handoff example."""

import dataclasses
import math
from collections.abc import Mapping

from src.online_manipulation.actions import JointPositionAction
from src.online_manipulation.observations import Observation
from src.online_manipulation.planning import PlanningQuery
from src.online_manipulation.protocols import OnlineEnvironment


@dataclasses.dataclass(frozen=True)
class JointGoalExecutionResult:
    """Result of executing one prevalidated direct joint-space edge."""

    observation: Observation
    policy_steps: int
    minimum_nonpenetration_distance_m: float
    minimum_safety_clearance_m: float


def execute_validated_joint_goal(
    *,
    env: OnlineEnvironment,
    query: PlanningQuery,
    observation: Observation,
    goal_positions: Mapping[str, float],
    maximum_policy_steps: int = 300,
    maximum_edge_joint_step: float = 0.002,
    position_tolerance: float = 0.01,
    velocity_tolerance: float = 0.05,
) -> JointGoalExecutionResult:
    """Validate one direct TAMP edge, then execute it through ``env.step``.

    The TAMP layer supplies a joint-space goal. This function synchronizes
    observed object poses, validates the current configuration and the complete
    direct edge in the independent planning context, and repeatedly sends an
    absolute typed joint target. It performs no path search or time
    parameterization.
    """
    if not goal_positions or any(not name for name in goal_positions):
        raise ValueError("goal_positions must contain named joints")
    if maximum_policy_steps < 1:
        raise ValueError("maximum_policy_steps must be positive")
    positive_values = (
        maximum_edge_joint_step,
        position_tolerance,
        velocity_tolerance,
    )
    if not all(
        math.isfinite(value) and value > 0.0 for value in positive_values
    ):
        raise ValueError("TAMP execution tolerances must be finite and positive")
    if not all(math.isfinite(value) for value in goal_positions.values()):
        raise ValueError("goal_positions must contain finite values")

    query_order = query.robot_adapter.spec.controlled_joint_names
    observed_by_name = dict(
        zip(
            observation.robot.joint_names,
            observation.robot.q,
            strict=True,
        )
    )
    missing = set(query_order) - observed_by_name.keys()
    unknown_goals = set(goal_positions) - set(query_order)
    if missing:
        raise ValueError(f"Query joints are absent from observation: {missing}")
    if unknown_goals:
        raise ValueError(f"Goal contains unknown joints: {unknown_goals}")

    query.set_observed_body_poses(
        {
            name: object_observation.pose
            for name, object_observation in observation.objects.items()
        }
    )
    start = tuple(observed_by_name[name] for name in query_order)
    goal = tuple(
        float(goal_positions.get(name, observed_by_name[name]))
        for name in query_order
    )
    start_check = query.check_configuration(start)
    if not start_check.valid:
        raise RuntimeError(
            "TAMP start configuration is invalid: "
            f"nonpenetration="
            f"{start_check.clearance.minimum_nonpenetration_distance_m}, "
            f"safety={start_check.clearance.minimum_safety_clearance_m}"
        )
    edge = query.check_edge(
        start,
        goal,
        maximum_joint_step=maximum_edge_joint_step,
    )
    if not edge.valid:
        raise RuntimeError(
            "TAMP direct edge is invalid: "
            f"nonpenetration={edge.minimum_nonpenetration_distance_m}, "
            f"safety={edge.minimum_safety_clearance_m}"
        )

    goal_by_name = dict(zip(query_order, goal, strict=True))
    commanded_names = tuple(goal_positions)
    commanded_positions = tuple(goal_by_name[name] for name in commanded_names)
    for policy_step in range(1, maximum_policy_steps + 1):
        observation, _, terminated, truncated, _ = env.step(
            JointPositionAction(commanded_names, commanded_positions)
        )
        if _goal_reached(
            observation,
            goal_positions,
            position_tolerance,
            velocity_tolerance,
        ):
            return JointGoalExecutionResult(
                observation=observation,
                policy_steps=policy_step,
                minimum_nonpenetration_distance_m=(
                    edge.minimum_nonpenetration_distance_m
                ),
                minimum_safety_clearance_m=(
                    edge.minimum_safety_clearance_m
                ),
            )
        if terminated or truncated:
            raise RuntimeError(
                "Environment ended before the validated joint goal was reached"
            )
    raise RuntimeError(
        "Validated joint goal was not reached within maximum_policy_steps"
    )


def _goal_reached(
    observation: Observation,
    goal_positions: Mapping[str, float],
    position_tolerance: float,
    velocity_tolerance: float,
) -> bool:
    """Return whether all commanded joints have settled at their targets."""
    index_by_name = {
        name: index
        for index, name in enumerate(observation.robot.joint_names)
    }
    return all(
        abs(observation.robot.q[index_by_name[name]] - target)
        <= position_tolerance
        and abs(observation.robot.v[index_by_name[name]])
        <= velocity_tolerance
        for name, target in goal_positions.items()
    )
