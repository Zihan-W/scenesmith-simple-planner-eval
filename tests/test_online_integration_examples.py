"""Tests for the minimal BT and TAMP integration examples."""

import types
import unittest

from examples.online_manipulation import (
    BehaviorTreeStatus,
    JointTargetLeaf,
    execute_validated_joint_goal,
)
from src.online_manipulation import (
    HoldAction,
    JointPositionAction,
    ObjectObservation,
    Observation,
    Pose,
    RobotObservation,
    SpatialVelocity,
)


def _observation(position: float = 0.0, time_s: float = 0.0) -> Observation:
    """Create one-joint state for integration-example tests."""
    pose = Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    velocity = SpatialVelocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    return Observation(
        time_s=time_s,
        robot=RobotObservation(
            joint_names=("joint",),
            q=(position,),
            v=(0.0,),
            q_commanded=(position,),
            torque_commanded=(0.0,),
            torque_applied=(0.0,),
            torque_saturated=(False,),
            end_effector_pose=pose,
            end_effector_twist=velocity,
        ),
        objects={"object": ObjectObservation(pose, velocity)},
        contacts=(),
        task={},
    )


class _InstantEnvironment:
    """Apply absolute joint targets immediately for contract-level tests."""

    def __init__(self) -> None:
        self.observation = _observation()
        self.actions = []

    def step(self, action):
        self.actions.append(action)
        position = self.observation.robot.q[0]
        if isinstance(action, JointPositionAction):
            position = action.positions[0]
        self.observation = _observation(
            position,
            self.observation.time_s + 0.1,
        )
        return self.observation, 0.0, False, False, {}


class _PlanningQuery:
    """Record TAMP checks while returning configurable edge validity."""

    def __init__(self, edge_valid: bool = True) -> None:
        self.robot_adapter = types.SimpleNamespace(
            spec=types.SimpleNamespace(controlled_joint_names=("joint",))
        )
        self.edge_valid = edge_valid
        self.synced_poses = None
        clearance = types.SimpleNamespace(
            minimum_nonpenetration_distance_m=0.02,
            minimum_safety_clearance_m=0.03,
        )
        self.configuration_result = types.SimpleNamespace(
            valid=True,
            clearance=clearance,
        )

    def set_observed_body_poses(self, poses) -> None:
        self.synced_poses = poses

    def check_configuration(self, configuration):
        self.start = configuration
        return self.configuration_result

    def check_edge(self, start, goal, maximum_joint_step):
        self.edge = (start, goal, maximum_joint_step)
        return types.SimpleNamespace(
            valid=self.edge_valid,
            minimum_nonpenetration_distance_m=0.02,
            minimum_safety_clearance_m=0.03,
        )


class OnlineIntegrationExamplesTest(unittest.TestCase):
    """Validate one-step BT semantics and query-before-execution TAMP flow."""

    def test_behavior_tree_leaf_ticks_environment_once(self) -> None:
        env = _InstantEnvironment()
        leaf = JointTargetLeaf("joint", 0.2)
        result = leaf.tick(env, env.observation)
        self.assertEqual(result.status, BehaviorTreeStatus.SUCCESS)
        self.assertEqual(len(env.actions), 1)
        self.assertIsInstance(env.actions[0], JointPositionAction)

        result = leaf.tick(env, result.observation)
        self.assertEqual(result.status, BehaviorTreeStatus.SUCCESS)
        self.assertIsInstance(env.actions[-1], HoldAction)

    def test_tamp_checks_edge_before_online_execution(self) -> None:
        env = _InstantEnvironment()
        query = _PlanningQuery()
        result = execute_validated_joint_goal(
            env=env,
            query=query,
            observation=env.observation,
            goal_positions={"joint": 0.2},
        )
        self.assertEqual(result.policy_steps, 1)
        self.assertEqual(query.start, (0.0,))
        self.assertEqual(query.edge, ((0.0,), (0.2,), 0.002))
        self.assertIn("object", query.synced_poses)
        self.assertEqual(len(env.actions), 1)
        self.assertEqual(env.actions[0].positions, (0.2,))

    def test_tamp_rejects_invalid_edge_without_stepping(self) -> None:
        env = _InstantEnvironment()
        query = _PlanningQuery(edge_valid=False)
        with self.assertRaisesRegex(RuntimeError, "direct edge is invalid"):
            execute_validated_joint_goal(
                env=env,
                query=query,
                observation=env.observation,
                goal_positions={"joint": 0.2},
            )
        self.assertEqual(env.actions, [])


if __name__ == "__main__":
    unittest.main()
