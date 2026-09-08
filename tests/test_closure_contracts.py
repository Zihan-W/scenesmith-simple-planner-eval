"""Focused handoff regression: factories, adapter contracts and fresh TAMP state."""

import dataclasses
import json
import unittest
from pathlib import Path
from unittest import mock

from src.online_manipulation import execute_validated_joint_goal
from src.online_manipulation.recipes.mobile import make_config as mobile_config
from src.online_manipulation import (
    BaseVelocityAction,
    DescriptionRobotAdapter,
    HoldAction,
    NullTask,
    RobotAdapter,
    build_planning_query,
    make_env,
)
from test_description_runtime import fixture_config
from test_dual_runtime import dual_config

ROOT = Path(__file__).resolve().parents[1]


class ClosureContractTest(unittest.TestCase):
    """Use actual models for construction failures, no additional robot project."""



    def test_adapter_mapping_rejected_at_construction(self):
        class MissingActuator(DescriptionRobotAdapter):
            def add_actuators(self, plant, instance):
                pass

        class WrongJoint(DescriptionRobotAdapter):
            def add_actuators(self, plant, instance):
                names = self.spec.controlled_joint_names
                for name, target in zip(names, reversed(names)):
                    plant.AddJointActuator(
                        f"{name}_actuator", plant.GetJointByName(target, instance)
                    )

        config = fixture_config()
        for cls, message in (
            (MissingActuator, "requires actuator"),
            (WrongJoint, "exclusively drive"),
        ):
            with self.subTest(adapter=cls.__name__), self.assertRaisesRegex(
                ValueError, message
            ):
                make_env(
                    dataclasses.replace(
                        config, robot_adapter=cls(config.robot_adapter.spec)
                    )
                )

    def test_named_gripper_contract(self):
        adapter = dual_config().robot_adapter
        self.assertIsInstance(adapter, RobotAdapter)
        for name, spec in adapter.spec.grippers.items():
            self.assertEqual(
                set(adapter.gripper_position_targets(0.04, name)), set(spec.joint_names)
            )
        with self.assertRaisesRegex(ValueError, "Unknown gripper"):
            adapter.gripper_position_targets(0.04, "absent")
        with self.assertRaisesRegex(ValueError, "no default gripper"):
            fixture_config().robot_adapter.gripper_position_targets(0.04)

    def test_tamp_refreshes_after_actual_mobile_motion(self):
        config = mobile_config("wheel_dynamic")
        adapter = config.robot_adapter
        home = list(adapter.spec.home_positions)
        for names in adapter.spec.arm_groups.values():
            home[adapter.spec.controlled_joint_names.index(names[3])] = 1.4
        adapter = type(adapter)(
            dataclasses.replace(adapter.spec, home_positions=tuple(home)),
            adapter.base_config,
        )
        config = dataclasses.replace(config, robot_adapter=adapter)
        env = make_env(config)
        initial, _ = env.reset(0)
        for _ in range(20):
            initial, *_ = env.step(HoldAction())
        stale = build_planning_query(
            scenario=config.scenario,
            robot_adapter=config.robot_adapter,
            timing=config.timing,
        )
        for _ in range(15):
            env.step(BaseVelocityAction(0.06, 0.08))
        for _ in range(10):
            env.step(HoldAction())
        before = env.observation
        self.assertGreater(
            before.base["pose"]["translation_m"][0]
            - initial.base["pose"]["translation_m"][0],
            0.03,
        )
        self.assertEqual(stale.state_time_s, 0)
        fresh = env.get_planning_query()
        actual_base = before.base["base_link_pose"]["translation_m"]
        fresh_base = fresh.frame_pose(
            adapter.spec.model_instance_name, adapter.spec.base_link_name
        )
        stale_base = stale.frame_pose(
            adapter.spec.model_instance_name, adapter.spec.base_link_name
        )
        for planned, measured in zip(
            fresh_base.translation_m, actual_base, strict=True
        ):
            self.assertAlmostEqual(planned, measured, places=10)
        self.assertGreater(abs(stale_base.translation_m[0] - actual_base[0]), 0.03)
        with mock.patch.object(
            env, "get_planning_query", wraps=env.get_planning_query
        ) as refresh:
            result = execute_validated_joint_goal(
                env=env,
                query=stale,
                observation=initial,
                goal_positions={before.robot.joint_names[0]: before.robot.q[0]},
                maximum_policy_steps=3,
            )
        refresh.assert_called_once()
        self.assertAlmostEqual(fresh.state_time_s, before.time_s)
        self.assertGreater(result.observation.time_s, before.time_s)
        print(
            "TAMP_SYNC_METRICS="
            + json.dumps(
                {
                    "mode": "wheel_dynamic",
                    "initial_time_s": initial.time_s,
                    "initial_base": dict(initial.base),
                    "before_execution_time_s": before.time_s,
                    "before_execution_base": dict(before.base),
                    "stale_query_time_s": stale.state_time_s,
                    "refreshed_query_time_s": fresh.state_time_s,
                    "refreshed_base_pose": fresh_base.as_dict(),
                    "stale_base_pose": stale_base.as_dict(),
                    "execution_policy_steps": result.policy_steps,
                }
            )
        )


if __name__ == "__main__":
    unittest.main()
