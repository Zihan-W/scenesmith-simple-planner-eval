"""Focused handoff regression: factories, adapter contracts and fresh TAMP state."""

import dataclasses
import json
import unittest
from pathlib import Path
from unittest import mock

from examples.online_manipulation.pick_lift_demo.minimal_setup import make_config
from examples.online_manipulation.tamp_execution import execute_validated_joint_goal
from examples.online_manipulation.mobile_smoke import make_config as mobile_config
from scripts.run_zerith_online_example import _parse_args, build_run
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

    def test_old_cli_hold_and_joint_step_ignore_expert_files(self):
        for policy in ("hold", "joint-step"):
            args = _parse_args(
                [
                    policy,
                    "/not-loaded/scene.dmd.yaml",
                    "--scene-package-xml",
                    "/not-loaded/package.xml",
                    "--output-root",
                    "/not-used",
                    "--pick-home-json",
                    "/missing/expert.json",
                    "--pregrasp-json",
                    "/missing/ik.json",
                    "--pick-lift-calibration-json",
                    "/missing/calibration.json",
                ]
            )
            with mock.patch("scripts.run_zerith_online_example.build_policy") as expert:
                actual, _, steps = build_run(args)
            expert.assert_not_called()
            expected = make_config(
                repository_root=ROOT,
                scene_root=Path("/not-loaded"),
                pick_artifact_root=Path("/not-loaded"),
                task_enabled=False,
            )
            self.assertIsInstance(actual.task, NullTask)
            for field in ("robot_xyz", "robot_yaw_deg", "rail_position", "q_home_left"):
                self.assertEqual(getattr(actual, field), getattr(expected, field))
            self.assertEqual(
                actual.scenario.observed_bodies, expected.scenario.observed_bodies
            )
            self.assertEqual(actual.robot_xyz, (2.65, 2.95, 0.1815))
            self.assertEqual(actual.q_home_left, (0.0,) * 7)
            self.assertEqual(actual.timing, expected.timing)
            self.assertEqual(
                actual.max_joint_delta, 0.01
            )  # Old CLI default, not .1 recipe default.
            self.assertEqual(actual.maximum_cartesian_joint_delta, 0.02)
            self.assertEqual(actual.episode_duration, 2.1)
            self.assertEqual(steps, 20)

    def test_old_cli_pick_uses_shared_task_and_policy_factory(self):
        args = _parse_args(
            [
                "pick-lift",
                "output/zerith_pick_eval/zerith_pick_eval.dmd.yaml",
                "--scene-package-xml",
                "/scene/package.xml",
                "--output-root",
                "/not-used",
                "--pick-home-json",
                "/expert/home.json",
                "--max-steps",
                "1200",
                "--maximum-joint-step",
                "0.1",
                "--closed-width",
                "0",
            ]
        )
        with mock.patch("scripts.run_zerith_online_example.build_policy") as expert:
            actual, _, _ = build_run(args)
        expected = make_config(
            repository_root=ROOT,
            scene_root=Path("/scene"),
            pick_artifact_root=ROOT / "output/zerith_pick_eval",
        )
        self.assertEqual(actual.task.config, expected.task.config)
        for field in (
            "robot_xyz",
            "robot_yaw_deg",
            "rail_position",
            "q_home_left",
            "timing",
            "max_joint_delta",
            "maximum_cartesian_joint_delta",
        ):
            self.assertEqual(getattr(actual, field), getattr(expected, field))
        self.assertAlmostEqual(actual.episode_duration, expected.episode_duration)
        self.assertIs(expert.call_args.args[0], actual)
        self.assertEqual(
            expert.call_args.kwargs["policy_overrides"]["closed_width_m"], 0
        )

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
