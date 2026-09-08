"""Real dual-arm action acceptance, atomic rejection and time regression."""

from pathlib import Path
import dataclasses
import unittest
import numpy as np

from src.online_manipulation import (
    GripperAction,
    HoldAction,
    JointDeltaAction,
    RobotCommand,
    RuntimeConfig,
    ScenarioSpec,
    ZerithDualRobotAdapter,
    make_env,
    make_zerith_dual_spec,
)


def dual_config():
    """Use repository assets, without any expert task or IK files."""
    root = Path(__file__).resolve().parents[1]
    scene = root / "models/online_env_minimal_scene"
    spec = make_zerith_dual_spec(
        robot_model_dir=root / "models/zerith_drake",
        robot_xyz=(0, 0, 0.2315),
        robot_yaw_deg=0,
        rail_position=0.4,
        q_home_left=(0,) * 7,
    )
    return RuntimeConfig(
        ScenarioSpec(scene / "scene.dmd.yaml", (scene / "package.xml",)),
        ZerithDualRobotAdapter(spec),
    )


class DualRuntimeTest(unittest.TestCase):
    """Both arms must physically respond to a single committed action."""

    def test_both_cartesian_forms_on_named_actual_arms(self):
        from src.online_manipulation import (
            CartesianDeltaAction,
            CartesianPoseAction,
            Pose,
        )

        config = dual_config()
        adapter = config.robot_adapter
        home = list(adapter.spec.home_positions)
        for side in ("left", "right"):
            home[
                adapter.spec.controlled_joint_names.index(
                    adapter.spec.arm_groups[side][3]
                )
            ] = 1.4
        adapter = type(adapter)(
            dataclasses.replace(adapter.spec, home_positions=tuple(home))
        )
        env = make_env(dataclasses.replace(config, robot_adapter=adapter))
        initial, _ = env.reset(0)
        right = initial.robot.end_effectors["right"]
        action = RobotCommand(
            arms={
                "left": CartesianDeltaAction(
                    adapter.spec.end_effector_frames["left"],
                    "world",
                    (0, 0, 0.001),
                    (0, 0, 0),
                ),
                "right": CartesianPoseAction(
                    adapter.spec.end_effector_frames["right"],
                    "world",
                    Pose(
                        tuple(np.array(right.translation_m) + [0, 0, 0.001]),
                        right.quaternion_wxyz,
                    ),
                ),
            }
        )
        obs, _, _, _, info = env.step(action)
        self.assertTrue(info["action_decision"]["accepted"], info)
        for side in ("left", "right"):
            self.assertGreater(
                np.linalg.norm(
                    np.array(obs.robot.end_effectors[side].translation_m)
                    - initial.robot.end_effectors[side].translation_m
                ),
                1e-5,
            )
        self.assertAlmostEqual(obs.time_s, 0.1)
        held = obs.robot.q_commanded
        bad = RobotCommand(
            arms={
                "left": action.arms["left"],
                "right": CartesianDeltaAction(
                    adapter.spec.end_effector_frames["right"],
                    "unsupported",
                    (0, 0, 0.001),
                    (0, 0, 0),
                ),
            }
        )
        obs, _, _, _, info = env.step(bad)
        self.assertFalse(info["action_decision"]["accepted"])
        self.assertEqual(obs.robot.q_commanded, held)

    def test_actual_cross_arm_collision_candidate(self):
        from src.online_manipulation import build_planning_query

        config = dual_config()
        query = build_planning_query(
            scenario=config.scenario,
            robot_adapter=config.robot_adapter,
            timing=config.timing,
        )
        candidate = (
            -0.9087098288239299,
            0.16573074682331407,
            -1.5041403101973398,
            0.8858147892063318,
            -0.9599869817834961,
            -0.023198356945379306,
            -0.12461862991548855,
            0.0,
            0.0,
            -0.8905929266896627,
            0.017689857448653812,
            1.6345777442265073,
            1.0469991359220667,
            -0.2333193258034285,
            0.17518697428146246,
            0.34243441168870425,
            0.0,
            0.0,
        )
        # Both configurations pass when the other arm stays at home.
        # Their combination fails: independent arm checks are insufficient.
        self.assertTrue(query.check_configuration(candidate[:9] + (0.0,) * 9).valid)
        self.assertTrue(query.check_configuration((0.0,) * 9 + candidate[9:]).valid)
        pairs = query.collision_pairs(candidate)
        relevant = [
            p
            for p in pairs
            if "left_jaw_right_finger_link" in p.body_a + p.body_b
            and "right_wrist_pitch_link" in p.body_a + p.body_b
        ]
        self.assertTrue(relevant)
        self.assertLess(min(p.distance_m for p in relevant), -0.025)
        self.assertFalse(query.check_configuration(candidate).valid)
        # Both arms change in one edge, rather than checking two separate worlds.
        edge = query.check_edge(
            config.robot_adapter.spec.home_positions, candidate, maximum_joint_step=0.02
        )
        self.assertFalse(edge.valid)

    def test_joint_stop_numerical_noise_is_not_a_collision(self):
        from src.online_manipulation import build_planning_query

        config = dual_config()
        query = build_planning_query(
            scenario=config.scenario,
            robot_adapter=config.robot_adapter,
            timing=config.timing,
        )
        q = list(config.robot_adapter.spec.home_positions)
        index = config.robot_adapter.spec.controlled_joint_names.index(
            "left_jaw_right_finger_joint"
        )
        q[index] = -3e-11
        self.assertTrue(query.check_configuration(q).within_joint_limits)
        q[index] = -1e-6
        self.assertFalse(query.check_configuration(q).within_joint_limits)

    def test_two_grippers_respect_explicit_task_contact_pairs(self):
        from src.online_manipulation import (
            ObservedBodySpec,
            PairContactPolicy,
            build_planning_query,
        )

        config = dual_config()
        spec = config.robot_adapter.spec
        query = build_planning_query(
            scenario=config.scenario,
            robot_adapter=config.robot_adapter,
            timing=config.timing,
        )
        root = Path(__file__).resolve().parents[1]
        bodies = tuple(
            ObservedBodySpec(side, f"{side}_contact_target", "body")
            for side in ("left", "right")
        )
        poses = {
            side: query.frame_pose_at(
                spec.home_positions,
                spec.model_instance_name,
                spec.end_effector_frames[side],
            )
            for side in ("left", "right")
        }
        scene = ScenarioSpec(
            root / "models/runtime_fixture/contact_scene.dmd.yaml",
            (
                root / "models/runtime_fixture/package.xml",
                root / "models/mobile_scene/package.xml",
            ),
            observed_bodies=bodies,
            initial_object_poses=poses,
        )
        query = build_planning_query(
            scenario=scene, robot_adapter=config.robot_adapter, timing=config.timing
        )
        contact_pairs = [
            (f"{spec.model_instance_name}::{finger}", f"{side}_contact_target::body")
            for side in ("left", "right")
            for finger in spec.grippers[side].contact_body_names
        ]
        policy = PairContactPolicy.from_pairs(
            "two_gripper_contact", contact_pairs, maximum_allowed_penetration_m=0.0001
        )
        q = list(spec.home_positions)
        # Pose assignment occurs only in the independent planning fixture.
        # This tests real geometry/Task rules, not physical grasp success.
        for side in ("left", "right"):
            low, high = 0.03, 0.05
            for _ in range(28):
                width = (low + high) / 2
                for name, value in config.robot_adapter.gripper_position_targets(
                    width, side
                ).items():
                    q[spec.controlled_joint_names.index(name)] = value
                distances = [
                    p.distance_m
                    for p in query.collision_pairs(q)
                    if f"{side}_contact_target::body" in (p.body_a, p.body_b)
                    and policy.permits(p.body_a, p.body_b)
                ]
                self.assertTrue(distances)
                if min(distances) < -2e-5:
                    low = width
                else:
                    high = width
        self.assertFalse(query.check_configuration(q).valid)
        self.assertTrue(query.check_configuration(q, contact_policy=policy).valid)
        self.assertTrue(
            query.check_edge(spec.home_positions, q, contact_policy=policy).valid
        )

    def test_motion_omission_and_atomic_rejection(self):
        config = dual_config()
        spec = config.robot_adapter.spec
        env = make_env(config)
        initial, _ = env.reset(0)
        command = RobotCommand(
            arms={
                side: JointDeltaAction((spec.arm_groups[side][0],), (-0.015,))
                for side in ("left", "right")
            },
            grippers={"left": GripperAction(0.070), "right": GripperAction(0.065)},
        )
        obs, _, _, _, info = env.step(command)
        self.assertTrue(info["action_decision"]["accepted"], info)
        self.assertAlmostEqual(obs.time_s, 0.1)
        self.assertEqual(info["control_updates"], 20)
        for side in ("left", "right"):
            index = obs.robot.joint_names.index(spec.arm_groups[side][0])
            self.assertLess(obs.robot.q[index], -0.001)
        self.assertLess(
            obs.robot.gripper_widths_m["right"], obs.robot.gripper_widths_m["left"]
        )
        before = obs.robot.q_commanded
        obs, _, _, _, info = env.step(
            RobotCommand(
                arms={"left": JointDeltaAction((spec.arm_groups["left"][0],), (0.01,))},
                grippers={"right": GripperAction(1.0)},
            )
        )
        self.assertFalse(info["action_decision"]["accepted"])
        self.assertEqual(obs.robot.q_commanded, before)
        self.assertAlmostEqual(obs.time_s, 0.2)
        obs, *_ = env.step(HoldAction())
        self.assertEqual(obs.robot.q_commanded, before)
        again, _ = env.reset(0)
        np.testing.assert_array_equal(initial.robot.q, again.robot.q)


if __name__ == "__main__":
    unittest.main()
