"""Single-object carrier identity and old/new command collision equivalence."""

import dataclasses
from pathlib import Path
from types import SimpleNamespace
import unittest
from src.online_manipulation import (
    BaseVelocityAction,
    CartesianDeltaAction,
    CartesianPoseAction,
    CompositeAction,
    ContactObservation,
    GripperAction,
    HoldAction,
    JointDeltaAction,
    JointPositionAction,
    ObservedBodySpec,
    PickLiftTask,
    PickLiftTaskConfig,
    Pose,
    RobotCommand,
    ScenarioSpec,
    build_planning_query,
    make_env,
    ZerithEnvironmentConfig,
)
from test_dual_runtime import dual_config


class PickLiftCommandContactTest(unittest.TestCase):
    """Geometry fixtures attest compatibility, not a new physical grasp demo."""

    def setUp(self):
        config = dual_config()
        self.spec = config.robot_adapter.spec
        root = Path(__file__).resolve().parents[1]
        q = build_planning_query(
            scenario=config.scenario,
            robot_adapter=config.robot_adapter,
            timing=config.timing,
        )
        self.poses = {
            side: q.frame_pose(self.spec.model_instance_name, frame)
            for side, frame in self.spec.end_effector_frames.items()
        }
        scene = ScenarioSpec(
            root / "models/runtime_fixture/contact_scene.dmd.yaml",
            (
                root / "models/runtime_fixture/package.xml",
                root / "models/mobile_scene/package.xml",
            ),
            observed_bodies=tuple(
                ObservedBodySpec(s, f"{s}_contact_target", "body")
                for s in ("left", "right")
            ),
            initial_object_poses=self.poses,
        )
        self.query = build_planning_query(
            scenario=scene, robot_adapter=config.robot_adapter, timing=config.timing
        )
        self.config = dataclasses.replace(config, scenario=scene)
        self.env = SimpleNamespace(
            observation=SimpleNamespace(
                objects={"target": SimpleNamespace(pose=self.poses["left"])},
                contacts=tuple(
                    ContactObservation(
                        f"{self.spec.model_instance_name}::{f}",
                        "left_contact_target::body",
                        1e-6,
                    )
                    for f in self.spec.grippers["left"].contact_body_names
                ),
            )
        )
        self.env.get_planning_query = lambda: self.query
        self.task = PickLiftTask(
            PickLiftTaskConfig(
                target_observation_name="target",
                target_contact_body="left_contact_target::body",
                gripper_contact_bodies=tuple(
                    f"{self.spec.model_instance_name}::{f}"
                    for f in self.spec.grippers["left"].contact_body_names
                ),
                carrier_arm_name="left",
                carrier_gripper_name="left",
            )
        )

    def test_joint_and_cartesian_forms_have_identical_carried_geometry(self):
        names = self.spec.arm_groups["left"]
        frame = self.spec.end_effector_frames["left"]
        actions = [
            JointDeltaAction((names[0],), (0.001,)),
            JointPositionAction((names[0],), (0.001,)),
            CartesianDeltaAction(frame, "world", (0.001, 0, 0), (0, 0, 0)),
            CartesianPoseAction(frame, "world", self.poses["left"]),
        ]
        baseline = None
        for arm in actions:
            for action in (
                arm,
                CompositeAction(arm, GripperAction(0.04)),
                RobotCommand(
                    arms={"left": arm}, grippers={"left": GripperAction(0.04)}
                ),
            ):
                self.query.set_observed_body_poses(self.poses)
                self.query.frame_pose_at(
                    self.spec.home_positions, self.spec.model_instance_name, frame
                )
                policy = self.task.allowed_contacts(self.env, action)
                if baseline is None:
                    baseline = policy
                self.assertEqual(policy, baseline)
                self.assertEqual(policy.carried_bodies[0].carrier_frame_name, frame)
        # Independent real geometry: put another box at the carried box position
        # only in a planning fixture. It must be detected for every action form.
        self.query.set_observed_body_poses({"right": self.poses["left"]})
        pairs = self.query.collision_pairs(
            self.spec.home_positions,
            additional_body_names=("left_contact_target::body",),
            carried_bodies=baseline.carried_bodies,
        )
        target_pairs = [
            p
            for p in pairs
            if {p.body_a, p.body_b}
            == {"left_contact_target::body", "right_contact_target::body"}
        ]
        self.assertTrue(target_pairs)
        self.assertLess(min(p.distance_m for p in target_pairs), 0)
        self.assertFalse(
            self.query.check_edge(
                self.spec.home_positions,
                self.spec.home_positions,
                contact_policy=baseline,
            ).valid
        )

    def test_ambiguity_and_wrong_carrier_are_explicit(self):
        unspecified = PickLiftTask(
            dataclasses.replace(self.task.config, carrier_arm_name=None)
        )
        with self.assertRaisesRegex(ValueError, "requires carrier_arm_name"):
            unspecified.allowed_contacts(self.env, HoldAction())
        wrong = CartesianDeltaAction(
            self.spec.end_effector_frames["right"], "world", (0, 0, 0), (0, 0, 0)
        )
        with self.assertRaisesRegex(ValueError, "differs from its bound carrier"):
            self.task.allowed_contacts(self.env, RobotCommand(arms={"left": wrong}))
        wrong_gripper = PickLiftTask(
            dataclasses.replace(self.task.config, carrier_gripper_name="right")
        )
        with self.assertRaisesRegex(ValueError, "does not match contact bodies"):
            wrong_gripper.allowed_contacts(self.env, HoldAction())
        for action in (
            BaseVelocityAction(0.01, 0),
            RobotCommand(base=BaseVelocityAction(0.01, 0)),
        ):
            with self.assertRaisesRegex(ValueError, "moving base is not supported"):
                self.task.allowed_contacts(self.env, action)

    def test_combined_joint_rejection_never_defaults_to_accepted(self):
        scene = dataclasses.replace(
            self.config.scenario,
            initial_object_poses={
                "left": self.poses["left"],
                "right": self.poses["left"],
            },
        )
        root = Path(__file__).resolve().parents[1]
        env = make_env(
            ZerithEnvironmentConfig(
                scenario=scene,
                robot_model_dir=root / "models/zerith_drake",
                robot_xyz=(0, 0, 0.2315),
                robot_yaw_deg=0,
                rail_position=0.4,
                q_home_left=(0,) * 7,
                enable_planning_query=True,
            )
        )
        obs, _ = env.reset(0)
        policy = self.task.allowed_contacts(self.env, HoldAction())
        frame = self.spec.end_effector_frames["left"]
        for arm in (
            JointDeltaAction((self.spec.arm_groups["left"][0],), (0.001,)),
            CartesianDeltaAction(frame, "world", (0.001, 0, 0), (0, 0, 0)),
        ):
            # Resolve on a real runtime but do not integrate the intentionally
            # intersecting fixture. Both paths must reject the same obstacle.
            _, old = env.backend.action_resolver.resolve(arm, policy)
            _, new = env.backend._combined_candidate(
                RobotCommand(arms={"left": arm}), policy
            )
            self.assertFalse(old["accepted"], old)
            self.assertFalse(new["accepted"], new)
        self.assertEqual(env.observation.robot.q_commanded, obs.robot.q_commanded)


if __name__ == "__main__":
    unittest.main()
