"""Real moving-base / moving-wrist sampled-camera attachment regression."""

import dataclasses
import unittest

import numpy as np

from examples.online_manipulation.mobile_smoke import make_config
from src.online_manipulation import (
    BaseVelocityAction,
    JointDeltaAction,
    RobotCommand,
    TimingConfig,
    make_env,
    make_zerith_camera_specs,
)
from src.online_manipulation.adapters.description import drake_pose


class MobileCameraTest(unittest.TestCase):
    """Both real base implementations must move the very same camera frames."""

    def test_pose_images_and_time_latch_on_both_modes(self):
        for mode in ("planar_kinematic", "wheel_dynamic"):
            with self.subTest(mode=mode):
                config = make_config(mode)
                adapter = config.robot_adapter
                cameras = make_zerith_camera_specs(
                    enabled_names=(
                        "head_camera",
                        "left_wrist_camera",
                        "right_wrist_camera",
                    ),
                    width=64,
                    height=48,
                    update_period_s=0.25,
                )
                adapter = type(adapter)(
                    dataclasses.replace(adapter.spec, cameras=cameras),
                    adapter.base_config,
                )
                env = make_env(
                    dataclasses.replace(
                        config,
                        robot_adapter=adapter,
                        timing=TimingConfig(policy_dt=0.05),
                    )
                )
                initial, _ = env.reset(3)
                obs = initial
                for tick in range(1, 6):
                    command = RobotCommand(
                        arms={
                            side: JointDeltaAction(
                                (adapter.spec.arm_groups[side][0],), (-0.004,)
                            )
                            for side in ("left", "right")
                        },
                        base=BaseVelocityAction(0.08, 0.15),
                    )
                    obs, _, _, _, info = env.step(command)
                    self.assertTrue(info["action_decision"]["accepted"], info)
                    for camera in cameras:
                        frame = obs.sensors[camera.name]
                        if tick < 5:
                            self.assertEqual(frame.timestamp_s, 0)
                            self.assertEqual(
                                frame.pose, initial.sensors[camera.name].pose
                            )
                            for modality in ("rgb", "depth", "label"):
                                np.testing.assert_array_equal(
                                    getattr(frame, modality),
                                    getattr(initial.sensors[camera.name], modality),
                                )
                        else:
                            self.assertAlmostEqual(frame.timestamp_s, 0.25)
                            expected = drake_pose(
                                obs.robot.frame_poses_world[camera.parent_frame]
                            ) @ drake_pose(camera.X_parent_camera_optical)
                            # A discrete RGBD event samples the pre-update
                            # Plant state; observation.q is post-update at the
                            # same timestamp. Bound the mismatch by one 1 ms
                            # physical update, not an entire camera period.
                            body_frame = env.backend.plant.GetFrameByName(
                                camera.parent_frame, env.backend.instance
                            )
                            velocity = body_frame.CalcSpatialVelocityInWorld(
                                env.backend.plant_context
                            )
                            angular_speed = np.linalg.norm(velocity.rotational())
                            linear_speed = np.linalg.norm(velocity.translational())
                            lever = np.linalg.norm(
                                camera.X_parent_camera_optical.translation_m
                            )
                            actual = drake_pose(frame.pose)
                            self.assertLessEqual(
                                np.linalg.norm(
                                    actual.translation() - expected.translation()
                                ),
                                2
                                * config.timing.physics_dt
                                * (linear_speed + lever * angular_speed)
                                + 1e-7,
                            )
                            angle = (
                                (actual.rotation() @ expected.rotation().inverse())
                                .ToAngleAxis()
                                .angle()
                            )
                            self.assertLessEqual(
                                angle,
                                2 * config.timing.physics_dt * angular_speed + 1e-7,
                            )
                            self.assertGreater(
                                np.linalg.norm(
                                    np.array(frame.pose.translation_m)
                                    - initial.sensors[camera.name].pose.translation_m
                                ),
                                0.001,
                            )
                self.assertGreater(
                    np.linalg.norm(
                        np.array(obs.base["pose"]["translation_m"])
                        - initial.base["pose"]["translation_m"]
                    ),
                    0.001,
                )
                repeated, _ = env.reset(3)
                for name, first in initial.sensors.items():
                    self.assertEqual(first.pose, repeated.sensors[name].pose)
                    np.testing.assert_array_equal(first.rgb, repeated.sensors[name].rgb)


if __name__ == "__main__":
    unittest.main()
