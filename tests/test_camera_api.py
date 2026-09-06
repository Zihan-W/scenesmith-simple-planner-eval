"""Contract tests for public camera specifications and observations."""

import math
import unittest
from pathlib import Path

import numpy as np

from src.online_manipulation import (
    FREE_MOTION_CONTACT_POLICY,
    CameraIntrinsics,
    CameraObservation,
    CameraSpec,
    HoldAction,
    JointSpec,
    Observation,
    Pose,
    RobotObservation,
    RobotSpec,
    SpatialVelocity,
    TaskEvaluation,
    OnlineManipulationEnv,
)


def _joint() -> JointSpec:
    """Return one valid mock revolute joint."""
    return JointSpec("joint", "revolute", -1.0, 1.0, 2.0, 3.0, 4.0, 0.5)


def _robot_spec(cameras: tuple[CameraSpec, ...]) -> RobotSpec:
    """Return a minimal robot declaration with configurable cameras."""
    return RobotSpec(
        name="mock",
        model_instance_name="mock",
        package_name="mock",
        model_path=Path("mock.urdf"),
        base_link_name="base",
        base_pose=Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
        controlled_joints=(_joint(),),
        locked_joint_positions={},
        end_effector_frame_name="tool",
        home_positions=(0.0,),
        cameras=cameras,
    )


class CameraApiTest(unittest.TestCase):
    """Validate robot-owned camera metadata and public image schemas."""

    def test_camera_spec_computes_simulation_intrinsics(self) -> None:
        camera = CameraSpec(
            name="wrist",
            parent_frame="tool",
            width=64,
            height=48,
            fov_y_rad=math.pi / 2.0,
            near_m=0.1,
            far_m=4.0,
            update_period_s=0.04,
            modalities=("rgb", "depth"),
        )

        intrinsics = camera.intrinsics

        self.assertEqual((intrinsics.width, intrinsics.height), (64, 48))
        self.assertAlmostEqual(intrinsics.focal_x_px, 24.0)
        self.assertAlmostEqual(intrinsics.focal_y_px, 24.0)
        self.assertAlmostEqual(intrinsics.center_x_px, 31.5)
        self.assertAlmostEqual(intrinsics.center_y_px, 23.5)
        self.assertEqual(camera.modalities, ("rgb", "depth"))

    def test_robot_spec_accepts_zero_or_multiple_cameras(self) -> None:
        self.assertEqual(_robot_spec(()).cameras, ())
        cameras = (
            CameraSpec("head", "head_link"),
            CameraSpec("wrist", "tool", enabled=False),
        )
        self.assertEqual(_robot_spec(cameras).cameras, cameras)

    def test_robot_spec_rejects_duplicate_public_camera_names(self) -> None:
        with self.assertRaisesRegex(ValueError, "camera names must be unique"):
            _robot_spec(
                (
                    CameraSpec("camera", "frame_a"),
                    CameraSpec("camera", "frame_b"),
                )
            )

    def test_camera_observation_enforces_shape_dtype_and_metric_depth(self):
        spec = CameraSpec(
            "camera",
            "frame",
            width=4,
            height=3,
            near_m=0.1,
            far_m=5.0,
        )
        rgb = np.zeros((3, 4, 3), dtype=np.uint8)
        depth = np.full((3, 4), 1.25, dtype=np.float32)
        label = np.zeros((3, 4), dtype=np.int16)

        observation = CameraObservation(
            frame="frame",
            timestamp_s=0.2,
            pose=Pose((0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 0.0)),
            intrinsics=spec.intrinsics,
            rgb=rgb,
            depth=depth,
            label=label,
        )

        self.assertEqual(observation.rgb.shape, (3, 4, 3))
        self.assertEqual(observation.rgb.dtype, np.uint8)
        self.assertEqual(observation.depth.dtype, np.float32)
        self.assertAlmostEqual(float(observation.depth[0, 0]), 1.25)
        self.assertEqual(observation.label.dtype, np.int16)
        self.assertFalse(observation.rgb.flags.writeable)
        self.assertFalse(observation.depth.flags.writeable)

    def test_invalid_camera_contracts_fail_loudly(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unsupported camera"):
            CameraSpec("camera", "frame", modalities=("thermal",))
        with self.assertRaisesRegex(TypeError, "rgb must have dtype"):
            CameraObservation(
                frame="frame",
                timestamp_s=0.0,
                pose=Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
                intrinsics=CameraSpec(
                    "camera", "frame", width=2, height=2
                ).intrinsics,
                rgb=np.zeros((2, 2, 3), dtype=np.float32),
            )

    def test_task_can_select_sensor_without_environment_name_knowledge(self):
        intrinsics = CameraIntrinsics(
            width=2,
            height=2,
            focal_x_px=1.0,
            focal_y_px=1.0,
            center_x_px=0.5,
            center_y_px=0.5,
            fov_y_rad=math.pi / 2.0,
            near_m=0.1,
            far_m=2.0,
        )
        camera = CameraObservation(
            frame="arbitrary_sensor_frame",
            timestamp_s=0.0,
            pose=Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
            intrinsics=intrinsics,
            rgb=np.full((2, 2, 3), 7, dtype=np.uint8),
        )

        class SensorBackend:
            """Return one camera without interpreting its public name."""

            def observation(self):
                pose = Pose(
                    (0.0, 0.0, 0.0),
                    (1.0, 0.0, 0.0, 0.0),
                )
                zeros = (0.0,)
                return Observation(
                    time_s=0.0,
                    robot=RobotObservation(
                        joint_names=("joint",),
                        q=zeros,
                        v=zeros,
                        q_commanded=zeros,
                        torque_commanded=zeros,
                        torque_applied=zeros,
                        torque_saturated=(False,),
                        end_effector_pose=pose,
                        end_effector_twist=SpatialVelocity(
                            (0.0, 0.0, 0.0),
                            (0.0, 0.0, 0.0),
                        ),
                    ),
                    objects={},
                    contacts=(),
                    task={},
                    sensors={"chosen_by_task": camera},
                )

            def reset(self, rng):
                del rng
                return self.observation(), {}

            def step(self, action, contact_policy):
                del action, contact_policy
                return self.observation(), False, {}

            def write_updated_scenario(self, output_path):
                del output_path
                return ()

            def start_recording(self):
                pass

            def save_recording(self, output_path):
                del output_path

        class SensorSelectingTask:
            """Read only the camera selected by this task."""

            def reset(self, env, rng):
                del env, rng
                return {}

            def observe(self, env):
                image = env.observation.sensors["chosen_by_task"].rgb
                return {"selected_rgb_mean": float(image.mean())}

            def evaluate(self, env):
                del env
                return TaskEvaluation()

            def allowed_contacts(self, env, action):
                del env, action
                return FREE_MOTION_CONTACT_POLICY

            def finalize(self, env):
                del env
                return {}

        env = OnlineManipulationEnv(SensorBackend(), SensorSelectingTask())
        observation, _ = env.reset(seed=0)
        observation, _, _, _, _ = env.step(HoldAction())
        self.assertEqual(observation.task["selected_rgb_mean"], 7.0)


if __name__ == "__main__":
    unittest.main()
