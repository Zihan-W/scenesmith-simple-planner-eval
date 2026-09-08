"""Real-Drake tests for Zerith camera attachment and sampled images."""

import unittest
from pathlib import Path

import numpy as np
from pydrake.all import Quaternion, RigidTransform

from src.online_manipulation import (
    HoldAction,
    JointDeltaAction,
    NullTask,
    ObservedBodySpec,
    PlanarPoseRandomizationSpec,
    Pose,
    ScenarioSpec,
    ZerithEnvironmentConfig,
    make_env,
    make_zerith_camera_specs,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SCENE_ROOT = REPOSITORY_ROOT / "models" / "online_env_minimal_scene"
ROBOT_MODEL_DIR = REPOSITORY_ROOT / "models" / "zerith_drake"


def _config(
    *,
    dmd_name: str = "scene.dmd.yaml",
    camera_names: tuple[str, ...] = ("head_camera",),
    update_period_s: float = 0.05,
    modalities: tuple[str, ...] = ("rgb", "depth", "label"),
    q_home_left: tuple[float, ...] = (0.0,) * 7,
    randomize_marker: bool = False,
    locked_joint_position_overrides: dict[str, float] | None = None,
) -> ZerithEnvironmentConfig:
    """Return a self-contained camera test configuration."""
    return ZerithEnvironmentConfig(
        scenario=ScenarioSpec(
            dmd_path=SCENE_ROOT / dmd_name,
            package_xmls=(SCENE_ROOT / "package.xml",),
            observed_bodies=(
                (
                    ObservedBodySpec(
                        "camera_marker",
                        "camera_marker",
                        "base_link",
                    ),
                )
                if randomize_marker
                else ()
            ),
            pose_randomizations=(
                (
                    PlanarPoseRandomizationSpec(
                        "camera_marker",
                        x_offset_range_m=(-0.05, 0.05),
                        y_offset_range_m=(-0.05, 0.05),
                        yaw_offset_range_rad=(-0.2, 0.2),
                    ),
                )
                if randomize_marker
                else ()
            ),
        ),
        robot_model_dir=ROBOT_MODEL_DIR,
        robot_xyz=(0.0, 0.0, 0.2315),
        robot_yaw_deg=0.0,
        rail_position=0.4,
        q_home_left=q_home_left,
        episode_duration=1.0,
        task=NullTask(),
        locked_joint_position_overrides=(
            locked_joint_position_overrides or {}
        ),
        cameras=make_zerith_camera_specs(
            enabled_names=camera_names,
            width=64,
            height=48,
            update_period_s=update_period_s,
            modalities=modalities,
        ),
    )


def _rigid_transform(pose) -> RigidTransform:
    """Convert a public Pose to a Drake transform for test arithmetic."""
    return RigidTransform(Quaternion(pose.quaternion_wxyz), pose.translation_m)


class ZerithCameraTest(unittest.TestCase):
    """Validate rendering, timing, attachment, and scene replacement."""

    def test_zerith_adapter_declares_all_urdf_camera_mounts(self) -> None:
        cameras = make_zerith_camera_specs(
            enabled_names=(
                "left_wrist_camera",
                "right_wrist_camera",
                "head_camera",
            )
        )
        self.assertEqual(
            {camera.name: camera.parent_frame for camera in cameras},
            {
                "left_wrist_camera": "left_wrist_pitch_link",
                "right_wrist_camera": "right_wrist_pitch_link",
                "head_camera": "neck_pitch_link",
            },
        )
        self.assertTrue(all(camera.enabled for camera in cameras))
        identity = Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        self.assertTrue(
            all(camera.X_mount_camera_optical == identity for camera in cameras)
        )

    def test_reset_returns_deterministic_real_camera_frame(self) -> None:
        env = make_env(_config())
        first, _ = env.reset(seed=9)
        repeated, _ = env.reset(seed=9)

        first_camera = first.sensors["head_camera"]
        repeated_camera = repeated.sensors["head_camera"]
        self.assertEqual(first_camera.timestamp_s, 0.0)
        self.assertEqual(first_camera.rgb.shape, (48, 64, 3))
        self.assertEqual(first_camera.rgb.dtype, np.uint8)
        self.assertEqual(first_camera.depth.shape, (48, 64))
        self.assertEqual(first_camera.depth.dtype, np.float32)
        self.assertEqual(first_camera.label.shape, (48, 64))
        self.assertEqual(first_camera.label.dtype, np.int16)
        self.assertGreater(int(first_camera.rgb.max()), 0)
        np.testing.assert_array_equal(first_camera.rgb, repeated_camera.rgb)
        np.testing.assert_array_equal(first_camera.depth, repeated_camera.depth)
        np.testing.assert_array_equal(first_camera.label, repeated_camera.label)

    def test_seeded_scene_randomization_is_image_reproducible(self) -> None:
        env = make_env(
            _config(
                dmd_name="camera_variant.dmd.yaml",
                randomize_marker=True,
            )
        )
        first, first_info = env.reset(seed=101)
        repeated, repeated_info = env.reset(seed=101)
        changed, changed_info = env.reset(seed=102)

        np.testing.assert_array_equal(
            first.sensors["head_camera"].rgb,
            repeated.sensors["head_camera"].rgb,
        )
        self.assertEqual(
            first_info["episode_randomization"],
            repeated_info["episode_randomization"],
        )
        self.assertNotEqual(
            first_info["episode_randomization"],
            changed_info["episode_randomization"],
        )
        self.assertFalse(
            np.array_equal(
                first.sensors["head_camera"].rgb,
                changed.sensors["head_camera"].rgb,
            )
        )

    def test_depth_uses_metric_values_and_documented_invalid_pixels(self):
        env = make_env(_config(modalities=("depth",)))
        observation, _ = env.reset(seed=0)
        camera = observation.sensors["head_camera"]

        valid = np.isfinite(camera.depth) & (camera.depth > 0.0)

        self.assertGreater(int(valid.sum()), 0)
        self.assertGreaterEqual(
            float(camera.depth[valid].min()),
            camera.intrinsics.near_m,
        )
        self.assertLessEqual(
            float(camera.depth[valid].max()),
            camera.intrinsics.far_m,
        )
        self.assertIsNone(camera.rgb)
        self.assertIsNone(camera.label)

    def test_camera_timestamp_is_sampled_and_held_between_frames(self):
        env = make_env(_config(update_period_s=0.15))
        reset_observation, _ = env.reset(seed=0)
        first, _, _, _, _ = env.step(HoldAction())
        second, _, _, _, _ = env.step(HoldAction())

        self.assertEqual(reset_observation.sensors["head_camera"].timestamp_s, 0.0)
        self.assertEqual(first.sensors["head_camera"].timestamp_s, 0.0)
        self.assertAlmostEqual(
            second.sensors["head_camera"].timestamp_s,
            0.15,
        )
        np.testing.assert_array_equal(
            reset_observation.sensors["head_camera"].rgb,
            first.sensors["head_camera"].rgb,
        )

    def test_moving_wrist_camera_holds_one_coherent_slow_frame(self):
        env = make_env(
            _config(
                camera_names=("left_wrist_camera",),
                update_period_s=0.3,
            )
        )
        initial, _ = env.reset(seed=0)
        moved, _, _, _, _ = env.step(
            JointDeltaAction(("left_shoulder_pitch_joint",), (0.08,))
        )
        held, _, _, _, _ = env.step(HoldAction())
        updated, _, _, _, _ = env.step(HoldAction())

        initial_camera = initial.sensors["left_wrist_camera"]
        moved_camera = moved.sensors["left_wrist_camera"]
        held_camera = held.sensors["left_wrist_camera"]
        updated_camera = updated.sensors["left_wrist_camera"]
        shoulder_index = initial.robot.joint_names.index(
            "left_shoulder_pitch_joint"
        )

        self.assertGreater(
            abs(moved.robot.q[shoulder_index] - initial.robot.q[shoulder_index]),
            1e-4,
        )
        for camera in (moved_camera, held_camera):
            self.assertEqual(camera.timestamp_s, initial_camera.timestamp_s)
            self.assertEqual(camera.pose, initial_camera.pose)
            np.testing.assert_array_equal(camera.rgb, initial_camera.rgb)
            np.testing.assert_array_equal(camera.depth, initial_camera.depth)
            np.testing.assert_array_equal(camera.label, initial_camera.label)

        self.assertAlmostEqual(updated_camera.timestamp_s, 0.3)
        self.assertNotEqual(updated_camera.pose, initial_camera.pose)
        self.assertFalse(np.array_equal(updated_camera.rgb, initial_camera.rgb))
        self.assertFalse(
            np.array_equal(updated_camera.depth, initial_camera.depth)
        )
        self.assertFalse(
            np.array_equal(updated_camera.label, initial_camera.label)
        )

    def test_wrist_camera_pose_follows_joint_and_keeps_mount_transform(self):
        initial_env = make_env(_config(camera_names=("left_wrist_camera",)))
        moved_env = make_env(
            _config(
                camera_names=("left_wrist_camera",),
                q_home_left=(0.08, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            )
        )
        initial, _ = initial_env.reset(seed=0)
        moved, _ = moved_env.reset(seed=0)
        camera_spec = next(
            camera
            for camera in initial_env.backend.adapter.spec.cameras
            if camera.name == "left_wrist_camera"
        )

        def mount_error(env, observation) -> float:
            runtime = env.backend
            plant = runtime.plant
            context = runtime.plant_context
            parent = plant.GetFrameByName(
                camera_spec.parent_frame,
                runtime.instance,
            )
            X_WP = plant.CalcRelativeTransform(
                context,
                plant.world_frame(),
                parent,
            )
            X_WC = _rigid_transform(
                observation.sensors["left_wrist_camera"].pose
            )
            X_PC = X_WP.inverse() @ X_WC
            expected = _rigid_transform(camera_spec.X_parent_camera)
            return max(
                np.max(np.abs(X_PC.translation() - expected.translation())),
                np.max(
                    np.abs(X_PC.rotation().matrix() - expected.rotation().matrix())
                ),
            )

        self.assertLess(mount_error(initial_env, initial), 1e-12)
        self.assertLess(mount_error(moved_env, moved), 1e-12)
        self.assertGreater(
            np.linalg.norm(
                np.asarray(
                    moved.sensors["left_wrist_camera"].pose.translation_m
                )
                - np.asarray(
                    initial.sensors["left_wrist_camera"].pose.translation_m
                )
            ),
            1e-4,
        )

    def test_scene_replacement_changes_camera_image_not_mount(self) -> None:
        first_env = make_env(_config(dmd_name="scene.dmd.yaml"))
        second_env = make_env(_config(dmd_name="camera_variant.dmd.yaml"))
        first, _ = first_env.reset(seed=0)
        second, _ = second_env.reset(seed=0)

        first_camera = first.sensors["head_camera"]
        second_camera = second.sensors["head_camera"]
        self.assertEqual(first_camera.pose, second_camera.pose)
        self.assertFalse(np.array_equal(first_camera.rgb, second_camera.rgb))
        self.assertGreater(
            np.count_nonzero(first_camera.depth != second_camera.depth),
            0,
        )

    def test_wrist_camera_world_pose_updates_after_online_joint_action(self):
        env = make_env(_config(camera_names=("left_wrist_camera",)))
        initial, _ = env.reset(seed=0)

        moved, _, _, _, _ = env.step(
            JointDeltaAction(("left_shoulder_pitch_joint",), (0.08,))
        )

        initial_translation = np.asarray(
            initial.sensors["left_wrist_camera"].pose.translation_m
        )
        moved_translation = np.asarray(
            moved.sensors["left_wrist_camera"].pose.translation_m
        )
        self.assertGreater(
            np.linalg.norm(moved_translation - initial_translation),
            1e-4,
        )

    def test_head_camera_world_pose_follows_configured_neck_pitch(self):
        level_env = make_env(_config(camera_names=("head_camera",)))
        pitched_env = make_env(
            _config(
                camera_names=("head_camera",),
                locked_joint_position_overrides={
                    "neck_pitch_joint": 0.3,
                },
            )
        )
        level, _ = level_env.reset(seed=0)
        pitched, _ = pitched_env.reset(seed=0)

        level_pose = level.sensors["head_camera"].pose
        pitched_pose = pitched.sensors["head_camera"].pose

        self.assertGreater(
            np.linalg.norm(
                np.asarray(pitched_pose.translation_m)
                - np.asarray(level_pose.translation_m)
            ),
            1e-4,
        )
        self.assertGreater(
            np.linalg.norm(
                np.asarray(pitched_pose.quaternion_wxyz)
                - np.asarray(level_pose.quaternion_wxyz)
            ),
            1e-4,
        )

    def test_right_wrist_camera_follows_configured_right_arm_joint(self):
        initial_env = make_env(_config(camera_names=("right_wrist_camera",)))
        moved_env = make_env(
            _config(
                camera_names=("right_wrist_camera",),
                locked_joint_position_overrides={
                    "right_shoulder_pitch_joint": 0.3,
                },
            )
        )
        initial, _ = initial_env.reset(seed=0)
        moved, _ = moved_env.reset(seed=0)

        initial_pose = initial.sensors["right_wrist_camera"].pose
        moved_pose = moved.sensors["right_wrist_camera"].pose

        self.assertGreater(
            np.linalg.norm(
                np.asarray(moved_pose.translation_m)
                - np.asarray(initial_pose.translation_m)
            ),
            1e-4,
        )

    def test_disabled_cameras_preserve_state_only_observation(self) -> None:
        env = make_env(_config(camera_names=()))
        observation, _ = env.reset(seed=0)
        self.assertEqual(observation.sensors, {})
        self.assertEqual(env.backend.cameras.systems, {})
        self.assertFalse(
            env.backend.scene_graph.HasRenderer(
                env.backend.scenario.renderer.name
            )
        )


if __name__ == "__main__":
    unittest.main()
