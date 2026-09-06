"""Real-Drake handoff tests for scene replacement, reset, and DMD output."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from src.online_manipulation import (
    HoldPolicy,
    JointDeltaAction,
    NullTask,
    ObservedBodySpec,
    PlanarPoseRandomizationSpec,
    ScenarioSpec,
    TimingConfig,
    ZerithEnvironmentConfig,
    ZerithRobotAdapter,
    build_planning_query,
    make_env,
    make_zerith_robot_spec,
    run_episodes,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SCENE_ROOT = REPOSITORY_ROOT / "models" / "online_env_minimal_scene"
ROBOT_MODEL_DIR = REPOSITORY_ROOT / "models" / "zerith_drake"
OBJECT_NAME = "portable_object"


def _config(
    *,
    dmd_path: Path | None = None,
    randomize: bool = False,
) -> ZerithEnvironmentConfig:
    """Return a public config for the portable minimal scene."""
    randomizations = ()
    if randomize:
        randomizations = (
            PlanarPoseRandomizationSpec(
                observation_name=OBJECT_NAME,
                x_offset_range_m=(-0.01, 0.01),
                y_offset_range_m=(-0.01, 0.01),
                yaw_offset_range_rad=(-0.05, 0.05),
            ),
        )
    return ZerithEnvironmentConfig(
        scenario=ScenarioSpec(
            dmd_path=dmd_path or SCENE_ROOT / "scene.dmd.yaml",
            package_xmls=(SCENE_ROOT / "package.xml",),
            observed_bodies=(
                ObservedBodySpec(
                    observation_name=OBJECT_NAME,
                    model_instance_name="portable_test_box",
                    body_name="base_link",
                    write_back=True,
                ),
            ),
            pose_randomizations=randomizations,
        ),
        robot_model_dir=ROBOT_MODEL_DIR,
        robot_xyz=(0.0, 0.0, 0.2315),
        robot_yaw_deg=0.0,
        rail_position=0.4,
        q_home_left=(0.0,) * 7,
        episode_duration=1.0,
        task=NullTask(),
    )


class OnlineHandoffTest(unittest.TestCase):
    """Exercise public handoff behavior against a second real DMD scene."""

    def test_minimal_scene_default_planning_state_has_no_penetration(self):
        config = _config()
        adapter = ZerithRobotAdapter(
            make_zerith_robot_spec(
                robot_model_dir=config.robot_model_dir,
                robot_xyz=config.robot_xyz,
                robot_yaw_deg=config.robot_yaw_deg,
                rail_position=config.rail_position,
                q_home_left=config.q_home_left,
            )
        )
        query = build_planning_query(
            scenario=config.scenario,
            robot_adapter=adapter,
            timing=TimingConfig(),
        )

        check = query.check_configuration(adapter.spec.home_positions)

        self.assertTrue(check.valid)
        self.assertGreaterEqual(
            check.clearance.minimum_nonpenetration_distance_m,
            0.0,
        )

    def test_scene_replacement_runs_adapter_task_and_hold_policy(self):
        env = make_env(_config())
        policy = HoldPolicy()
        observation, info = env.reset(seed=0)
        policy.reset(observation, info)
        action = policy.act(observation)
        observation, reward, terminated, truncated, step_info = env.step(
            action
        )

        self.assertEqual(set(observation.objects), {OBJECT_NAME})
        self.assertEqual(len(observation.robot.joint_names), 9)
        self.assertAlmostEqual(observation.time_s, 0.1)
        self.assertEqual(reward, 0.0)
        self.assertFalse(terminated)
        self.assertFalse(truncated)
        self.assertEqual(step_info["action_decision"]["status"], "accepted")
        self.assertEqual(observation.task["task_name"], "null")

    def test_seeded_reset_restores_robot_controller_and_free_body(self):
        env = make_env(_config(randomize=True))
        first, first_info = env.reset(seed=17)
        initial_state = (
            first.robot.q,
            first.robot.q_commanded,
            first.objects[OBJECT_NAME].pose,
        )
        env.step(
            JointDeltaAction(
                joint_names=(first.robot.joint_names[0],),
                deltas=(0.02,),
            )
        )

        repeated, repeated_info = env.reset(seed=17)
        repeated_state = (
            repeated.robot.q,
            repeated.robot.q_commanded,
            repeated.objects[OBJECT_NAME].pose,
        )
        self.assertEqual(repeated_state, initial_state)
        self.assertEqual(
            repeated_info["episode_randomization"],
            first_info["episode_randomization"],
        )

        changed, changed_info = env.reset(seed=18)
        self.assertNotEqual(
            changed_info["episode_randomization"],
            first_info["episode_randomization"],
        )
        self.assertNotEqual(
            changed.objects[OBJECT_NAME].pose,
            first.objects[OBJECT_NAME].pose,
        )

    def test_headless_benchmark_writes_isolated_machine_readable_results(self):
        env = make_env(_config(randomize=True))
        with tempfile.TemporaryDirectory() as directory:
            output_root = Path(directory) / "benchmark"
            results = run_episodes(
                env=env,
                policy=HoldPolicy(),
                seeds=(31, 32),
                max_steps=1,
                output_root=output_root,
            )
            self.assertEqual(len(results), 2)
            self.assertTrue(
                (
                    output_root
                    / "episode_000_seed_31"
                    / "summary.json"
                ).is_file()
            )
            self.assertTrue(
                (
                    output_root
                    / "episode_001_seed_32"
                    / "failure.json"
                ).is_file()
            )
            aggregate = json.loads(
                (output_root / "benchmark_summary.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(
                aggregate["evaluation_mode"],
                "randomized_initial_state",
            )
            self.assertEqual(aggregate["failure_count"], 2)
            self.assertEqual(
                aggregate["episodes"][0]["termination_reason"],
                "max_steps",
            )
            self.assertNotEqual(
                aggregate["episodes"][0]["initial_object_poses_json"],
                aggregate["episodes"][1]["initial_object_poses_json"],
            )
            with (
                output_root / "benchmark_episodes.csv"
            ).open(encoding="utf-8") as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 2)

    def test_final_dmd_is_new_reloadable_and_pose_round_trips(self):
        source = SCENE_ROOT / "scene.dmd.yaml"
        source_bytes = source.read_bytes()
        env = make_env(_config(randomize=True))
        observation, _ = env.reset(seed=23)
        observation, _, _, _, _ = env.step(HoldPolicy().act(observation))

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "final.dmd.yaml"
            updated = env.write_updated_scenario(output)
            self.assertEqual(updated, (OBJECT_NAME,))
            self.assertEqual(source.read_bytes(), source_bytes)
            self.assertTrue(output.is_file())

            reloaded_env = make_env(_config(dmd_path=output))
            reloaded, _ = reloaded_env.reset(seed=999)

        expected = observation.objects[OBJECT_NAME].pose
        actual = reloaded.objects[OBJECT_NAME].pose
        np.testing.assert_allclose(
            actual.translation_m,
            expected.translation_m,
            atol=1e-12,
        )
        quaternion_dot = abs(
            float(
                np.dot(
                    actual.quaternion_wxyz,
                    expected.quaternion_wxyz,
                )
            )
        )
        self.assertAlmostEqual(quaternion_dot, 1.0, places=12)


if __name__ == "__main__":
    unittest.main()
