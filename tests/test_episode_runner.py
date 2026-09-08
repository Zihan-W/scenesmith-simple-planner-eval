"""Tests for deterministic episode execution and benchmark artifacts."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from src.online_manipulation import (
    CameraIntrinsics,
    CameraObservation,
    ContactObservation,
    HoldAction,
    ObjectObservation,
    Observation,
    Pose,
    RobotObservation,
    SpatialVelocity,
    run_episode,
    run_episodes,
)


def _observation(
    time_s: float,
    *,
    saturated: bool,
    include_camera: bool = False,
) -> Observation:
    """Build one deterministic runner observation."""
    pose = Pose((time_s, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0))
    twist = SpatialVelocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    robot = RobotObservation(
        joint_names=("joint",),
        q=(time_s,),
        v=(0.0,),
        q_commanded=(time_s + 0.01,),
        torque_commanded=(2.0,),
        torque_applied=(1.0,),
        torque_saturated=(saturated,),
        end_effector_pose=pose,
        end_effector_twist=twist,
    )
    contacts = (
        ContactObservation("robot::finger", "scene::box", 0.001),
    ) if saturated else ()
    sensors = {}
    if include_camera:
        sensors["review_camera"] = CameraObservation(
            frame="review_camera",
            timestamp_s=time_s,
            pose=pose,
            intrinsics=CameraIntrinsics(
                width=2,
                height=2,
                focal_x_px=1.0,
                focal_y_px=1.0,
                center_x_px=0.5,
                center_y_px=0.5,
                fov_y_rad=1.0,
                near_m=0.1,
                far_m=5.0,
            ),
            rgb=np.full((2, 2, 3), 254, dtype=np.uint8),
            depth=np.full((2, 2), 1.25, dtype=np.float32),
            label=np.full((2, 2), 1234, dtype=np.int16),
        )
    return Observation(
        time_s=time_s,
        robot=robot,
        objects={"box": ObjectObservation(pose, twist)},
        contacts=contacts,
        task={"success": time_s >= 0.2},
        sensors=sensors,
    )


class _Policy:
    """Return hold actions and record every reset seed."""

    def __init__(self) -> None:
        self.reset_seeds = []

    def reset(self, observation, info):
        del observation
        self.reset_seeds.append(info["seed"])

    def act(self, observation):
        del observation
        return HoldAction()

    def diagnostics(self):
        """Expose one generic policy state for artifact coverage."""
        return {"stage": "hold"}


class _FailingPolicy(_Policy):
    """Raise after one completed policy period for artifact testing."""

    def __init__(self) -> None:
        super().__init__()
        self.action_count = 0

    def act(self, observation):
        del observation
        self.action_count += 1
        if self.action_count == 2:
            raise RuntimeError("intentional policy failure")
        return HoldAction()


class _TerminalDiagnosticPolicy(_Policy):
    """Expose a terminal policy reason after one environment step."""

    @property
    def stop_reason(self):
        """Use the public lifecycle signal, not a diagnostic convention."""
        return "no_safe_action"

    def diagnostics(self):
        """Return one explicit external-policy failure reason."""
        return {
            "stage": "failed",
            "failure_reason": "no_safe_action",
        }


class _Environment:
    """Two-step deterministic online environment used by runner tests."""

    def __init__(
        self,
        *,
        report_randomization: bool = False,
        include_camera: bool = False,
    ) -> None:
        self.time_s = 0.0
        self.reset_count = 0
        self.report_randomization = report_randomization
        self.include_camera = include_camera

    def reset(self, seed=None):
        self.time_s = 0.0
        self.reset_count += 1
        return _observation(
            0.0,
            saturated=False,
            include_camera=self.include_camera,
        ), {
            "seed": seed,
            "episode_randomization": (
                {"box": {"x_offset_m": 0.001 * float(seed or 0)}}
                if self.report_randomization
                else {}
            ),
            "minimum_collision_distance_m": 0.02,
            "minimum_collision_distance_is_lower_bound": False,
        }

    def step(self, action):
        del action
        self.time_s += 0.1
        done = self.time_s >= 0.2
        return (
            _observation(
                self.time_s,
                saturated=done,
                include_camera=self.include_camera,
            ),
            1.0 if done else 0.0,
            done,
            False,
            {
                "minimum_collision_distance_m": (
                    0.005 if done else 0.01
                ),
                "minimum_collision_distance_is_lower_bound": False,
                "action_decision": {
                    "status": "accepted",
                    "reasons": (),
                },
                "task": {
                    "success": done,
                    "reason": "goal_reached" if done else "running",
                },
            },
        )

    def finalize_episode(self):
        return {"success": self.time_s >= 0.2, "height_m": 0.1}

    def write_updated_scenario(self, output_path):
        Path(output_path).write_text("directives: []\n", encoding="utf-8")
        return ("box",)

    def start_recording(self):
        self.recording = True

    def save_recording(self, output_path):
        Path(output_path).write_text("<html></html>\n", encoding="utf-8")


class EpisodeRunnerTest(unittest.TestCase):
    """Validate metrics, reset behavior, and non-overwriting artifacts."""

    def test_run_episode_writes_required_artifacts_and_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "episode"
            result = run_episode(
                env=_Environment(report_randomization=True),
                policy=_Policy(),
                seed=9,
                max_steps=4,
                output_directory=output,
                record_html=True,
                write_final_dmd=True,
            )

            self.assertTrue(result.success)
            self.assertEqual(result.summary["termination_reason"], "goal_reached")
            self.assertEqual(result.summary["policy_steps"], 2)
            self.assertAlmostEqual(
                result.summary["minimum_collision_distance_m"],
                0.005,
            )
            self.assertAlmostEqual(
                result.summary["maximum_tracking_error"],
                0.01,
            )
            self.assertEqual(
                result.summary["torque_saturation"][
                    "policy_steps_with_saturation"
                ],
                1,
            )
            self.assertEqual(len(result.summary["contact_events"]), 1)
            self.assertAlmostEqual(
                result.summary["reset_info"]["episode_randomization"][
                    "box"
                ]["x_offset_m"],
                0.009,
            )
            self.assertEqual(result.summary["updated_bodies"], ["box"])
            for name in (
                "summary.json",
                "trace.csv",
                "simulation.html",
                "final.dmd.yaml",
            ):
                self.assertTrue((output / name).is_file(), name)
            summary = json.loads(
                (output / "summary.json").read_text(encoding="utf-8")
            )
            self.assertTrue(summary["success"])
            with (output / "trace.csv").open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[-1]["action_type"], "HoldAction")
            self.assertEqual(
                json.loads(rows[-1]["action_decision_json"]),
                {"status": "accepted", "reasons": []},
            )
            self.assertEqual(
                json.loads(rows[-1]["policy_diagnostics_json"]),
                {"stage": "hold"},
            )
            self.assertEqual(result.summary["policy"], {"stage": "hold"})

    def test_unsuccessful_episode_writes_failure_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "episode"
            result = run_episode(
                env=_Environment(),
                policy=_Policy(),
                seed=10,
                max_steps=1,
                output_directory=output,
            )
            self.assertFalse(result.success)
            self.assertEqual(
                result.artifact_paths["failure_json"],
                str(output / "failure.json"),
            )
            failure = json.loads(
                (output / "failure.json").read_text(encoding="utf-8")
            )
            self.assertEqual(failure["policy"], {"stage": "hold"})

    def test_policy_failure_stops_episode_and_preserves_reason(self):
        with tempfile.TemporaryDirectory() as directory:
            result = run_episode(
                env=_Environment(),
                policy=_TerminalDiagnosticPolicy(),
                seed=11,
                max_steps=10,
                output_directory=Path(directory) / "episode",
            )
            self.assertFalse(result.success)
            self.assertEqual(result.summary["policy_steps"], 1)
            self.assertTrue(result.summary["truncated"])
            self.assertEqual(
                result.summary["termination_reason"],
                "policy_failed:no_safe_action",
            )

    def test_run_episodes_resets_and_uses_unique_directories(self):
        with tempfile.TemporaryDirectory() as directory:
            env = _Environment(report_randomization=True)
            policy = _Policy()
            results = run_episodes(
                env=env,
                policy=policy,
                seeds=(3, 7),
                max_steps=2,
                output_root=Path(directory),
            )
            self.assertEqual(len(results), 2)
            self.assertEqual(env.reset_count, 2)
            self.assertEqual(policy.reset_seeds, [3, 7])
            self.assertTrue(
                (Path(directory) / "episode_000_seed_3" / "summary.json").is_file()
            )
            self.assertTrue(
                (Path(directory) / "episode_001_seed_7" / "summary.json").is_file()
            )
            aggregate = json.loads(
                (Path(directory) / "benchmark_summary.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(aggregate["episode_count"], 2)
            self.assertEqual(aggregate["success_count"], 2)
            self.assertEqual(
                aggregate["evaluation_mode"],
                "randomized_initial_state",
            )
            self.assertEqual(
                [episode["seed"] for episode in aggregate["episodes"]],
                [3, 7],
            )
            with (
                Path(directory) / "benchmark_episodes.csv"
            ).open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertAlmostEqual(
                json.loads(rows[0]["episode_randomization_json"])["box"][
                    "x_offset_m"
                ],
                0.003,
            )

    def test_camera_images_are_not_inlined_in_benchmark_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            run_episodes(
                env=_Environment(include_camera=True),
                policy=_Policy(),
                seeds=(4,),
                max_steps=2,
                output_root=output,
            )

            artifact_text = "\n".join(
                path.read_text(encoding="utf-8")
                for path in (
                    output / "benchmark_summary.json",
                    output / "benchmark_episodes.csv",
                    output / "episode_000_seed_4" / "summary.json",
                    output / "episode_000_seed_4" / "trace.csv",
                )
            )
            self.assertNotIn('"rgb"', artifact_text)
            self.assertNotIn('"depth"', artifact_text)
            self.assertNotIn('"label"', artifact_text)
            self.assertNotIn("1234", artifact_text)
            self.assertNotIn("254", artifact_text)

    def test_run_episodes_requires_at_least_one_seed(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "at least one"):
                run_episodes(
                    env=_Environment(),
                    policy=_Policy(),
                    seeds=(),
                    max_steps=2,
                    output_root=Path(directory) / "benchmark",
                )

    def test_benchmark_failure_reason_is_machine_readable(self):
        with tempfile.TemporaryDirectory() as directory:
            results = run_episodes(
                env=_Environment(),
                policy=_Policy(),
                seeds=(12,),
                max_steps=1,
                output_root=Path(directory),
            )
            self.assertFalse(results[0].success)
            aggregate = json.loads(
                (Path(directory) / "benchmark_summary.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(aggregate["failure_count"], 1)
            self.assertEqual(
                aggregate["evaluation_mode"],
                "fixed_initial_state",
            )
            self.assertEqual(
                aggregate["episodes"][0]["termination_reason"],
                "max_steps",
            )
            failure = json.loads(
                (
                    Path(directory)
                    / "episode_000_seed_12"
                    / "failure.json"
                ).read_text(encoding="utf-8")
            )
            self.assertEqual(failure["termination_reason"], "max_steps")

    def test_nonempty_output_directory_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "existing.txt").write_text("keep", encoding="utf-8")
            with self.assertRaisesRegex(FileExistsError, "not empty"):
                run_episode(
                    env=_Environment(),
                    policy=_Policy(),
                    seed=0,
                    max_steps=1,
                    output_directory=output,
                )

    def test_exception_is_reraised_after_failure_artifacts_are_saved(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "episode"
            with self.assertRaisesRegex(
                RuntimeError,
                "intentional policy failure",
            ):
                run_episode(
                    env=_Environment(),
                    policy=_FailingPolicy(),
                    seed=11,
                    max_steps=4,
                    output_directory=output,
                    record_html=True,
                )
            failure = json.loads(
                (output / "failure.json").read_text(encoding="utf-8")
            )
            self.assertEqual(failure["failed_step"], 1)
            self.assertEqual(failure["exception_type"], "RuntimeError")
            self.assertEqual(failure["policy_steps_completed"], 1)
            self.assertTrue((output / "trace.csv").is_file())
            self.assertTrue((output / "simulation.html").is_file())


if __name__ == "__main__":
    unittest.main()
