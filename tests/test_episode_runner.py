"""Tests for deterministic episode execution and benchmark artifacts."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from src.online_manipulation import (
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


def _observation(time_s: float, *, saturated: bool) -> Observation:
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
    return Observation(
        time_s=time_s,
        robot=robot,
        objects={"box": ObjectObservation(pose, twist)},
        contacts=contacts,
        task={"success": time_s >= 0.2},
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


class _Environment:
    """Two-step deterministic online environment used by runner tests."""

    def __init__(self) -> None:
        self.time_s = 0.0
        self.reset_count = 0

    def reset(self, seed=None):
        self.time_s = 0.0
        self.reset_count += 1
        return _observation(0.0, saturated=False), {
            "seed": seed,
            "minimum_collision_distance_m": 0.02,
            "minimum_collision_distance_is_lower_bound": False,
        }

    def step(self, action):
        del action
        self.time_s += 0.1
        done = self.time_s >= 0.2
        return (
            _observation(self.time_s, saturated=done),
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
                env=_Environment(),
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

    def test_run_episodes_resets_and_uses_unique_directories(self):
        with tempfile.TemporaryDirectory() as directory:
            env = _Environment()
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
