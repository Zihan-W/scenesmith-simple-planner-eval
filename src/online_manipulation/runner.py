"""Deterministic episode execution and benchmark artifact writing."""

import csv
import dataclasses
import json
import math
import traceback
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from src.online_manipulation.actions import RobotAction
from src.online_manipulation.observations import Observation
from src.online_manipulation.protocols import OnlineEnvironment, Policy


@dataclasses.dataclass(frozen=True)
class EpisodeResult:
    """One complete episode summary, trace, and optional artifacts."""

    summary: Mapping[str, Any]
    trace: tuple[Mapping[str, Any], ...]
    artifact_paths: Mapping[str, str]

    @property
    def success(self) -> bool:
        """Return the task's final success flag."""
        return bool(self.summary["success"])


def _jsonable(value: Any) -> Any:
    """Convert public dataclasses and NumPy values to JSON-compatible data."""
    if dataclasses.is_dataclass(value):
        data = {
            field.name: getattr(value, field.name)
            for field in dataclasses.fields(value)
        }
        data["type"] = type(value).__name__
        return _jsonable(data)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


def _tracking_error(observation: Observation) -> float:
    """Return maximum absolute commanded-position error at one sample."""
    return max(
        abs(position - command)
        for position, command in zip(
            observation.robot.q,
            observation.robot.q_commanded,
            strict=True,
        )
    )


def _object_poses(observation: Observation) -> dict[str, dict[str, list]]:
    """Return JSON-compatible object poses without velocities."""
    return {
        name: object_observation.pose.as_dict()
        for name, object_observation in observation.objects.items()
    }


def _contact_rows(
    observation: Observation,
    step: int,
) -> list[dict[str, Any]]:
    """Return contact events stamped at a policy boundary."""
    return [
        {
            "step": step,
            "simulation_time_s": observation.time_s,
            **contact.as_dict(),
        }
        for contact in observation.contacts
    ]


def _policy_diagnostics(policy: Policy) -> Mapping[str, Any]:
    """Return optional policy-owned diagnostics without requiring them."""
    diagnostics = getattr(policy, "diagnostics", None)
    if diagnostics is None:
        return {}
    if not callable(diagnostics):
        raise TypeError("policy.diagnostics must be callable")
    result = diagnostics()
    if not isinstance(result, Mapping):
        raise TypeError("policy.diagnostics() must return a mapping")
    return result


def _trace_row(
    *,
    step: int,
    action: RobotAction,
    observation: Observation,
    reward: float,
    terminated: bool,
    truncated: bool,
    info: Mapping[str, Any],
    policy_diagnostics: Mapping[str, Any],
) -> dict[str, Any]:
    """Build one compact, CSV-safe policy-period record."""
    return {
        "step": step,
        "simulation_time_s": observation.time_s,
        "reward": reward,
        "terminated": terminated,
        "truncated": truncated,
        "action_type": type(action).__name__,
        "action_json": json.dumps(_jsonable(action), separators=(",", ":")),
        "action_decision_json": json.dumps(
            _jsonable(info.get("action_decision")),
            separators=(",", ":"),
        ),
        "q_json": json.dumps(list(observation.robot.q)),
        "q_commanded_json": json.dumps(
            list(observation.robot.q_commanded)
        ),
        "v_json": json.dumps(list(observation.robot.v)),
        "torque_commanded_json": json.dumps(
            list(observation.robot.torque_commanded)
        ),
        "torque_applied_json": json.dumps(
            list(observation.robot.torque_applied)
        ),
        "torque_saturated_json": json.dumps(
            list(observation.robot.torque_saturated)
        ),
        "gripper_width_m": observation.robot.gripper_width_m,
        "end_effector_pose_json": json.dumps(
            observation.robot.end_effector_pose.as_dict(),
            separators=(",", ":"),
        ),
        "object_poses_json": json.dumps(
            _object_poses(observation),
            separators=(",", ":"),
        ),
        "max_tracking_error": _tracking_error(observation),
        "saturated_joint_count": sum(
            observation.robot.torque_saturated
        ),
        "contact_count": len(observation.contacts),
        "minimum_collision_distance_m": info.get(
            "minimum_collision_distance_m"
        ),
        "minimum_collision_distance_is_lower_bound": info.get(
            "minimum_collision_distance_is_lower_bound"
        ),
        "task_reason": info.get("task", {}).get("reason", "running"),
        "policy_diagnostics_json": json.dumps(
            _jsonable(policy_diagnostics),
            separators=(",", ":"),
        ),
    }


def _prepare_output_directory(output_directory: Path) -> Path:
    """Create one empty result directory without overwriting artifacts."""
    destination = Path(output_directory)
    if destination.exists() and any(destination.iterdir()):
        raise FileExistsError(
            f"Episode output directory is not empty: {destination}"
        )
    destination.mkdir(parents=True, exist_ok=True)
    return destination


def _write_trace(path: Path, trace: Sequence[Mapping[str, Any]]) -> None:
    """Write a policy-period CSV trace with stable columns."""
    if not trace:
        raise ValueError("Cannot write an empty episode trace")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def _write_exception_artifacts(
    *,
    env: OnlineEnvironment,
    destination: Path | None,
    seed: int,
    step: int,
    trace: Sequence[Mapping[str, Any]],
    error: Exception,
    traceback_text: str,
    record_html: bool,
) -> None:
    """Persist partial diagnostics for an exception, then let caller re-raise."""
    if destination is None:
        return
    payload = {
        "seed": int(seed),
        "success": False,
        "termination_reason": "exception",
        "failed_step": int(step),
        "exception_type": type(error).__name__,
        "exception_message": str(error),
        "traceback": traceback_text,
        "policy_steps_completed": len(trace),
    }
    if trace:
        _write_trace(destination / "trace.csv", trace)
    if record_html:
        try:
            env.save_recording(destination / "simulation.html")
        except Exception as recording_error:  # Preserve the original failure.
            payload["recording_error"] = (
                f"{type(recording_error).__name__}: {recording_error}"
            )
    (destination / "failure.json").write_text(
        json.dumps(_jsonable(payload), indent=2) + "\n",
        encoding="utf-8",
    )


def run_episode(
    *,
    env: OnlineEnvironment,
    policy: Policy,
    seed: int,
    max_steps: int,
    output_directory: Path | None = None,
    record_html: bool = False,
    write_final_dmd: bool = False,
) -> EpisodeResult:
    """Run one online episode and optionally persist benchmark artifacts."""
    if max_steps < 1:
        raise ValueError("max_steps must be positive")
    destination = (
        _prepare_output_directory(output_directory)
        if output_directory is not None
        else None
    )
    if record_html:
        if destination is None:
            raise ValueError("record_html requires output_directory")
        env.start_recording()

    observation, reset_info = env.reset(seed=seed)
    policy.reset(observation, reset_info)
    initial_time_s = observation.time_s
    initial_object_poses = _object_poses(observation)
    maximum_tracking_error = _tracking_error(observation)
    minimum_collision_distance = float(
        reset_info.get("minimum_collision_distance_m", math.inf)
    )
    minimum_distance_is_lower_bound = bool(
        reset_info.get("minimum_collision_distance_is_lower_bound", True)
    )
    saturated_policy_steps = 0
    saturated_joint_observations = 0
    maximum_saturated_joint_count = 0
    contact_events = _contact_rows(observation, step=-1)
    trace = []
    terminated = False
    truncated = False
    reason = "max_steps"
    last_info: Mapping[str, Any] = reset_info

    for step in range(max_steps):
        try:
            action = policy.act(observation)
            observation, reward, terminated, truncated, last_info = env.step(
                action
            )
        except Exception as error:
            _write_exception_artifacts(
                env=env,
                destination=destination,
                seed=seed,
                step=step,
                trace=trace,
                error=error,
                traceback_text=traceback.format_exc(),
                record_html=record_html,
            )
            raise
        row = _trace_row(
            step=step,
            action=action,
            observation=observation,
            reward=reward,
            terminated=terminated,
            truncated=truncated,
            info=last_info,
            policy_diagnostics=_policy_diagnostics(policy),
        )
        trace.append(row)
        maximum_tracking_error = max(
            maximum_tracking_error,
            float(row["max_tracking_error"]),
        )
        saturated_count = int(row["saturated_joint_count"])
        saturated_policy_steps += int(saturated_count > 0)
        saturated_joint_observations += saturated_count
        maximum_saturated_joint_count = max(
            maximum_saturated_joint_count,
            saturated_count,
        )
        distance = row["minimum_collision_distance_m"]
        if distance is not None and float(distance) < minimum_collision_distance:
            minimum_collision_distance = float(distance)
            minimum_distance_is_lower_bound = bool(
                row["minimum_collision_distance_is_lower_bound"]
            )
        contact_events.extend(_contact_rows(observation, step=step))
        policy_failure = _policy_diagnostics(policy).get("failure_reason")
        if policy_failure:
            reason = f"policy_failed:{policy_failure}"
            truncated = True
            break
        if terminated:
            reason = last_info.get("task", {}).get("reason", "terminated")
            break
        if truncated:
            reason = "time_limit"
            break
    else:
        truncated = True

    final_task = env.finalize_episode()
    success = bool(final_task.get("success", False))
    if terminated and reason == "running":
        reason = "terminated"
    summary = {
        "seed": int(seed),
        "success": success,
        "termination_reason": reason,
        "terminated": terminated,
        "truncated": truncated,
        "episode_time_s": observation.time_s - initial_time_s,
        "policy_steps": len(trace),
        "minimum_collision_distance_m": (
            minimum_collision_distance
            if math.isfinite(minimum_collision_distance)
            else None
        ),
        "minimum_collision_distance_is_lower_bound": (
            minimum_distance_is_lower_bound
        ),
        "maximum_tracking_error": maximum_tracking_error,
        "torque_saturation": {
            "policy_steps_with_saturation": saturated_policy_steps,
            "joint_saturation_observations": (
                saturated_joint_observations
            ),
            "maximum_simultaneously_saturated_joints": (
                maximum_saturated_joint_count
            ),
        },
        "contact_events": contact_events,
        "reset_info": _jsonable(reset_info),
        "initial_object_poses": initial_object_poses,
        "final_object_poses": _object_poses(observation),
        "task": _jsonable(final_task),
        "policy": _jsonable(_policy_diagnostics(policy)),
    }
    artifacts: dict[str, str] = {}
    if destination is not None:
        summary_path = destination / "summary.json"
        trace_path = destination / "trace.csv"
        summary_path.write_text(
            json.dumps(_jsonable(summary), indent=2) + "\n",
            encoding="utf-8",
        )
        _write_trace(trace_path, trace)
        artifacts.update(
            summary_json=str(summary_path),
            trace_csv=str(trace_path),
        )
        if record_html:
            html_path = destination / "simulation.html"
            env.save_recording(html_path)
            artifacts["simulation_html"] = str(html_path)
        if write_final_dmd:
            dmd_path = destination / "final.dmd.yaml"
            updated_bodies = env.write_updated_scenario(dmd_path)
            artifacts["final_dmd"] = str(dmd_path)
            summary["updated_bodies"] = list(updated_bodies)
            summary_path.write_text(
                json.dumps(_jsonable(summary), indent=2) + "\n",
                encoding="utf-8",
            )
        if not success:
            failure_path = destination / "failure.json"
            failure_path.write_text(
                json.dumps(_jsonable(summary), indent=2) + "\n",
                encoding="utf-8",
            )
            artifacts["failure_json"] = str(failure_path)
    return EpisodeResult(
        summary=summary,
        trace=tuple(trace),
        artifact_paths=artifacts,
    )


def run_episodes(
    *,
    env: OnlineEnvironment,
    policy: Policy,
    seeds: Sequence[int],
    max_steps: int,
    output_root: Path,
    record_html: bool = False,
    write_final_dmd: bool = False,
) -> tuple[EpisodeResult, ...]:
    """Run reset-isolated episodes and write machine-readable aggregates."""
    if not seeds:
        raise ValueError("seeds must contain at least one episode seed")
    destination = _prepare_output_directory(output_root)
    results = []
    for index, seed in enumerate(seeds):
        episode_directory = destination / (
            f"episode_{index:03d}_seed_{int(seed)}"
        )
        results.append(
            run_episode(
                env=env,
                policy=policy,
                seed=int(seed),
                max_steps=max_steps,
                output_directory=episode_directory,
                record_html=record_html,
                write_final_dmd=write_final_dmd,
            )
        )
    result_tuple = tuple(results)
    aggregate_rows = [
        {
            "episode_index": index,
            "seed": result.summary["seed"],
            "success": result.summary["success"],
            "termination_reason": result.summary["termination_reason"],
            "policy_steps": result.summary["policy_steps"],
            "episode_time_s": result.summary["episode_time_s"],
            "minimum_collision_distance_m": result.summary[
                "minimum_collision_distance_m"
            ],
            "maximum_tracking_error": result.summary[
                "maximum_tracking_error"
            ],
            "randomized_initial_state": bool(
                result.summary["reset_info"].get(
                    "episode_randomization",
                    {},
                )
            ),
            "episode_randomization_json": json.dumps(
                result.summary["reset_info"].get(
                    "episode_randomization",
                    {},
                ),
                separators=(",", ":"),
            ),
            "initial_object_poses_json": json.dumps(
                result.summary["initial_object_poses"],
                separators=(",", ":"),
            ),
        }
        for index, result in enumerate(result_tuple)
    ]
    aggregate_csv = destination / "benchmark_episodes.csv"
    with aggregate_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=tuple(aggregate_rows[0].keys()),
        )
        writer.writeheader()
        writer.writerows(aggregate_rows)
    randomization_flags = {
        row["randomized_initial_state"] for row in aggregate_rows
    }
    if randomization_flags == {True}:
        evaluation_mode = "randomized_initial_state"
    elif randomization_flags == {False}:
        evaluation_mode = "fixed_initial_state"
    else:
        evaluation_mode = "mixed_initial_state"
    aggregate = {
        "evaluation_mode": evaluation_mode,
        "episode_count": len(result_tuple),
        "success_count": sum(result.success for result in result_tuple),
        "failure_count": sum(not result.success for result in result_tuple),
        "success_rate": (
            sum(result.success for result in result_tuple)
            / len(result_tuple)
        ),
        "episodes": aggregate_rows,
    }
    (destination / "benchmark_summary.json").write_text(
        json.dumps(_jsonable(aggregate), indent=2) + "\n",
        encoding="utf-8",
    )
    return result_tuple
