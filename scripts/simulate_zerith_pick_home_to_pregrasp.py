#!/usr/bin/env python3
"""Drive Zerith online from q_pick_home to PREGRASP."""

import argparse
import json
import sys

from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.zerith_grasp_geometry import (
    PICK_RAIL_POSITION_METERS,
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)
from src.online_manipulation.adapters.zerith import (
    ZerithRobotAdapter,
    make_legacy_zerith_online_environment,
    make_zerith_robot_spec,
)
from src.online_manipulation.actions import (
    CompositeAction,
    GripperAction,
    JointDeltaAction,
)
from src.online_manipulation.specs import (
    ObservedBodySpec,
    ScenarioSpec,
    TimingConfig,
    VisualizationConfig,
)
from src.zerith_online_env import LEFT_ARM_SERVO_CONFIGS
from src.zerith_pick_workspace import (
    PickWorkspaceEvaluator,
    edge_workspace_metrics,
)
from src.zerith_pregrasp_collision import (
    apply_safety_clearance_filters,
    build_pregrasp_planning_model,
    configuration_clearance_metrics,
    edge_clearance_metrics,
)

EVAL_PACKAGE_XML = (
    REPOSITORY_ROOT / "models" / "zerith_pick_eval" / "package.xml"
)
ZERITH_MODEL_DIR = REPOSITORY_ROOT / "models" / "zerith_drake"
TARGET_OBSERVATION_NAME = "pick_target"


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Run a 10 Hz bounded-increment policy from q_pick_home to "
            "PREGRASP over a prevalidated direct edge."
        )
    )
    parser.add_argument("scene_dmd", type=Path)
    parser.add_argument("--scene-package-xml", type=Path, required=True)
    parser.add_argument("--pick-home-json", type=Path, required=True)
    parser.add_argument(
        "--eval-package-xml",
        type=Path,
        default=EVAL_PACKAGE_XML,
    )
    parser.add_argument(
        "--robot-model-dir",
        type=Path,
        default=ZERITH_MODEL_DIR,
    )
    parser.add_argument(
        "--table-model-name",
        default="living_room_coffee_table_0",
    )
    parser.add_argument(
        "--rail-position",
        type=float,
        default=PICK_RAIL_POSITION_METERS,
    )
    parser.add_argument("--policy-dt", type=float, default=0.1)
    parser.add_argument("--controller-dt", type=float, default=0.005)
    parser.add_argument("--physics-dt", type=float, default=0.001)
    parser.add_argument("--maximum-policy-steps", type=int, default=300)
    parser.add_argument("--maximum-joint-step", type=float, default=0.01)
    parser.add_argument("--edge-sample-step", type=float, default=0.002)
    parser.add_argument(
        "--safety-objective-clearance",
        type=float,
        default=0.007,
    )
    parser.add_argument(
        "--safety-acceptance-clearance",
        type=float,
        default=0.005,
    )
    parser.add_argument(
        "--workspace-safety-margin",
        type=float,
        default=0.005,
    )
    parser.add_argument(
        "--influence-distance-offset",
        type=float,
        default=0.01,
    )
    parser.add_argument("--joint-goal-tolerance", type=float, default=0.02)
    parser.add_argument("--velocity-goal-tolerance", type=float, default=0.05)
    parser.add_argument("--required-stable-steps", type=int, default=5)
    parser.add_argument(
        "--maximum-tracking-error",
        type=float,
        default=0.08,
    )
    parser.add_argument(
        "--maximum-continuous-saturation-duration",
        type=float,
        default=0.25,
    )
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--realtime-rate", type=float, default=0.0)
    parser.add_argument("--meshcat", action="store_true")
    parser.add_argument("--meshcat-port", type=int)
    parser.add_argument("--record-html", type=Path)
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Defaults to online_pregrasp.json beside pick-home-json.",
    )
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    """Validate control timing and safety parameters."""
    positive_fields = (
        "policy_dt",
        "controller_dt",
        "physics_dt",
        "maximum_joint_step",
        "edge_sample_step",
        "safety_objective_clearance",
        "safety_acceptance_clearance",
        "workspace_safety_margin",
        "influence_distance_offset",
        "joint_goal_tolerance",
        "velocity_goal_tolerance",
        "maximum_tracking_error",
        "maximum_continuous_saturation_duration",
    )
    for name in positive_fields:
        if getattr(args, name) <= 0.0:
            raise ValueError(f"{name.replace('_', '-')} must be positive")
    if (
        args.safety_objective_clearance
        < args.safety_acceptance_clearance
    ):
        raise ValueError(
            "safety-objective-clearance must be at least "
            "safety-acceptance-clearance"
        )
    if args.maximum_policy_steps < 1:
        raise ValueError("maximum-policy-steps must be positive")
    if args.required_stable_steps < 1:
        raise ValueError("required-stable-steps must be positive")
    if args.episodes < 1:
        raise ValueError("episodes must be positive")


def _load_targets(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load q_pick_home and q_pregrasp from search output."""
    payload = json.loads(path.read_text(encoding="utf-8"))
    q_pick_home = np.asarray(payload["q_pick_home"], dtype=float)
    q_pregrasp = np.asarray(payload["q_pregrasp"], dtype=float)
    for name, value in (
        ("q_pick_home", q_pick_home),
        ("q_pregrasp", q_pregrasp),
    ):
        if value.shape != (7,) or not np.all(np.isfinite(value)):
            raise ValueError(f"pick-home-json contains an invalid {name}")
    return q_pick_home, q_pregrasp


def _full_configuration(model, q_left: np.ndarray) -> np.ndarray:
    """Insert a left-arm posture into the fixed planning scene."""
    q = model.q_scene.copy()
    q[model.arm_indices] = q_left
    return q


def _maximum_saturation_run(samples, controller_dt: float) -> float:
    """Return the longest continuous arm torque-saturation duration."""
    current = np.zeros(7, dtype=int)
    maximum = np.zeros(7, dtype=int)
    for sample in samples:
        current = np.where(sample.saturated[:7], current + 1, 0)
        maximum = np.maximum(maximum, current)
    return float(np.max(maximum) * controller_dt)


def _runtime_metrics(model, workspace, q_left, args) -> tuple[dict, dict]:
    """Return both collision layers and whole-wrist workspace metrics."""
    q = _full_configuration(model, q_left)
    diagnostic_distance = max(
        0.05,
        args.safety_objective_clearance
        + args.influence_distance_offset,
    )
    return (
        configuration_clearance_metrics(model, q, diagnostic_distance),
        workspace.evaluate(q, args.workspace_safety_margin),
    )


def _grasp_frame_error(model, q_left, q_goal) -> dict:
    """Return grasp-frame pose error between two arm configurations."""
    model.plant.SetPositions(
        model.plant_context,
        _full_configuration(model, q_left),
    )
    actual_pose = model.plant.CalcRelativeTransform(
        model.plant_context,
        model.plant.world_frame(),
        model.grasp_frame,
    )
    model.plant.SetPositions(
        model.plant_context,
        _full_configuration(model, q_goal),
    )
    goal_pose = model.plant.CalcRelativeTransform(
        model.plant_context,
        model.plant.world_frame(),
        model.grasp_frame,
    )
    orientation_error = (
        goal_pose.rotation().inverse() @ actual_pose.rotation()
    ).ToAngleAxis().angle()
    return {
        "position_error_m": float(
            np.linalg.norm(
                actual_pose.translation() - goal_pose.translation()
            )
        ),
        "orientation_error_deg": float(np.rad2deg(orientation_error)),
    }


def _run_episode(env, model, workspace, q_pregrasp, args) -> dict:
    """Execute one observation-driven online PREGRASP episode."""
    observation, _ = env.reset()
    initial_target_translation = np.asarray(
        observation.objects[TARGET_OBSERVATION_NAME].pose.translation_m
    )
    desired_q = np.asarray(observation.robot.q[:7])
    stable_steps = 0
    trace = []
    for step_index in range(args.maximum_policy_steps):
        q_current = np.asarray(observation.robot.q[:7])
        collision, workspace_metrics = _runtime_metrics(
            model,
            workspace,
            q_current,
            args,
        )
        if (
            collision["minimum_nonpenetration_distance"] < 0.0
            or collision["minimum_safety_clearance"]
            < args.safety_acceptance_clearance
        ):
            return {
                "success": False,
                "reason": "runtime_clearance_below_acceptance",
                "steps": step_index,
                "q_left": q_current.tolist(),
                "desired_q_left": desired_q.tolist(),
                "clearance_metrics": collision,
                "workspace_metrics": workspace_metrics,
                "trace": trace,
            }
        if not workspace_metrics["satisfied"]:
            return {
                "success": False,
                "reason": "runtime_workspace_constraint_violated",
                "steps": step_index,
                "trace": trace,
            }

        goal_error = q_pregrasp - q_current
        maximum_goal_error = float(np.max(np.abs(goal_error)))
        maximum_speed = float(np.max(np.abs(observation.robot.v[:7])))
        if (
            maximum_goal_error <= args.joint_goal_tolerance
            and maximum_speed <= args.velocity_goal_tolerance
        ):
            stable_steps += 1
        else:
            stable_steps = 0
        if stable_steps >= args.required_stable_steps:
            minimum_nonpenetration_distance = min(
                [collision["minimum_nonpenetration_distance"]]
                + [
                    item["minimum_nonpenetration_distance"]
                    for item in trace
                ]
            )
            minimum_safety_clearance = min(
                [collision["minimum_safety_clearance"]]
                + [item["minimum_safety_clearance"] for item in trace]
            )
            maximum_tracking_error = max(
                [0.0]
                + [item["maximum_tracking_error_rad"] for item in trace]
            )
            maximum_saturation_run = max(
                [0.0]
                + [item["maximum_saturation_run_s"] for item in trace]
            )
            return {
                "success": True,
                "reason": "pregrasp_reached",
                "steps": step_index,
                "q_left": q_current.tolist(),
                "desired_q_left": desired_q.tolist(),
                "gripper_width_m": observation.robot.gripper_width_m,
                "grasp_frame_error": _grasp_frame_error(
                    model,
                    q_current,
                    q_pregrasp,
                ),
                "maximum_joint_error_rad": maximum_goal_error,
                "maximum_joint_speed_rad_per_s": maximum_speed,
                "maximum_tracking_error_rad": maximum_tracking_error,
                "maximum_continuous_saturation_duration_s": (
                    maximum_saturation_run
                ),
                "minimum_runtime_nonpenetration_distance_m": (
                    minimum_nonpenetration_distance
                ),
                "minimum_runtime_safety_clearance_m": (
                    minimum_safety_clearance
                ),
                "red_box_translation_m": float(
                    np.linalg.norm(
                        np.asarray(
                            observation.objects[
                                TARGET_OBSERVATION_NAME
                            ].pose.translation_m
                        )
                        - initial_target_translation
                    )
                ),
                "final_clearance_metrics": collision,
                "final_workspace_metrics": workspace_metrics,
                "trace": trace,
            }

        tracking_error = float(np.max(np.abs(desired_q - q_current)))
        if tracking_error > args.maximum_tracking_error:
            q_command = desired_q.copy()
            command_result = "hold_for_servo_tracking"
        else:
            q_command = desired_q + np.clip(
                q_pregrasp - desired_q,
                -args.maximum_joint_step,
                args.maximum_joint_step,
            )
            command_result = "bounded_target_increment"

        edge = edge_clearance_metrics(
            model,
            _full_configuration(model, q_current),
            _full_configuration(model, q_command),
            max(
                0.05,
                args.safety_objective_clearance
                + args.influence_distance_offset,
            ),
            max_joint_step=args.edge_sample_step,
        )
        workspace_edge = edge_workspace_metrics(
            workspace,
            _full_configuration(model, q_current),
            _full_configuration(model, q_command),
            args.workspace_safety_margin,
            args.edge_sample_step,
        )
        if (
            edge["minimum_nonpenetration_distance"] < 0.0
            or edge["minimum_safety_clearance"]
            < args.safety_acceptance_clearance
            or not workspace_edge["workspace_constraint_satisfied"]
        ):
            return {
                "success": False,
                "reason": "policy_action_edge_rejected",
                "steps": step_index,
                "rejected_edge": edge,
                "rejected_workspace_edge": workspace_edge,
                "trace": trace,
            }

        desired_q_before_action = desired_q.copy()
        log_start = len(env.backend.control_log)
        action = CompositeAction(
            arm=JointDeltaAction(
                tuple(config.name for config in LEFT_ARM_SERVO_CONFIGS),
                tuple(q_command - desired_q),
            ),
            gripper=GripperAction(width_m=0.08),
        )
        observation, _, _, _, info = env.step(action)
        expected_control_updates = round(args.policy_dt / args.controller_dt)
        expected_physics_steps = round(args.controller_dt / args.physics_dt)
        if (
            info["control_updates"] != expected_control_updates
            or info["physics_steps_per_control"] != expected_physics_steps
        ):
            return {
                "success": False,
                "reason": "multirate_schedule_contract_violated",
                "steps": step_index + 1,
                "info": info,
                "trace": trace,
            }
        desired_q = info["desired_q_left"]
        new_control_samples = env.backend.control_log[log_start:]
        saturation_run = _maximum_saturation_run(
            env.backend.control_log,
            args.controller_dt,
        )
        saturation_counts = {
            config.name: int(
                sum(sample.saturated[index] for sample in new_control_samples)
            )
            for index, config in enumerate(LEFT_ARM_SERVO_CONFIGS)
        }
        trace.append(
            {
                "step": step_index,
                "simulation_time_s": observation.time_s,
                "command_result": command_result,
                "control_updates": info["control_updates"],
                "physics_steps_per_control": info[
                    "physics_steps_per_control"
                ],
                "q_left_before_action": q_current.tolist(),
                "desired_q_left_before_action": (
                    desired_q_before_action.tolist()
                ),
                "commanded_q_left": q_command.tolist(),
                "q_left_after_action": list(observation.robot.q[:7]),
                "v_left_after_action": list(observation.robot.v[:7]),
                "maximum_joint_goal_error_rad": maximum_goal_error,
                "maximum_tracking_error_rad": tracking_error,
                "minimum_nonpenetration_distance": collision[
                    "minimum_nonpenetration_distance"
                ],
                "minimum_safety_clearance": collision[
                    "minimum_safety_clearance"
                ],
                "workspace_constraint_satisfied": workspace_metrics[
                    "satisfied"
                ],
                "action_edge_metrics": edge,
                "action_workspace_edge_metrics": workspace_edge,
                "maximum_saturation_run_s": saturation_run,
                "saturation_sample_counts": saturation_counts,
            }
        )
        if saturation_run > args.maximum_continuous_saturation_duration:
            return {
                "success": False,
                "reason": "continuous_torque_saturation",
                "steps": step_index + 1,
                "trace": trace,
            }
        penetrations = env.backend.robot_penetrations()
        if penetrations:
            return {
                "success": False,
                "reason": "dynamics_robot_penetration",
                "steps": step_index + 1,
                "q_left": list(observation.robot.q[:7]),
                "desired_q_left": desired_q.tolist(),
                "penetrations": [vars(item) for item in penetrations],
                "trace": trace,
            }
    return {
        "success": False,
        "reason": "maximum_policy_steps",
        "steps": args.maximum_policy_steps,
        "trace": trace,
    }


def main() -> None:
    """Build planning and dynamics models and run repeatable episodes."""
    args = _parse_args()
    _validate_args(args)
    scene_dmd = args.scene_dmd.resolve()
    scene_package_xml = args.scene_package_xml.resolve()
    eval_package_xml = args.eval_package_xml.resolve()
    robot_model_dir = args.robot_model_dir.resolve()
    pick_home_json = args.pick_home_json.resolve()
    q_pick_home, q_pregrasp = _load_targets(pick_home_json)

    model = build_pregrasp_planning_model(
        scene_dmd=scene_dmd,
        scene_package_xml=scene_package_xml,
        eval_package_xml=eval_package_xml,
        robot_model_dir=robot_model_dir,
        robot_xyz=ROBOT_BASE_XYZ_METERS,
        robot_yaw_deg=ROBOT_BASE_YAW_DEG,
        rail_position=args.rail_position,
    )
    apply_safety_clearance_filters(
        model,
        influence_distance=(
            args.safety_objective_clearance
            + args.influence_distance_offset
        ),
    )
    workspace = PickWorkspaceEvaluator(
        model=model,
        table_model_name=args.table_model_name,
    )
    full_edge = edge_clearance_metrics(
        model,
        _full_configuration(model, q_pick_home),
        _full_configuration(model, q_pregrasp),
        max(
            0.05,
            args.safety_objective_clearance
            + args.influence_distance_offset,
        ),
        max_joint_step=args.edge_sample_step,
    )
    full_workspace_edge = edge_workspace_metrics(
        workspace,
        _full_configuration(model, q_pick_home),
        _full_configuration(model, q_pregrasp),
        args.workspace_safety_margin,
        args.edge_sample_step,
    )
    if (
        full_edge["minimum_nonpenetration_distance"] < 0.0
        or full_edge["minimum_safety_clearance"]
        < args.safety_acceptance_clearance
        or not full_workspace_edge["workspace_constraint_satisfied"]
    ):
        raise ValueError(
            "q_pick_home-to-PREGRASP failed independent dense edge "
            "validation"
        )

    robot_spec = make_zerith_robot_spec(
        robot_model_dir=robot_model_dir,
        robot_xyz=ROBOT_BASE_XYZ_METERS,
        robot_yaw_deg=ROBOT_BASE_YAW_DEG,
        rail_position=args.rail_position,
        q_home_left=q_pick_home,
    )
    adapter = ZerithRobotAdapter(robot_spec)
    scenario = ScenarioSpec(
        dmd_path=scene_dmd,
        package_xmls=(scene_package_xml, eval_package_xml),
        observed_bodies=(
            ObservedBodySpec(
                observation_name=TARGET_OBSERVATION_NAME,
                model_instance_name="living_room_box_0",
                body_name="base_link",
            ),
        ),
        visualization=VisualizationConfig(
            enabled=args.meshcat,
            port=args.meshcat_port,
            record_html_path=args.record_html,
            realtime_rate=args.realtime_rate,
        ),
    )
    timing = TimingConfig(
        physics_dt=args.physics_dt,
        controller_dt=args.controller_dt,
        policy_dt=args.policy_dt,
    )
    env = make_legacy_zerith_online_environment(
        scenario=scenario,
        adapter=adapter,
        timing=timing,
        target_model_name="living_room_box_0",
        episode_duration=(
            args.maximum_policy_steps + args.required_stable_steps + 1
        )
        * args.policy_dt,
        max_joint_delta=args.maximum_joint_step,
    )
    meshcat = env.backend.runtime.meshcat
    if meshcat is not None:
        print(f"Meshcat URL: {meshcat.web_url()}")
        meshcat.StartRecording()

    episodes = []
    for episode_index in range(args.episodes):
        result = _run_episode(
            env,
            model,
            workspace,
            q_pregrasp,
            args,
        )
        result["episode"] = episode_index
        episodes.append(result)
        print(
            f"Episode {episode_index}: {result['reason']} "
            f"after {result['steps']} policy steps",
            flush=True,
        )
        if not result["success"]:
            break

    if meshcat is not None:
        meshcat.StopRecording()
        meshcat.PublishRecording()
        if args.record_html is not None:
            args.record_html.resolve().write_text(
                meshcat.StaticHtml(),
                encoding="utf-8",
            )

    passed = len(episodes) == args.episodes and all(
        episode["success"] for episode in episodes
    )
    output = {
        "passed": passed,
        "calibration_status": (
            "fixed_rail_online_pregrasp_validated"
        ),
        "daogui_joint_position_m": args.rail_position,
        "control_architecture": {
            "policy_frequency_hz": 1.0 / args.policy_dt,
            "servo_frequency_hz": 1.0 / args.controller_dt,
            "physics_frequency_hz": 1.0 / args.physics_dt,
            "offline_trajectory_used": False,
            "rrt_used": False,
            "toppra_used": False,
        },
        "q_pick_home": q_pick_home.tolist(),
        "q_pregrasp": q_pregrasp.tolist(),
        "prevalidated_collision_edge": full_edge,
        "prevalidated_workspace_edge": full_workspace_edge,
        "episodes_requested": args.episodes,
        "episodes_completed": len(episodes),
        "episodes": episodes,
    }
    output_path = (
        args.output_json.resolve()
        if args.output_json is not None
        else pick_home_json.parent / "online_pregrasp.json"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(f"Diagnostics: {output_path}")
    if not passed:
        raise RuntimeError("Online q_pick_home-to-PREGRASP validation failed")
    print("PASS: online policy reached PREGRASP reproducibly")


if __name__ == "__main__":
    main()
