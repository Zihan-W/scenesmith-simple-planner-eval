"""Verification-only base causality experiment; no runtime/default changes.

The optional actuator cut is local to this process and applied before reset.
Simulator monitors read real plant state and the actual actuator input at every
physics step. This diagnostic deliberately uses backend internals, unlike a
Policy: it does not write any pose, velocity, constraint or physical parameter.
"""

import argparse
import csv
import dataclasses
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from numpy.lib import recfunctions

from src.online_manipulation.recipes.mobile import make_config
from src.online_manipulation import (
    BaseVelocityAction, NavigationGoal, Navigator, Pose,
    build_navigation_map, make_env,
)


@dataclasses.dataclass(frozen=True)
class AuditSettings:
    """Independent experiment configuration; not a production control mode."""

    disable_drive_torque: bool = False


class Recorder:
    """Read physics-step states and applied input, without changing dynamics."""

    def __init__(self, env, settings):
        self.runtime = env.backend
        self.phase = "initial"
        self.command = (0.0, 0.0)
        self.rows = []
        self.prescribed_writes = []
        self.monitor_calls = []
        r = self.runtime
        self.frame = r.plant.GetFrameByName(r.adapter.navigation_frame_name, r.instance)
        self.body = r.plant.GetBodyByName(r.spec.base_link_name, r.instance)
        self.wheels = {}
        if r.adapter.base_mode == "planar_kinematic":
            original_prescribed = r.base_backend.before_physics

            def record_prescribed(dt):
                joint = r.plant.GetJointByName(r.adapter.planar_joint_name, r.instance)
                t = r.plant_context.get_time()
                before = np.r_[joint.get_translation(r.plant_context), joint.get_rotation(r.plant_context)]
                original_prescribed(dt)
                after = np.r_[joint.get_translation(r.plant_context), joint.get_rotation(r.plant_context)]
                self.prescribed_writes.append({
                    "time_s": t, "dt_s": dt,
                    "x_before": before[0], "y_before": before[1], "yaw_before": before[2],
                    "x_after": after[0], "y_after": after[1], "yaw_after": after[2],
                    "limited_v": r.base_backend.limited[0], "limited_omega": r.base_backend.limited[1],
                })

            r.base_backend.before_physics = record_prescribed
        if r.adapter.base_mode == "wheel_dynamic":
            self.wheels = {
                side: (
                    r.plant.GetJointByName(name, r.instance),
                    r.plant.GetJointActuatorByName(f"{name}_drive", r.instance).input_start(),
                )
                for side, name in zip(("left", "right"), r.adapter.wheel_joint_names)
            }
        if settings.disable_drive_torque:
            if not self.wheels:
                raise ValueError("Drive cut requires wheel_dynamic")
            original = r.base_backend.wheel_actuation
            indices = [index for _, index in self.wheels.values()]

            def cut_drive(dt):
                actuation = original(dt)
                # This branch contains wheel contributions only. Arm control
                # remains in runtime._servo_update and is not intercepted.
                other = np.delete(actuation, indices)
                if np.any(other != 0):
                    raise AssertionError("Wheel output includes other actuators")
                actuation[indices] = 0.0
                return actuation

            r.base_backend.wheel_actuation = cut_drive

    def record(self, root):
        """Simulator monitor: read current frame FK/twist and held actuation."""
        r = self.runtime
        context = r.plant.GetMyContextFromRoot(root)
        transform = self.frame.CalcPoseInWorld(context)
        xyz = transform.translation()
        rpy = transform.rotation().ToRollPitchYaw().vector()
        twist = self.frame.CalcSpatialVelocityInWorld(context)
        linear = twist.translational()
        yaw = float(rpy[2])
        forward = float(linear[0] * np.cos(yaw) + linear[1] * np.sin(yaw))
        omega = float(twist.rotational()[2])
        public = r.base_backend.observe()
        row = {
            "time_s": float(context.get_time()), "phase": self.phase,
            "command_v_m_s": self.command[0], "command_omega_rad_s": self.command[1],
            "limited_v_m_s": float(r.base_backend.limited[0]),
            "limited_omega_rad_s": float(r.base_backend.limited[1]),
            "x_m": float(xyz[0]), "y_m": float(xyz[1]), "z_m": float(xyz[2]),
            "roll_rad": float(rpy[0]), "pitch_rad": float(rpy[1]), "yaw_rad": yaw,
            "plant_v_forward_m_s": forward, "plant_speed_xy_m_s": float(np.linalg.norm(linear[:2])),
            "plant_omega_rad_s": omega,
            "public_v_forward_m_s": float(np.dot(public["linear_velocity_world_m_s"][:2], [np.cos(yaw), np.sin(yaw)])),
            "public_speed_xy_m_s": float(np.linalg.norm(public["linear_velocity_world_m_s"][:2])),
            "public_omega_rad_s": public["yaw_rate_rad_s"],
            "base_link_z_m": float(self.body.EvalPoseInWorld(context).translation()[2]),
        }
        if not self.wheels:
            self.monitor_calls.append({key: row[key] for key in ("time_s", "x_m", "y_m", "yaw_rad")})
        applied = r.plant.get_actuation_input_port().Eval(context)
        for side, (joint, index) in self.wheels.items():
            telemetry = r.base_backend.wheel_telemetry[joint.name()]
            row[f"{side}_target_rad_s"] = telemetry["target_rad_s"]
            row[f"{side}_actual_rad_s"] = float(joint.get_angular_rate(context))
            row[f"{side}_computed_torque_nm"] = telemetry["torque_nm"]
            row[f"{side}_applied_torque_nm"] = float(applied[index])
        if self.rows and abs(row["time_s"] - self.rows[-1]["time_s"]) < 1e-10:
            self.rows[-1] = row
        else:
            self.rows.append(row)

    def save(self, output):
        """Add independent pose finite differences, keeping raw plant values."""
        for index, row in enumerate(self.rows):
            values = (float("nan"),) * 3
            if index:
                previous = self.rows[index-1]
                dt = row["time_s"]-previous["time_s"]
                dx, dy = row["x_m"]-previous["x_m"], row["y_m"]-previous["y_m"]
                yaw = (row["yaw_rad"]+previous["yaw_rad"])/2
                delta = row["yaw_rad"]-previous["yaw_rad"]
                values = ((dx*np.cos(yaw)+dy*np.sin(yaw))/dt, np.hypot(dx, dy)/dt, np.arctan2(np.sin(delta), np.cos(delta))/dt)
            row.update(zip(("pose_fd_v_m_s", "pose_fd_speed_xy_m_s", "pose_fd_omega_rad_s"), values))
        with (output / "physics.csv").open("w") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.rows[0])
            writer.writeheader()
            writer.writerows(self.rows)
        for name, rows in (("prescribed_writes", self.prescribed_writes), ("monitor_calls", self.monitor_calls)):
            if rows:
                with (output / f"{name}.csv").open("w") as stream:
                    writer = csv.DictWriter(stream, fieldnames=rows[0])
                    writer.writeheader()
                    writer.writerows(rows)
        return read_trace(output / "physics.csv")


def read_trace(path):
    """Read numeric/string columns with NumPy; no extra dependency required."""
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8").view(np.recarray)


def build_config(mode, navigation):
    """Reuse production parameters; both arms hold the normal compact posture."""
    config = make_config(mode, obstacle=navigation)
    spec = config.robot_adapter.spec
    home = list(spec.home_positions)
    for side in ("left", "right"):
        home[spec.controlled_joint_names.index(f"{side}_elbow_joint")] = 1.4
    adapter = type(config.robot_adapter)(
        dataclasses.replace(spec, home_positions=tuple(home)),
        config.robot_adapter.base_config,
    )
    return dataclasses.replace(config, robot_adapter=adapter, episode_duration=180)


def run_case(mode, navigation, settings, output):
    """Run original env.step with a local monitor and optional wheel-only cut."""
    output.mkdir(parents=True, exist_ok=False)
    config = build_config(mode, navigation)
    env = make_env(config)
    recorder = Recorder(env, settings)
    obs, _ = env.reset(seed=0)
    runtime = env.backend
    recorder.record(runtime.simulator.get_context())
    runtime.simulator.set_monitor(recorder.record)
    manifest = {
        "mode": mode, "settings": dataclasses.asdict(settings),
        "base_config": dataclasses.asdict(config.robot_adapter.base_config),
        "timing": dataclasses.asdict(config.timing),
        "home_positions": config.robot_adapter.spec.home_positions,
        "root_floating": recorder.body.is_floating(),
        "num_actuated_dofs": runtime.plant.num_actuated_dofs(),
        "num_constraints": runtime.plant.num_constraints(),
        "joint_names": [runtime.plant.get_joint(i).name() for i in runtime.plant.GetJointIndices(runtime.instance)],
        "locked_joint_names": [runtime.plant.get_joint(i).name() for i in runtime.plant.GetJointIndices(runtime.instance) if runtime.plant.get_joint(i).is_locked(runtime.plant_context)],
    }
    commands = []

    def advance(action, phase):
        nonlocal obs
        recorder.phase = phase
        recorder.command = (action.velocity_m_s, action.yaw_rate_rad_s)
        commands.append({"start_time_s": obs.time_s, "phase": phase, "v": action.velocity_m_s, "omega": action.yaw_rate_rad_s})
        obs, _, terminated, truncated, info = env.step(action)
        if terminated or truncated or not info["action_decision"]["accepted"]:
            raise RuntimeError(info)

    if navigation:
        query = env.get_planning_query()
        navmap = build_navigation_map(query, navigation_frame=config.robot_adapter.navigation_frame_name, ground_body_names=config.scenario.ground_body_names, ground_geometries=config.scenario.ground_geometries)
        navigator = Navigator(navmap)
        for _ in range(20):
            advance(BaseVelocityAction(0, 0), "settle")
        navigator.set_goal(NavigationGoal(Pose((2.8, 0, 0), (1, 0, 0, 0))), obs)
        while True:
            action = navigator.act(obs)
            if navigator.status == "arrived":
                manifest["arrival_time_s"] = obs.time_s
                manifest["navigator_stable_since_s"] = navigator.stable_since
                manifest["navigation_config"] = dataclasses.asdict(navigator.config)
                break
            if navigator.status != "tracking":
                raise RuntimeError(f"Navigation failed: {navigator.status}")
            advance(action, "navigation")
    else:
        for phase, steps, v, omega in (
            ("stationary", 20, 0, 0), ("forward", 40, .1, 0),
            ("stop_forward", 30, 0, 0), ("spin", 40, 0, .3),
            ("stop_spin", 30, 0, 0),
        ):
            for _ in range(steps):
                advance(BaseVelocityAction(v, omega), phase)
            print(mode, settings, phase, obs.time_s, flush=True)
    frame = recorder.save(output)
    manifest["max_sample_gap_s"] = float(np.diff(frame.time_s).max())
    manifest["samples"] = len(frame)
    manifest["physics_sha256"] = hashlib.sha256((output / "physics.csv").read_bytes()).hexdigest()
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
    (output / "commands.json").write_text(json.dumps(commands, indent=2))
    print("Saved", output, flush=True)


def analyze(root):
    """Create side-by-side plots and independently recompute arrival windows."""
    names = ("planar_kinematic", "wheel_dynamic", "wheel_disabled")
    colors = ("tab:blue", "tab:orange", "tab:green")
    frames = {n: read_trace(root / n / "physics.csv") for n in names}
    summary = {"motion": {}, "navigation": {}}
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    for name, color in zip(names, colors):
        f = frames[name]
        v = f.pose_fd_v_m_s if name == "planar_kinematic" else f.plant_v_forward_m_s
        w = f.pose_fd_omega_rad_s if name == "planar_kinematic" else f.plant_omega_rad_s
        for ax, values in zip(axes, (v, w, f.x_m - f.x_m[0], f.yaw_rad)):
            ax.plot(f.time_s, values, color=color, label=name, linewidth=1.2)
        forward = f[(f.time_s >= 5) & (f.time_s < 6)]
        start = f[np.argmin(abs(f.time_s - 2))]
        end = f[np.argmin(abs(f.time_s - 6))]
        summary["motion"][name] = {
            "forward_last_second_mean_v_m_s": float(forward.public_v_forward_m_s.mean()),
            "forward_displacement_xy_m": float(np.hypot(end.x_m-start.x_m, end.y_m-start.y_m)),
            "forward_max_abs_v_m_s": float(abs(f[(f.time_s >= 2) & (f.time_s <= 6)].public_v_forward_m_s).max()),
            "initial_to_final_base_z_change_m": float(f.base_link_z_m[-1]-f.base_link_z_m[0]),
            "max_sample_gap_s": float(np.diff(f.time_s).max()),
            "spin_last_second_mean_omega_rad_s": float(f[(f.time_s >= 12) & (f.time_s < 13)].public_omega_rad_s.mean()),
        }
        for stop_time, label in ((6, "forward"), (13, "spin")):
            tail = f[(f.time_s >= stop_time-1e-9) & (f.time_s <= stop_time+3+1e-9)]
            moving = (abs(tail.public_v_forward_m_s) > .01) | (abs(tail.public_omega_rad_s) > .02)
            last_moving = np.flatnonzero(moving)
            stopped_index = int(last_moving[-1]+1) if len(last_moving) else 0
            summary["motion"][name][f"{label}_stop_delay_s"] = float(tail.time_s[stopped_index]-stop_time) if stopped_index < len(tail) else None
        if name != "planar_kinematic":
            summary["motion"][name]["max_abs_applied_wheel_torque_nm"] = float(max(abs(f.left_applied_torque_nm).max(), abs(f.right_applied_torque_nm).max()))
    f = frames["planar_kinematic"]
    axes[0].step(f.time_s, f.command_v_m_s, color="black", linestyle="--", label="requested", where="post")
    axes[1].step(f.time_s, f.command_omega_rad_s, color="black", linestyle="--", where="post")
    for ax, label in zip(axes, ("forward v [m/s]", "yaw rate [rad/s]", "delta x [m]", "yaw [rad]")):
        ax.set_ylabel(label)
        ax.grid(True, alpha=.3)
    axes[0].legend()
    axes[-1].set_xlabel("simulation time [s]")
    fig.suptitle("Same command / original limits; kinematic velocity from pose finite differences")
    fig.tight_layout()
    fig.savefig(root / "base_comparison.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(12, 7))
    for name, color in zip(names[:2], colors[:2]):
        f = frames[name]
        velocity = f.pose_fd_v_m_s if name == "planar_kinematic" else f.plant_v_forward_m_s
        omega = f.pose_fd_omega_rad_s if name == "planar_kinematic" else f.plant_omega_rad_s
        for row, values, windows in ((0, velocity, ((1.95, 2.65), (5.95, 6.65))), (1, omega, ((8.95, 9.65), (12.95, 13.65)))):
            for col, (start, end) in enumerate(windows):
                mask = (f.time_s >= start) & (f.time_s <= end)
                axes[row, col].plot(f.time_s[mask], values[mask], color=color, label=name)
                if name == names[0]:
                    requested = f.command_v_m_s if row == 0 else f.command_omega_rad_s
                    axes[row, col].plot(f.time_s[mask], requested[mask], "k--", label="requested")
    for index, ax in enumerate(axes.flat):
        ax.set_ylabel("v [m/s]" if index < 2 else "omega [rad/s]")
        ax.set_xlabel("simulation time [s]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=.3)
    fig.suptitle("Unchanged parameters: startup / requested-stop detail")
    fig.tight_layout()
    fig.savefig(root / "base_transients.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    for col, name in enumerate(("wheel_dynamic", "wheel_disabled")):
        f = frames[name]
        for side, color in (("left", "tab:blue"), ("right", "tab:orange")):
            axes[0, col].plot(f.time_s, f[f"{side}_actual_rad_s"], color=color, label=f"{side} actual")
            axes[0, col].plot(f.time_s, f[f"{side}_target_rad_s"], color=color, linestyle="--", label=f"{side} target")
            axes[1, col].plot(f.time_s, f[f"{side}_applied_torque_nm"], color=color, label=f"{side} applied")
            axes[1, col].plot(f.time_s, f[f"{side}_computed_torque_nm"], color=color, linestyle=":", label=f"{side} computed")
        axes[0, col].set_title(name)
        axes[0, col].set_ylabel("wheel angular rate [rad/s]")
        axes[1, col].set_ylabel("wheel torque [Nm]")
        axes[1, col].set_xlabel("simulation time [s]")
    for ax in axes.flat:
        ax.legend(fontsize=8)
        ax.grid(True, alpha=.3)
    fig.tight_layout()
    fig.savefig(root / "wheel_torque_isolation.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(4, 2, figsize=(12, 10), sharex="col")
    for col, mode in enumerate(names[:2]):
        directory = root / f"navigation_{mode}"
        f = read_trace(directory / "physics.csv")
        meta = json.loads((directory / "manifest.json").read_text())
        end = meta["arrival_time_s"]
        # Match public stop contract, independently check FK finite differences
        # for the prescribed mode (the locked Plant twist itself is zero).
        fields = ("position_error_m", "yaw_error_rad", "speed_check_m_s", "omega_check_rad_s")
        f = recfunctions.append_fields(f, fields, (
            np.hypot(f.x_m-2.8, f.y_m),
            abs(np.arctan2(np.sin(f.yaw_rad), np.cos(f.yaw_rad))),
            f.pose_fd_speed_xy_m_s if mode == "planar_kinematic" else f.plant_speed_xy_m_s,
            abs(f.pose_fd_omega_rad_s if mode == "planar_kinematic" else f.plant_omega_rad_s),
        ), usemask=False, asrecarray=True)
        thresholds = (.03, np.deg2rad(3), .01, .02)
        window = f[(f.time_s >= end-.5-1e-9) & (f.time_s <= end+1e-9)]
        maxima = {key: float(window[key].max()) for key in fields}
        passed = all(maxima[key] <= bound for key, bound in zip(fields, thresholds))
        duration = float(window.time_s[-1]-window.time_s[0])
        gap = float(np.diff(window.time_s).max())
        passed = passed and duration >= .5-1e-8 and gap <= .001+1e-8
        summary["navigation"][mode] = {
            "pass": bool(passed), "arrival_time_s": end,
            "navigator_stable_since_s": meta["navigator_stable_since_s"],
            "window_start_s": float(window.time_s[0]), "duration_s": duration,
            "samples": len(window), "maximum_sample_gap_s": gap, "window_maxima": maxima,
        }
        with (directory / "arrival_window.csv").open("w") as stream:
            writer = csv.writer(stream)
            writer.writerow(window.dtype.names)
            writer.writerows(window.tolist())
        shown = f[f.time_s >= end-1.5]
        for row, (field, bound) in enumerate(zip(fields, thresholds)):
            ax = axes[row, col]
            ax.plot(shown.time_s, shown[field])
            ax.axhline(bound, color="red", linestyle="--", label="threshold")
            ax.axvspan(end-.5, end, color="green", alpha=.15, label="0.5 s audit window")
            ax.set_ylabel(field)
            ax.grid(True, alpha=.3)
        axes[0, col].set_title(mode)
        axes[0, col].legend(fontsize=8)
        axes[-1, col].set_xlabel("simulation time [s]")
    fig.tight_layout()
    fig.savefig(root / "navigation_arrival_window.png", dpi=150)
    plt.close(fig)
    normal_meta = json.loads((root / "wheel_dynamic/manifest.json").read_text())
    cut_meta = json.loads((root / "wheel_disabled/manifest.json").read_text())
    same_parameters = all(normal_meta[key] == cut_meta[key] for key in ("base_config", "timing", "home_positions", "joint_names", "locked_joint_names", "num_constraints", "num_actuated_dofs", "root_floating"))
    ratio = summary["motion"]["wheel_disabled"]["forward_displacement_xy_m"] / summary["motion"]["wheel_dynamic"]["forward_displacement_xy_m"]
    sequences = [json.loads((root / name / "commands.json").read_text()) for name in names]
    same_commands = all(
        [(r["phase"], r["v"], r["omega"]) for r in sequence]
        == [(r["phase"], r["v"], r["omega"]) for r in sequences[0]]
        for sequence in sequences[1:]
    )
    summary["isolation"] = {
        "same_model_control_initial_configuration": same_parameters,
        "same_command_sequence_all_three_runs": same_commands,
        "disabled_to_normal_forward_displacement_ratio": ratio,
    }
    repo = Path(__file__).resolve().parents[2]
    summary["source_sha256"] = {
        str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (
            repo / "examples/online_manipulation/audit_base_modes.py",
            *(repo / "src/online_manipulation" / name for name in (
                "runtime.py", "base.py", "controller.py", "navigation.py",
                "adapters/zerith_mobile.py", "adapters/description.py",
            )),
        )
    }
    (root / "comparison_metrics.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    if not all(item["pass"] for item in summary["navigation"].values()):
        raise AssertionError("Navigation did not meet all four thresholds for 0.5 s")
    if summary["motion"]["wheel_disabled"]["max_abs_applied_wheel_torque_nm"] != 0:
        raise AssertionError("Isolation failed to cut wheel actuation")
    if not same_parameters or not same_commands or ratio >= .01:
        raise AssertionError("Isolation not established: configuration differs or drift exceeds 1% of driven travel")


def analyze_sampling(output):
    """Explain monitor event-side differences using actual per-write deltas."""
    directory = output / "planar_kinematic"
    writes = read_trace(directory / "prescribed_writes.csv")
    frames = read_trace(directory / "physics.csv")
    distance = np.hypot(writes.x_after-writes.x_before, writes.y_after-writes.y_before)
    dyaw = writes.yaw_after-writes.yaw_before
    metrics = {
        "number_of_actual_state_writes": len(writes),
        "maximum_translation_increment_error_m": float(max(abs(distance-abs(writes.limited_v)*writes.dt_s))),
        "maximum_yaw_increment_error_rad": float(max(abs(dyaw-writes.limited_omega*writes.dt_s))),
        "minimum_write_time_gap_s": float(np.diff(writes.time_s).min()),
        "maximum_write_time_gap_s": float(np.diff(writes.time_s).max()),
    }
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    for ax, center, raw_key, values, label in (
        (axes[0], 4.986, "pose_fd_v_m_s", distance/writes.dt_s, "forward v [m/s]"),
        (axes[1], 9.816, "pose_fd_omega_rad_s", dyaw/writes.dt_s, "yaw rate [rad/s]"),
    ):
        raw = frames[(frames.time_s > center-.005) & (frames.time_s < center+.005)]
        selected = (writes.time_s > center-.005) & (writes.time_s < center+.005)
        ax.plot(raw.time_s, raw[raw_key], "o-", label="monitor merged-time FK difference")
        ax.plot(writes.time_s[selected], values[selected], "x--", label="actual state write delta / dt")
        ax.set_ylabel(label)
        ax.set_xlabel("simulation time [s]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=.3)
    fig.suptitle("Sampling-side artifact: raw spikes preserved; no filtering or physics edits")
    fig.tight_layout()
    fig.savefig(output / "kinematic_sampling_diagnostic.png", dpi=150)
    plt.close(fig)
    (output / "sampling_metrics.json").write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2))
    if metrics["maximum_translation_increment_error_m"] > 1e-12 or metrics["maximum_yaw_increment_error_rad"] > 1e-12:
        raise AssertionError("Unexpected prescribed integration error")


def main():
    """Run one reproducible audit case, or plot and check all saved cases."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--case", required=True, choices=("planar_kinematic", "wheel_dynamic", "wheel_disabled", "navigation_planar_kinematic", "navigation_wheel_dynamic", "analyze", "analyze_sampling"))
    args = parser.parse_args()
    if args.case == "analyze":
        analyze(args.output)
    elif args.case == "analyze_sampling":
        analyze_sampling(args.output)
    else:
        navigation = args.case.startswith("navigation_")
        mode = args.case.removeprefix("navigation_")
        settings = AuditSettings(disable_drive_torque=mode == "wheel_disabled")
        if settings.disable_drive_torque:
            mode = "wheel_dynamic"
        run_case(mode, navigation, settings, args.output / args.case)


if __name__ == "__main__":
    main()
