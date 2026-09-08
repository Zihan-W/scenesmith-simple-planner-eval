"""Finite-profile experiment composition on the existing Runtime and runner.

JSON is data. External module:function factories execute trusted local Python
only when the caller explicitly opts in. No model service or planner is loaded
by default. Collision checking is part of Runtime, not an optional plugin.
"""

import argparse
import dataclasses
import hashlib
import json
import os
from collections.abc import Mapping
from pathlib import Path

from src.online_manipulation.assembly import load_factory, run
from src.online_manipulation.evaluation import EvaluatedTask, Evaluator, TaskResultEvaluator
from src.online_manipulation.protocols import Policy, RobotAdapter, Task
from src.online_manipulation.runtime import RuntimeConfig
from src.online_manipulation.controller import configure_joint_servos
from src.online_manipulation.scene_input import load_dmd, prepare_scene, inspect_dependencies, package_roots
from src.online_manipulation.specs import ObservedBodySpec, ScenarioSpec, TimingConfig, VisualizationConfig
from src.online_manipulation.tasks import NullTask, PickLiftTask, PickLiftTaskConfig


def _json(value):
    if dataclasses.is_dataclass(value):
        return {f.name: _json(getattr(value, f.name)) for f in dataclasses.fields(value)}
    if isinstance(value, Mapping):
        return {str(k): _json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json(v) for v in value]
    if isinstance(value, Path):
        return str(value)
    if callable(value):
        return f"{value.__module__}:{value.__qualname__}"
    return value


@dataclasses.dataclass(frozen=True)
class FactoryContext:
    """Public inputs to trusted policy factories; no live Drake internals."""

    environment_config: object
    robot_spec: object
    options: Mapping
    repository_root: Path


@dataclasses.dataclass(frozen=True)
class Experiment:
    """Resolved composition, usable directly or through the existing runner."""

    environment_config: object
    policy: Policy
    resolved_config: Mapping
    run_options: Mapping

    def run(self, output_root):
        """Write standard episode artifacts plus full configuration provenance."""
        destination = Path(output_root)
        # Runner owns the empty-output check; do not overwrite an earlier run.
        if destination.exists() and any(destination.iterdir()):
            raise FileExistsError(f"Output directory is not empty: {destination}")
        try:
            return run(self.environment_config, self.policy, output_root=destination,
                       **self.run_options)
        finally:
            destination.mkdir(parents=True, exist_ok=True)
            (destination / "resolved_config.json").write_text(json.dumps(
                _json({**self.resolved_config, "run": self.run_options}), indent=2))


def load_experiment(config_path, *, repository_root, cache_root, scene_root=None,
                    trust_factories=False, meshcat=False):
    """Resolve finite profiles and prepare assets without constructing a simulator.

    Relative profile/asset paths are repository-relative. SCENE_ROOT is needed
    only for the SceneSmith scene profile. cache_root must be a fresh directory
    for preparation, so previous run inputs are never silently overwritten.
    """
    root = Path(repository_root).resolve()
    path = Path(config_path).resolve()
    user = json.loads(path.read_text())
    allowed = {"robot", "control", "initial_state", "scene", "policy", "task",
               "evaluator", "scene_options", "robot_options", "policy_options",
               "task_options", "evaluator_options", "initial_state_options",
               "control_options", "base", "run", "requires"}
    if set(user) - allowed:
        raise ValueError(f"Unknown experiment keys: {sorted(set(user) - allowed)}")
    profile_path = root / "experiments/profiles.json"
    profiles = json.loads(profile_path.read_text())
    selected = {}
    for group in ("robot", "control", "initial_state", "scene", "policy", "task", "evaluator"):
        reference = user[group]
        if reference not in profiles[group]:
            if not trust_factories or ":" not in reference or group not in ("robot", "policy", "task", "evaluator"):
                raise ValueError(f"Unknown/untrusted {group} profile: {reference}")
            selected[group] = {"factory": reference}
        else:
            selected[group] = dict(profiles[group][reference])
        selected[group].update(user.get(group + "_options", {}))
    scene = selected["scene"]
    prepared = None
    if scene["kind"] == "scenesmith":
        source = scene_root or os.environ.get(scene["root_env"])
        if not source:
            raise ValueError(f"Provide scene_root or {scene['root_env']}")
        overrides = load_dmd(root / scene["overrides"]) if scene.get("overrides") else None
        prepared = prepare_scene(scene_root=source, variant=scene["variant"],
                                 cache_root=cache_root, overrides=overrides,
                                 additional_package_xmls=tuple(root / p for p in scene.get("additional_package_xmls", ())))
        scenario = ScenarioSpec(prepared.dmd_path, prepared.package_xmls,
                                ground_geometries=prepared.ground_geometries)
    elif scene["kind"] == "dmd":
        dmd = root / scene["dmd"]
        xmls = tuple(root / p for p in scene["package_xmls"])
        inspect_dependencies(dmd, xmls)
        scenario = ScenarioSpec(dmd, xmls,
                                ground_geometries=tuple(tuple(p) for p in scene.get("ground_geometries", ())),
                                observed_bodies=tuple(ObservedBodySpec(**b) for b in scene.get("observed_bodies", ())))
    else:
        raise ValueError(f"Unknown scene kind: {scene['kind']}")
    scenario = dataclasses.replace(scenario, visualization=VisualizationConfig(enabled=meshcat))
    initial, control, robot = selected["initial_state"], selected["control"], selected["robot"]
    base_data = dict(user.get("base", {"mode": "fixed"}))
    if "factory" in robot:
        adapter = load_factory(robot["factory"])(robot, initial, control, base_data, root)
        if not isinstance(adapter, RobotAdapter):
            raise TypeError("Robot factory must return RobotAdapter")
        spec = adapter.spec
        legacy = False
    else:
        from src.online_manipulation.adapters.zerith import (
            make_zerith_robot_spec, make_zerith_camera_specs, ZerithRobotAdapter)
        from src.online_manipulation.adapters.zerith_dual import make_zerith_dual_spec, ZerithDualRobotAdapter
        from src.online_manipulation.adapters.zerith_mobile import ZerithMobileRobotAdapter
        from src.online_manipulation.base import BaseConfig
        cameras = make_zerith_camera_specs(**robot.get("camera_options", {}))
        kwargs = dict(initial, robot_model_dir=root / robot["model_dir"], cameras=cameras,
                      locked_joint_position_overrides=robot.get("locked_joint_positions", {}))
        if robot["kind"] == "zerith_left":
            if base_data["mode"] != "fixed":
                raise ValueError("Mobile profiles require zerith_dual; no implicit adapter change")
            spec = make_zerith_robot_spec(**kwargs)
            spec = dataclasses.replace(spec, controlled_joints=configure_joint_servos(spec.controlled_joints, control.get("joint_servo_settings", {})))
            adapter, legacy = ZerithRobotAdapter(spec), True
        elif robot["kind"] == "zerith_dual":
            spec = make_zerith_dual_spec(**kwargs)
            spec = dataclasses.replace(spec, controlled_joints=configure_joint_servos(spec.controlled_joints, control.get("joint_servo_settings", {})))
            if base_data["mode"] == "fixed":
                adapter = ZerithDualRobotAdapter(spec)
            else:
                base = BaseConfig(**base_data)
                if abs(initial["robot_xyz"][2] - base.base_height_m) > 1e-9:
                    raise ValueError("Initial base height and base configuration disagree")
                adapter = ZerithMobileRobotAdapter(spec, base)
            legacy = False
        else:
            raise ValueError(f"Unknown robot kind {robot['kind']}")
    task_data = selected["task"]
    if "factory" in task_data:
        task = load_factory(task_data["factory"])(task_data, spec)
    elif task_data["kind"] == "null":
        task = NullTask()
    elif task_data["kind"] == "picklift":
        b = task_data["bindings"]
        arm = task_data.get("carrier_arm_name", "left")
        if arm not in spec.grippers:
            raise ValueError(f"Task requires named gripper {arm}")
        grip = spec.grippers[arm]
        task = PickLiftTask(PickLiftTaskConfig(
            target_observation_name=task_data["target_observation_name"],
            target_contact_body=f"{b['target_model_name']}::{b['target_body_name']}",
            gripper_contact_bodies=tuple(f"{spec.model_instance_name}::{n}" for n in grip.contact_body_names),
            support_contact_bodies=tuple(b["support_contact_bodies"]),
            required_lift_m=task_data["required_lift_m"], required_hold_s=task_data["required_hold_s"],
            carrier_arm_name=arm, carrier_gripper_name=arm))
        scenario = dataclasses.replace(scenario, observed_bodies=(ObservedBodySpec(
            task_data["target_observation_name"], b["target_model_name"], b["target_body_name"], write_back=True),))
    else:
        raise ValueError(f"Unknown task kind {task_data['kind']}")
    if not isinstance(task, Task):
        raise TypeError("Task factory must implement the public Task lifecycle")
    timing = TimingConfig(**control["timing"])
    if legacy:
        from src.online_manipulation.adapters.zerith import ZerithEnvironmentConfig
        config = ZerithEnvironmentConfig(
            scenario=scenario, **kwargs, task=task, timing=timing,
            episode_duration=control["episode_duration"], max_joint_delta=control["max_joint_delta"],
            maximum_cartesian_joint_delta=control["maximum_cartesian_joint_delta"], enable_planning_query=True,
            joint_servo_settings=control.get("joint_servo_settings", {}))
    else:
        config = RuntimeConfig(scenario, adapter, timing=timing, task_factory=lambda: task,
                               episode_duration=control["episode_duration"],
                               maximum_joint_delta=control["max_joint_delta"],
                               maximum_cartesian_joint_delta=control["maximum_cartesian_joint_delta"])
    p = selected["policy"]
    if "factory" in p:
        policy = load_factory(p["factory"])(FactoryContext(config, spec, p, root))
    elif p["kind"] == "hold":
        from src.online_manipulation.policies import HoldPolicy
        policy = HoldPolicy()
    elif p["kind"] == "picklift":
        if not legacy or not isinstance(task, PickLiftTask):
            raise ValueError("Calibrated fixed PickLift expert requires zerith_left + PickLiftTask")
        from src.online_manipulation.recipes.pick_policy import build_policy
        expected_path = root / p["inputs"] / "expectations.json"
        expected = json.loads(expected_path.read_text())
        if prepared is None:
            raise ValueError("Fixed PickLift profile requires its explicitly prepared SceneSmith scene")
        source_dmd = Path(source) / "combined_house/house_furniture_welded.dmd.yaml"
        physical = next(o for o in json.loads(prepared.metadata_path.read_text())["objects"]
                        if o["model_instance"] == task_data["bindings"]["target_model_name"])
        package, relative = physical["asset"][len("package://"):].split("/", 1)
        target_asset = package_roots(prepared.package_xmls)[package] / relative
        actual = {
            "source_dmd_semantic_sha256": hashlib.sha256(json.dumps(load_dmd(source_dmd), sort_keys=True).encode()).hexdigest(),
            "robot_urdf_sha256": hashlib.sha256(spec.model_path.read_bytes()).hexdigest(),
            "target_sdf_sha256": hashlib.sha256(target_asset.read_bytes()).hexdigest(),
        }
        if scene["variant"] != "furniture_welded" or any(actual[k] != expected[k] for k in actual):
            raise ValueError("Fixed PickLift source/robot fingerprint changed; select a recalibrated policy instead")
        policy = build_policy(config, pick_artifact_root=root / p["inputs"],
                              calibration_json=root / p["calibration"],
                              policy_overrides=p.get("overrides"))
    else:
        raise ValueError(f"Unknown policy kind {p['kind']}")
    if not isinstance(policy, Policy):
        raise TypeError("Policy factory must implement the public Policy protocol")
    e = selected["evaluator"]
    evaluator = load_factory(e["factory"])(e) if "factory" in e else TaskResultEvaluator()
    if "factory" not in e and e["kind"] != "task_result":
        raise ValueError(f"Unknown evaluator kind {e['kind']}")
    if not isinstance(evaluator, Evaluator):
        raise TypeError("Evaluator factory must implement the public Evaluator protocol")
    evaluated = EvaluatedTask(task, evaluator)
    config = dataclasses.replace(config, **({"task": evaluated} if legacy else {"task_factory": lambda: evaluated}))
    required = user.get("requires", {})
    declared = getattr(policy, "required_capabilities", {})
    required = {key: set(required.get(key, ())) | set(declared.get(key, ()))
                for key in set(required) | set(declared)}
    actual = {"joints": set(spec.controlled_joint_names), "grippers": set(spec.grippers),
              "arms": set(spec.arm_groups), "sensors": {c.name for c in spec.cameras if c.enabled}}
    for kind, names in required.items():
        if kind not in actual or not set(names).issubset(actual[kind]):
            raise ValueError(f"Unsatisfied required capability {kind}: {names}")
    options = {"seeds": (0,), "max_steps": 100, "record_html": False, "write_final_dmd": False}
    options.update(user.get("run", {}))
    if options["record_html"] and not meshcat:
        raise ValueError("record_html requires meshcat enabled explicitly")
    input_files = {str(spec.model_path): hashlib.sha256(spec.model_path.read_bytes()).hexdigest()}
    if p.get("kind") == "picklift":
        for filename in (root / p["inputs"] / "pick_home.json", root / p["inputs"] / "pregrasp_ik.json", root / p["calibration"]):
            input_files[str(filename)] = hashlib.sha256(filename.read_bytes()).hexdigest()
    resolved = {"user_config": user, "selected_profiles": selected, "robot_spec": _json(spec),
                "scenario": _json(scenario), "control": control, "base": _json(getattr(adapter, "base_config", base_data)),
                "task": _json(getattr(task, "config", task_data)), "policy": _json(getattr(policy, "config", p)),
                "evaluator": e, "run": options,
                "provenance": {"experiment": str(path), "experiment_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                               "profiles": str(profile_path), "profiles_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
                               "robot_hardware_limits": str(spec.model_path),
                               "input_sha256": input_files,
                               "servo_defaults": "src/zerith_servo_config.py (validated simulation, not hardware specification)",
                               "scene_manifest": str(prepared.manifest_path) if prepared else None}}
    return Experiment(config, policy, resolved, options)


def main():
    """Run a finite-profile experiment from any working directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("--repository-root", required=True, type=Path)
    parser.add_argument("--cache-root", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--scene-root", type=Path)
    parser.add_argument("--trust-factories", action="store_true")
    parser.add_argument("--meshcat", action="store_true")
    parser.add_argument("--record-html", action="store_true", default=None)
    parser.add_argument("--write-final-dmd", action="store_true", default=None)
    args = parser.parse_args()
    experiment = load_experiment(args.config, repository_root=args.repository_root,
                                 cache_root=args.cache_root, scene_root=args.scene_root,
                                 trust_factories=args.trust_factories, meshcat=args.meshcat)
    overrides = {name: getattr(args, name) for name in ("record_html", "write_final_dmd")
                 if getattr(args, name) is not None}
    if overrides.get("record_html") and not args.meshcat:
        parser.error("--record-html requires --meshcat")
    experiment = dataclasses.replace(experiment, run_options={**experiment.run_options, **overrides})
    for result in experiment.run(args.output_root):
        print(json.dumps({key: result.summary[key] for key in (
            "seed", "success", "termination_reason", "episode_time_s", "policy_steps")}))
    print(f"Artifacts: {args.output_root.resolve()}")


if __name__ == "__main__":
    main()
