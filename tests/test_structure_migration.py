"""Targeted structure migration checks, including optional real A scene audit."""

import ast
import dataclasses
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from src.online_manipulation import (
    BaseConfig, HoldAction, ScenarioSpec, TimingConfig, ZerithMobileRobotAdapter,
    FREE_MOTION_CONTACT_POLICY,
    build_navigation_map, build_planning_query, load_experiment, make_env,
    make_zerith_dual_spec, inspect_dependencies, prepare_scene,
)
from src.online_manipulation.contact import SupportContactPolicy, permits_contact
from src.online_manipulation.scene_geometry import resolve_ground_geometries
from src.online_manipulation.scene_input import load_dmd

ROOT = Path(__file__).resolve().parents[1]


class StructureMigrationTest(unittest.TestCase):
    """Test entry-point boundaries and relocatable assets without old output."""

    def test_formal_modules_do_not_depend_on_examples_or_monitor(self):
        paths = list((ROOT / "src/online_manipulation").rglob("*.py"))
        for path in paths:
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.ImportFrom):
                    self.assertFalse((node.module or "").startswith(("examples", "scripts", "src.item_locking_monitor")), path)
        self.assertNotIn("zerith_grasp_geometry", (ROOT / "src/online_manipulation/adapters/zerith.py").read_text())

    def test_external_policy_evaluator_reset_and_artifacts(self):
        with tempfile.TemporaryDirectory() as folder:
            outside = Path(folder)
            shutil.copyfile(ROOT / "examples/online_manipulation/external_evaluator.py", outside / "my_components.py")
            config = json.loads((ROOT / "experiments/minimal.json").read_text())
            config.update(policy="my_components:make_policy", evaluator="my_components:make_evaluator")
            config["run"]["seeds"] = [7, 8]
            file = outside / "run.json"
            file.write_text(json.dumps(config))
            result = subprocess.run([
                sys.executable, "-B", "-m", "src.online_manipulation", str(file),
                "--repository-root", str(ROOT), "--cache-root", str(outside / "cache"),
                "--output-root", str(outside / "output"), "--trust-factories",
            ], cwd=outside, env={**os.environ, "PYTHONPATH": f"{outside}:{ROOT}"},
                text=True, capture_output=True, check=True)
            self.assertIn("custom_hold_confirmed", result.stdout)
            for seed, index in ((7, 0), (8, 1)):
                summary = json.loads((outside / f"output/episode_{index:03d}_seed_{seed}/summary.json").read_text())
                self.assertEqual(summary["policy_steps"], 3)
                self.assertTrue(summary["success"])
            self.assertTrue((outside / "output/resolved_config.json").is_file())
            with self.assertRaisesRegex(ValueError, "untrusted"):
                load_experiment(file, repository_root=ROOT, cache_root=outside / "unused")

    def test_profile_swap_and_capability_rejection(self):
        with tempfile.TemporaryDirectory() as folder:
            config = json.loads((ROOT / "experiments/mobile.json").read_text())
            path = Path(folder) / "config.json"
            for mode, height in (("planar_kinematic", .1816), ("wheel_dynamic", .1808)):
                config["base"] = {"mode": mode, "base_height_m": height}
                # Finite profile stays the source; explicit initial-state
                # selection below is exercised via a temporary profile root.
                if mode == "planar_kinematic":
                    config["initial_state"] = "mobile_planar"
                else:
                    config["initial_state"] = "mobile"
                path.write_text(json.dumps(config))
                experiment = load_experiment(path, repository_root=ROOT, cache_root=Path(folder) / mode)
                env = make_env(experiment.environment_config)
                obs, _ = env.reset(0)
                obs, *_ = env.step(HoldAction())
                self.assertGreater(obs.time_s, 0)
                self.assertEqual(set(obs.robot.gripper_widths_m), {"left", "right"})
            config["requires"] = {"sensors": ["not_a_camera"]}
            path.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, "Unsatisfied"):
                load_experiment(path, repository_root=ROOT, cache_root=folder)

    def test_control_settings_do_not_change_hardware_limits(self):
        from src.online_manipulation.controller import configure_joint_servos
        from src.online_manipulation import JointSpec
        joint = JointSpec("external_axis", "revolute", -2, 2, 3, 10, 100, 20)
        result, = configure_joint_servos((joint,), {"external_axis": {"kp": 80, "effort_limit": 8}})
        self.assertEqual((result.position_lower, result.position_upper, result.velocity_limit), (-2, 2, 3))
        self.assertEqual((result.kp, result.effort_limit), (80, 8))
        with self.assertRaisesRegex(ValueError, "exceeds"):
            configure_joint_servos((joint,), {"external_axis": {"effort_limit": 11}})

    def test_b_reports_missing_textures_without_substitution(self):
        root = ROOT / "models/21-20-10_cleaned/scene_000"
        with self.assertRaisesRegex(ValueError, "Wood094") as failure:
            inspect_dependencies(root / "combined_house/house.dmd.yaml", (root / "package.xml",))
        self.assertEqual(str(failure.exception).count("Missing dependency:"), 3)

    @unittest.skipUnless(os.environ.get("SCENE_ROOT"), "Set SCENE_ROOT to dependency-complete real A scene")
    def test_real_a_relocation_metadata_and_wall_scope(self):
        source = Path(os.environ["SCENE_ROOT"])
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder)
            prepared = prepare_scene(scene_root=source, variant="furniture_welded", cache_root=destination / "cache",
                                     overrides=load_dmd(ROOT / "experiments/inputs/pick_lift/scene_overrides.yaml"),
                                     additional_package_xmls=(ROOT / "models/zerith_pick_eval/package.xml",))
            mapping = json.loads(prepared.metadata_path.read_text())["objects"]
            box = next(o for o in mapping if o["model_instance"] == "living_room_box_0")
            self.assertEqual(box["geometry_facts"]["dimensions_m"], [.06, .04, .03])
            self.assertEqual(box["semantic_binding"], ["living_room", "box_0"])
            manifest = json.loads(prepared.manifest_path.read_text())
            for name, digest in manifest["source_sha256"].items():
                self.assertEqual(hashlib.sha256(Path(name).read_bytes()).hexdigest(), digest)
            # Move the upstream dependency closure itself, not only our cache.
            moved_source = destination / "moved_upstream"
            dependencies, _ = inspect_dependencies(source / "combined_house/house_furniture_welded.dmd.yaml", (source / "package.xml",))
            for path in (*dependencies, source / "package.xml", source / "combined_house/house_state.json"):
                target = moved_source / path.relative_to(source)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(path, target)
            moved = prepare_scene(scene_root=moved_source, variant="furniture_welded", cache_root=destination / "moved_cache")
            self.assertTrue(moved.dmd_path.is_file())
            relocated = destination / "relocated"
            shutil.copytree(destination / "cache", relocated)
            scenario = ScenarioSpec(relocated / prepared.dmd_path.relative_to(destination / "cache"),
                                    tuple(relocated / p.relative_to(destination / "cache") for p in prepared.package_xmls),
                                    ground_geometries=prepared.ground_geometries)
            spec = make_zerith_dual_spec(robot_model_dir=ROOT / "models/zerith_drake", robot_xyz=(3.5, 3.8, .1808), robot_yaw_deg=0,
                                         rail_position=.4, q_home_left=(0,) * 7)
            adapter = ZerithMobileRobotAdapter(spec, BaseConfig(mode="wheel_dynamic", base_height_m=.1808))
            query = build_planning_query(scenario=scenario, robot_adapter=adapter, timing=TimingConfig())
            target = query.plant.GetBodyByName("base_link", query.plant.GetModelInstanceByName("living_room_box_0"))
            import numpy as np
            np.testing.assert_allclose(target.EvalPoseInWorld(query.context).translation(),
                                       (1.983109433597935, 2.7700010500000007, .5107438948291487), atol=1e-12)
            inspector = query.plant.get_geometry_query_input_port().Eval(query.context).inspector()
            with self.assertRaisesRegex(ValueError, "explicit ground_geometries"):
                resolve_ground_geometries(query.plant, inspector, ("room_geometry_living_room::room_geometry_body_link",))
            nav = build_navigation_map(query, navigation_frame=spec.base_link_name, ground_geometries=scenario.ground_geometries,
                                       bounds=(-1, -1, 8, 7))
            self.assertFalse(nav.edge_free((3.5, 4), (3.5, 6)))
            policy = query._contact_policy(FREE_MOTION_CONTACT_POLICY)
            for pair in query.support_geometry_limits_m:
                (a, ga), (b, gb) = pair
                self.assertTrue(permits_contact(policy, a, b, ga, gb))
                self.assertFalse(permits_contact(policy, a, b, ga.replace("floor_collision", "north_wall_collision"), gb.replace("floor_collision", "north_wall_collision")))

    def test_minimal_without_optional_packages(self):
        """Block optional imports even though this developer venv contains them."""
        code = '''
import importlib.abc, sys, tempfile
class BlockOptional(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'scenesmith', 'openai', 'agents', 'torch', 'trimesh', 'manipulation', 'networkx'}:
            raise ImportError('optional package disabled: ' + fullname)
sys.meta_path.insert(0, BlockOptional())
from src.online_manipulation import load_experiment, make_env, HoldAction
from pathlib import Path
root = Path(sys.argv[1])
with tempfile.TemporaryDirectory() as cache:
    experiment = load_experiment(root / 'experiments/minimal.json', repository_root=root, cache_root=cache)
    env = make_env(experiment.environment_config)
    obs, _ = env.reset(0)
    obs, _, _, _, info = env.step(HoldAction())
    assert obs.time_s > 0
    assert info['action_decision']['accepted']
print('optional-dependencies-disabled: PASS')
'''
        result = subprocess.run([sys.executable, "-B", "-c", code, str(ROOT)],
                                cwd="/tmp", env={**os.environ, "PYTHONPATH": str(ROOT)},
                                capture_output=True, text=True, check=True)
        self.assertIn("PASS", result.stdout)


if __name__ == "__main__":
    unittest.main()
