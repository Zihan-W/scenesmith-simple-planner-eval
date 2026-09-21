"""Observed-scene continuous problem export for a fixed Zerith skill skeleton.

This builder performs no skeleton search, no optimization and no execution.
Only PickLift and NavigateToPick -> PickLift are currently supported. The
official MoveFree/Pick pair represents the grasp approach witness; it does not
insert an additional executable navigation skill into a PickLift-only program.
"""

import copy
import dataclasses
import hashlib
import json
from pathlib import Path

from scripts.export_cutamp_geometry import export_query_geometry, stationary_support_check
from simulation.src import Pose
from simulation.src.robots.adapters.description import drake_pose

from .hierarchy import picklift_registry, validate_skill_program


class ContinuousProblemBuilder:
    """Bind a supported, already chosen program to one measured scene snapshot."""

    def __init__(self, domain, resolved_config, kinematics_template, robot_template):
        self.domain = domain
        self.resolved_config = resolved_config
        self.reference = json.loads(Path(kinematics_template).read_text())
        self.robot = json.loads(Path(robot_template).read_text())

    def build(self, program, world, output_root):
        """Write fresh geometry, measured joints and FK references without solving."""
        domain = self.domain
        validate_skill_program(program, picklift_registry(), frozenset(world.objects))
        names = tuple(step.skill for step in program.steps)
        if names not in (("PickLift",), ("NavigateToPick", "PickLift")):
            raise ValueError(f"Unsupported fixed cuTAMP skeleton: {names}")
        if any(step.arguments["object"] != domain.target_name for step in program.steps):
            raise ValueError("The fixed-skeleton adapter supports only the configured target")
        if world.observation_id != f"{domain.observation.time_s:.6f}":
            raise ValueError("World and geometry observations have different timestamps")
        measured = domain.observation.base["base_link_pose"]
        if world.robot["base_link_pose"] != measured:
            raise ValueError("World and geometry observations have different base poses")
        if (list(world.robot["joint_names"]) != list(domain.observation.robot.joint_names)
                or list(world.robot["q"]) != list(domain.observation.robot.q)):
            raise ValueError("World and geometry observations have different joint states")
        for name, item in domain.observation.objects.items():
            for key, value in item.pose.as_dict().items():
                if list(world.objects[name][key]) != value:
                    raise ValueError("World and geometry observations have different object poses")
        pose = Pose(tuple(measured["translation_m"]), tuple(measured["quaternion_wxyz"]))
        # Do not reuse a cached query modified by a previous constraint check.
        domain._query_cache = None
        query, _ = domain._candidate_query(pose)
        spec = query.robot_adapter.spec
        if hashlib.sha256(spec.model_path.read_bytes()).hexdigest() != self.reference["urdf_sha256"]:
            raise ValueError("Robot model changed since the collision/kinematics export")
        robot = copy.deepcopy(self.robot)
        kin = robot["kinematics"]
        reference_kin = self.reference["robot_config"]["kinematics"]
        for key in ("urdf_path", "base_link", "ee_link", "extra_links"):
            if kin[key] != reference_kin[key]:
                raise ValueError(f"Collision and kinematics templates disagree: {key}")
        plant, context, instance = query.plant, query.context, query.robot_model_instance
        positions = plant.GetPositions(context)

        def joint_position(name):
            joint = plant.GetJointByName(name, instance)
            if joint.num_positions() != 1:
                raise ValueError(f"Expected scalar robot joint: {name}")
            return float(positions[joint.position_start()])

        kin["lock_joints"] = {name: joint_position(name) for name in kin["lock_joints"]}
        active = list(spec.arm_groups["left"])
        if active != self.reference["joint_names"]:
            raise ValueError("Active arm changed since the reference export")
        base = plant.GetFrameByName(spec.base_link_name, instance)
        tcp = plant.GetFrameByName(spec.end_effector_frames["left"], instance)
        reference = {**self.reference, "scope": "measured_snapshot_kinematics_not_collision_certificate",
                     "configurations": [[joint_position(name) for name in active]],
                     "base_from_tcp": [plant.CalcRelativeTransform(context, base, tcp).GetAsMatrix4().tolist()],
                     "robot_config": robot, "observation_id": world.observation_id}
        geometry = export_query_geometry(query, domain.config, self.resolved_config,
                                         check_stationary_target=True)
        geometry["observed_robot_base_world_matrix"] = drake_pose(pose).GetAsMatrix4().tolist()
        geometry["observation_id"] = world.observation_id
        if names == ("PickLift",):
            geometry["stationary_robot_support_check"] = stationary_support_check(query, geometry)
        if not geometry["stationary_target_world_check"]["valid"]:
            raise ValueError("Observed target violates the unchanged static-world contact checks")
        output = Path(output_root)
        output.mkdir(parents=True, exist_ok=False)
        manifest = {"scope": "fixed_skeleton_observed_scene_continuous_problem",
                    "program": dataclasses.asdict(program), "observation_id": world.observation_id,
                    "optimize_base_xy": names[0] == "NavigateToPick",
                    "grasp_gradient_optimized": False,
                    "physical_success": False,
                    "full_robot_collision_model_validated": False}
        for filename, document in (("geometry.json", geometry), ("kinematics.json", reference),
                                   ("robot_config.json", robot), ("manifest.json", manifest)):
            (output / filename).write_text(json.dumps(document, indent=2) + "\n")
        return manifest
