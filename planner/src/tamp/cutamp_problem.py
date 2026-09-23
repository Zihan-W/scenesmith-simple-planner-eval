"""Observed-scene continuous problem export for a fixed Zerith skill skeleton.

This builder performs no skeleton search, no optimization and no execution.
NavigateToPick, PickLift and NavigateToPick -> PickLift are supported. The
official MoveFree/Pick pair represents the grasp approach witness; it does not
insert executable skills. A navigation-only program uses Pick as a reachability
witness, matching the existing NavigateToPick domain contract.
"""

import copy
import dataclasses
import hashlib
import json
from pathlib import Path

from .cutamp_backend.geometry_export import export_query_geometry, stationary_support_check
from simulation.src import Pose
from simulation.src.robots.adapters.description import drake_pose

from .subdomains import validate_program_subdomains, restrict_base_candidates, grasp_range, step_bounds
from .cutamp_contact_audits import audit_locked_self_pairs, audit_planar_support
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
        validate_program_subdomains(program, picklift_registry())
        from .task_domain import PICK_LIFT_DOMAIN
        PICK_LIFT_DOMAIN.validate_cutamp_program(program)
        names = tuple(step.skill for step in program.steps)
        if any(step.arguments["object"] != domain.target_name for step in program.steps):
            raise ValueError("The fixed-skeleton adapter supports only the configured target")
        domain.snapshot.require_world(world)
        measured = domain.observation.base["base_link_pose"]
        pose = Pose(tuple(measured["translation_m"]), tuple(measured["quaternion_wxyz"]))
        query = domain.planning_query_at(pose)
        spec = query.robot_adapter.spec
        if hashlib.sha256(spec.model_path.read_bytes()).hexdigest() != self.reference["urdf_sha256"]:
            raise ValueError("Robot model changed since the collision/kinematics export")
        robot = copy.deepcopy(self.robot)
        kin = robot["kinematics"]
        reference_kin = self.reference["robot_config"]["kinematics"]
        for key in ("urdf_path", "base_link", "ee_link", "extra_links"):
            if kin[key] != reference_kin[key]:
                raise ValueError(f"Collision and kinematics templates disagree: {key}")
        kin["urdf_path"] = str(spec.model_path.resolve())
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
        base_step = next((step for step in program.steps if step.skill == "NavigateToPick"), None)
        pick_step = next((step for step in program.steps if step.skill == "PickLift"), None)
        geometry["base_candidates_xyz_yaw"] = restrict_base_candidates(
            domain.candidates, step_bounds(program, base_step, "base_pose") if base_step else {})
        geometry["grasp_lateral_range_m"] = grasp_range(
            domain, step_bounds(program, pick_step, "grasp_pose") if pick_step else {})
        geometry["declared_subdomains"] = getattr(program, "parameter_subdomains", {})
        geometry["locked_self_collision_audit"] = audit_locked_self_pairs(query, geometry, robot)
        if names == ("PickLift",):
            geometry["stationary_robot_support_check"] = stationary_support_check(query, geometry)
        else:
            geometry["planar_robot_support_check"] = audit_planar_support(query, geometry, robot)
            geometry["stationary_robot_support_check"] = geometry["planar_robot_support_check"]["stationary_audit"]
        if not geometry["stationary_target_world_check"]["valid"]:
            raise ValueError("Observed target violates the unchanged static-world contact checks")
        output = Path(output_root)
        output.mkdir(parents=True, exist_ok=False)
        manifest = {"scope": "fixed_skeleton_observed_scene_continuous_problem",
                    "program": dataclasses.asdict(program), "observation_id": world.observation_id,
                    "optimize_base_xy": names[0] == "NavigateToPick",
                    "grasp_gradient_optimized": False,
                    "pick_is_navigation_witness_only": names == ("NavigateToPick",),
                    "physical_success": False,
                    "full_robot_collision_model_validated": False}
        for filename, document in (("geometry.json", geometry), ("kinematics.json", reference),
                                   ("robot_config.json", robot), ("manifest.json", manifest)):
            (output / filename).write_text(json.dumps(document, indent=2) + "\n")
        return manifest
