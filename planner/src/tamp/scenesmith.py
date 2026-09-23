"""SceneSmith geometric constraints for mobile NavigateTo + PickLift skills.

This adapter evaluates VLM-proposed skills against the same Drake scene and
robot model used by scene-eval.  It does not certify contact dynamics: a
grounded plan must still be tested by the normal episode runner.
"""

from __future__ import annotations

import dataclasses
import json
import math
import time

from .geometry import GeometryDeadlineExceeded, require_time_remaining
from collections.abc import Mapping
from pathlib import Path

from pydrake.common.eigen_geometry import Quaternion

from planner.src.skills.runtime import SkillInvocation
from planner.src.skills.navigation import certified_local_map
from planner.src.tamp.planner import Subgoal
from simulation.src import PickLiftPolicyConfig, Pose, build_planning_query
from simulation.src.robots.adapters.description import drake_pose, public_pose
from simulation.src.robots.adapters.zerith_mobile import ZerithMobileRobotAdapter
from simulation.src.geometry.contact import CarriedBody, PairContactPolicy, SupportContactPolicy
from simulation.src.geometry.planning import IkResult


def _yaw_pose(x: float, y: float, z: float, yaw: float) -> Pose:
    return Pose((x, y, z), (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))


def target_relative_base_candidates(base_pose: Pose, target_pose: Pose):
    """Return the existing constant-heading base domain in world meters/radians."""
    q = base_pose.quaternion_wxyz
    yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                     1 - 2 * (q[2] ** 2 + q[3] ** 2))
    target = target_pose.translation_m
    forward = (math.cos(yaw), math.sin(yaw))
    lateral = (-forward[1], forward[0])
    return tuple((round(target[0] - forward[0] * distance + lateral[0] * offset, 4),
                  round(target[1] - forward[1] * distance + lateral[1] * offset, 4), yaw)
                 for distance in (0.67, 0.62, 0.72, 0.57, 0.77)
                 for offset in (-0.18, 0.0, 0.18, -0.09, 0.09))


class SceneSmithPickDomain:
    """Sample stop poses, check swept navigation and target-relative arm IK."""

    skill_names = frozenset({"NavigateToPick", "PickLift"})
    lift_ik_tolerance_m = 0.003

    def _lift_distance(self):
        """Use the shared skill's lift target and reserve the IK error budget."""
        return max(self.nominal_lift_distance_m,
                   self.task_config.required_lift_m + 2 * self.lift_ik_tolerance_m)

    @property
    def program_schema(self):
        return {
            "NavigateToPick": {
                "required": ["base_x_m", "base_y_m", "base_yaw_rad"],
                "units": {"base_x_m": "m", "base_y_m": "m",
                          "base_yaw_rad": "rad"},
            },
            "PickLift": {
                "required": ["target", "arm"],
                "optional": ["grasp_lateral_offset_m", "approach_height_offset_m",
                             "approach_segments", "grasp_arm_joint_positions"],
                "grasp_lateral_offset_m_range": [-0.02, 0.02],
                "arm_values": ["left"],
            },
        }

    @property
    def sampler_hints(self):
        return {"base_candidates_xyz_yaw": [list(pose) for pose in self.candidates],
                "target": self.target_name}

    def __init__(self, *, environment_config, observation, calibration_path: Path,
                 pick_home_path: Path,
                 base_candidates: tuple[tuple[float, float, float], ...] = (),
                 open_width_m: float | None = None,
                 lift_distance_m: float | None = None,
                 planning_query=None):
        self.config = environment_config
        from .snapshot import PlanningSnapshot
        self.snapshot = PlanningSnapshot(observation, planning_query)
        self.calibration = json.loads(Path(calibration_path).read_text())
        self.pick_home = json.loads(Path(pick_home_path).read_text())
        self.candidates = tuple(base_candidates)
        task = self.config.task_factory()
        self.task_config = getattr(task, "task", task).config
        if lift_distance_m is None:
            lift_distance_m = PickLiftPolicyConfig.lift_distance_m
        if not math.isfinite(lift_distance_m) or lift_distance_m <= 0:
            raise ValueError("Lift distance must be positive and finite")
        self.nominal_lift_distance_m = lift_distance_m
        self.target_name = self.task_config.target_observation_name
        if self.target_name not in observation.objects:
            raise ValueError(f"Target is absent from reset observation: {self.target_name}")
        if not isinstance(self.config.robot_adapter, ZerithMobileRobotAdapter):
            raise TypeError("NavigateToPick requires the mobile Zerith adapter")
        gripper = self.config.robot_adapter.spec.grippers["left"]
        self.open_width_m = gripper.maximum_width_m if open_width_m is None else open_width_m
        # Use the same public width-to-joint mapping as the runtime.
        self.config.robot_adapter.gripper_position_targets(self.open_width_m, "left")
        if not self.candidates:
            self.candidates = self._automatic_base_candidates()

    @property
    def observation(self):
        """Return the snapshot observation without exposing its owned storage."""
        return self.snapshot.observation

    def _automatic_base_candidates(self):
        """Generate bounded target-relative poses; physical checks decide feasibility.

        Preserve the observed final heading and search several reach distances
        and lateral offsets. The corridor checker also checks travel headings.
        """
        pose = self.observation.base["base_link_pose"]
        return target_relative_base_candidates(
            Pose(tuple(pose["translation_m"]), tuple(pose["quaternion_wxyz"])),
            self.observation.objects[self.target_name].pose)

    def samples(self, subgoal: Subgoal, state: Mapping):
        if subgoal.skill == "NavigateToPick":
            for x, y, yaw in self.candidates:
                yield {"base_x_m": x, "base_y_m": y, "base_yaw_rad": yaw}
        elif subgoal.skill == "PickLift":
            opening = self.config.robot_adapter.spec.grippers["left"].maximum_width_m
            for offset in (0.0, -0.003, 0.003, -0.006, 0.006,
                           -0.0015, 0.0015, -0.0045, 0.0045):
                for approach_height, segments in ((0.0, 1), (0.0, 8),
                                                  (0.5 * opening, 8), (opening, 8)):
                    yield {"target": self.target_name, "arm": "left",
                           "grasp_lateral_offset_m": offset,
                           "approach_height_offset_m": approach_height,
                           "approach_segments": segments}

    @staticmethod
    def parameter_control_keys(parameter):
        """Candidate controls defining each open parameter in this skill domain."""
        return {
            "base_pose": ("base_x_m", "base_y_m", "base_yaw_rad"),
            "grasp_pose": ("grasp_lateral_offset_m",),
            "approach_pose": ("approach_height_offset_m", "approach_segments"),
        }[parameter]

    def sample_candidate(self, subgoal, state, rng):
        """One CCSP draw: continuous base/grasp and discrete calibrated approach.

        Uses the same measured base domain and checked offset range as the
        existing SceneSmith contract. Numeric ranges are not model solutions.
        """
        if subgoal.skill == "NavigateToPick":
            first = rng.choice(self.candidates)
            # Convex combinations stay inside the existing measured candidate
            # envelope. Preserve the measured final heading.
            second = rng.choice([pose for pose in self.candidates if pose[2] == first[2]])
            alpha = rng.random()
            x = alpha * first[0] + (1 - alpha) * second[0]
            y = alpha * first[1] + (1 - alpha) * second[1]
            yaw = first[2]
            return {"base_x_m": x, "base_y_m": y, "base_yaw_rad": yaw}
        if subgoal.skill == "PickLift":
            candidate = dict(rng.choice(tuple(self.samples(subgoal, state))))
            low, high = self.program_schema["PickLift"]["grasp_lateral_offset_m_range"]
            candidate["grasp_lateral_offset_m"] = rng.uniform(low, high)
            opening = self.config.robot_adapter.spec.grippers["left"].maximum_width_m
            candidate["approach_height_offset_m"] = rng.uniform(0.0, opening)
            return candidate
        raise ValueError(f"No CCSP sampler for skill: {subgoal.skill}")

    def _placed_adapter(self, base_pose):
        """Place the robot model without treating measurements as home commands."""
        adapter = self.config.robot_adapter
        spec = dataclasses.replace(adapter.spec, base_pose=base_pose)
        return ZerithMobileRobotAdapter(spec, adapter.base_config)

    def _synchronize_observation(self, query):
        query.set_observed_joint_positions(dict(zip(
            self.observation.robot.joint_names, self.observation.robot.q, strict=True)))
        query.set_observed_body_poses({name: item.pose
                                       for name, item in self.observation.objects.items()})

    def _query_at_base(self, adapter):
        """Use a live full-state snapshot, or the existing offline observation input."""
        query = self.snapshot.fork_query()
        if query is not None:
            query.set_robot_base_pose(adapter.spec.base_pose)
            return query
        # Offline callers have no environment; preserve their observation-only API.
        query = build_planning_query(scenario=self.config.scenario,
                                     robot_adapter=adapter, timing=self.config.timing)
        self._synchronize_observation(query)
        return query

    def planning_query_at(self, base_pose: Pose):
        """Return an independently mutable candidate query from this snapshot."""
        return self._query_at_base(self._placed_adapter(base_pose))

    def _candidate_query(self, base_pose: Pose):
        placed = self._placed_adapter(base_pose)
        return self._query_at_base(placed), placed

    def rank_candidate(self, subgoal, parameters, checks):
        """Prefer balanced finger gaps, then lift margin and centered offsets."""
        if subgoal.skill != "PickLift":
            return ()
        waypoints = checks["ik"]["lift_waypoints_world"]["waypoints"]
        margin = min(point["min_joint_margin_rad"] for point in waypoints)
        imbalance = checks["ik"]["grasp_contacts"]["gap_imbalance_m"]
        return (imbalance, -margin, abs(parameters.get("grasp_lateral_offset_m", 0.0)))

    def _grasp_contact_policy(self, *, carried_bodies=()):
        """Match planned grasp contact permissions to the physical task."""
        target_body = self.task_config.target_contact_body
        fingers = self.task_config.gripper_contact_bodies
        grasp = PairContactPolicy.from_pairs(
            "planned_pick_grasp",
            ((finger, target_body) for finger in fingers),
            carried_bodies=carried_bodies,
            maximum_allowed_penetration_m=(
                self.task_config.maximum_allowed_contact_penetration_m),
        )
        supports = self.task_config.support_contact_bodies
        if not supports:
            return grasp
        bound = (self.task_config.maximum_allowed_support_penetration_m
                 or self.task_config.maximum_allowed_contact_penetration_m)
        return SupportContactPolicy(grasp, {
            tuple(sorted((body, target_body))): bound for body in supports
        })

    def _check_supplied_grasp_configuration(self, query, desired, seed, values):
        """Validate solver-supplied arm joints without replacing them with IK.

        Match the existing IK position-box and orientation constraints, then
        keep the original configuration for the normal dense edge, contact and
        lift checks. Success here is not permission to skip those later checks.
        """
        spec = query.robot_adapter.spec
        arm_names = spec.arm_groups["left"]
        if len(values) != len(arm_names) or not all(math.isfinite(float(v)) for v in values):
            raise ValueError("Supplied grasp must contain one finite value per left-arm joint")
        configuration = list(seed)
        for name, value in zip(arm_names, values, strict=True):
            configuration[spec.controlled_joint_names.index(name)] = float(value)
        check = query.check_configuration(configuration, contact_policy=self._grasp_contact_policy())
        actual = query.frame_pose(spec.model_instance_name, spec.end_effector_frames["left"])
        deltas = tuple(a - b for a, b in zip(actual.translation_m, desired.translation_m, strict=True))
        orientation_error = (drake_pose(desired).rotation().inverse()
                             @ drake_pose(actual).rotation()).ToAngleAxis().angle()
        pose_valid = max(abs(value) for value in deltas) <= 0.001 and orientation_error <= math.radians(2.0)
        reason = ("success" if check.valid and pose_valid else
                  "endpoint_collision_or_clearance" if not check.valid else "supplied_grasp_pose_error")
        return IkResult(
            success=bool(check.valid and pose_valid), reason=reason,
            configuration=tuple(configuration), solver_result="external_configuration_checked_without_ik",
            position_error_m=math.sqrt(sum(value * value for value in deltas)),
            orientation_error_rad=float(orientation_error), clearance=check.clearance)

    def _navigation_goal(self, base_pose: Pose, start_pose: Pose | None = None):
        if start_pose is None:
            measured = self.observation.base["base_link_pose"]
            start_pose = Pose(tuple(measured["translation_m"]), tuple(measured["quaternion_wxyz"]))
        adapter = self._placed_adapter(start_pose)
        query = self._query_at_base(adapter)
        plant, context = query.plant, query.context
        instance = query.robot_model_instance
        base = plant.GetFrameByName(adapter.spec.base_link_name, instance)
        navigation = plant.GetFrameByName(adapter.navigation_frame_name, instance)
        X_BN = base.CalcPoseInWorld(context).inverse() @ navigation.CalcPoseInWorld(context)
        X_WN = drake_pose(base_pose) @ X_BN
        x, y = X_WN.translation()[:2]
        yaw = X_WN.rotation().ToRollPitchYaw().yaw_angle()
        goal = SkillInvocation("NavigateTo", (str(x), str(y), str(yaw), "world"))
        return query, goal

    def set_deadline(self, deadline_monotonic_s):
        """Bound nested witness search, without weakening any native check."""
        self.deadline_monotonic_s = deadline_monotonic_s

    def _require_time(self):
        require_time_remaining(getattr(self, "deadline_monotonic_s", None))

    def _timed_check(self, label, function, *args, **kwargs):
        self._require_time()
        started = time.perf_counter()
        try:
            return function(*args, **kwargs)
        finally:
            work = getattr(self, "search_work", {})
            item = work.setdefault(label, {"calls": 0, "wall_time_s": 0.0})
            item["calls"] += 1
            item["wall_time_s"] += time.perf_counter() - started
            self.search_work = work
            self._require_time()

    def check_navigation_corridor(self, subgoal, parameters, state):
        """Check only the original corridor; not a complete navigation certificate.

        Callers must also attach a complete target-specific PickLift witness
        before returning a usable NavigateToPick action.
        """
        self._require_time()
        if subgoal.arguments.get("target") != self.target_name:
            return False, "unknown_target", {}
        base_pose = _yaw_pose(parameters["base_x_m"], parameters["base_y_m"],
                              state["base_height_m"], parameters["base_yaw_rad"])
        try:
            query, goal = self._navigation_goal(base_pose, state.get("base_pose"))
            self._timed_check("navigation_corridor", certified_local_map, query, self.config.robot_adapter,
                                 self.config.scenario, goal)
        except GeometryDeadlineExceeded:
            raise
        except (ValueError, RuntimeError) as error:
            return False, "navigation_geometry", {"message": str(error)}
        return True, "valid", {"navigation_goal": list(goal.args),
                                "base_xyz_yaw": [parameters["base_x_m"], parameters["base_y_m"],
                                                 parameters["base_yaw_rad"]],
                                "swept_robot_checked": True}

    def check(self, subgoal: Subgoal, parameters: Mapping, state: Mapping):
        self._require_time()
        if subgoal.arguments.get("target") != self.target_name:
            return False, "unknown_target", {"target": subgoal.arguments.get("target")}
        if subgoal.skill == "NavigateToPick":
            valid, reason, corridor = self.check_navigation_corridor(subgoal, parameters, state)
            if not valid:
                return valid, reason, corridor
            # A stop for picking needs a complete geometric witness before
            # moving. This does not add a symbolic fact or authorize execution
            # of the witness: measured-state PickLift is solved again on arrival.
            predicted = self.predict(subgoal, parameters, dict(state))
            pick = Subgoal("PickLift", {"target": self.target_name})
            failures = []
            for candidate in self.samples(pick, predicted):
                feasible, reason, details = self._timed_check(
                    "navigation_pick_witness", self.check, pick, candidate, predicted)
                if feasible:
                    break
                failures.append({"parameters": candidate, "reason": reason, "details": details})
            else:
                return False, "navigation_pick_infeasible", {"pick_failures": failures}
            return True, "valid", {**corridor,
                                    "pick_witness": {"parameters": candidate, "checks": details}}

        if subgoal.skill == "PickLift":
            if parameters["target"] != self.target_name or parameters["arm"] != "left":
                return False, "skill_binding", {}
            lateral_offset = float(parameters.get("grasp_lateral_offset_m", 0.0))
            if not math.isfinite(lateral_offset) or abs(lateral_offset) > 0.02:
                return False, "invalid_grasp_lateral_offset", {}
            approach_height = float(parameters.get("approach_height_offset_m", 0.0))
            opening = self.config.robot_adapter.spec.grippers["left"].maximum_width_m
            if not math.isfinite(approach_height) or not 0 <= approach_height <= opening:
                return False, "invalid_approach_height", {}
            segments = parameters.get("approach_segments", 1)
            if type(segments) is not int or not 1 <= segments <= 16:
                return False, "invalid_approach_segments", {}
            base = state.get("base_pose")
            if base is None:
                return False, "base_not_parked", {}
            try:
                query, adapter = self._candidate_query(base)
                target = self.observation.objects[self.target_name].pose
                frame = adapter.spec.end_effector_frames["left"]
                arm_names = adapter.spec.arm_groups["left"]
                joint_names = adapter.spec.controlled_joint_names
                arm_indices = [joint_names.index(name) for name in arm_names]
                # A failed skill may leave the arm/gripper far from calibration.
                # Reusing a query must not reuse its last evaluated joint state.
                measured = dict(zip(self.observation.robot.joint_names,
                                    self.observation.robot.q, strict=True))
                start = [measured[name] for name in joint_names]
                seed = list(start)
                for name, position in adapter.gripper_position_targets(
                        self.open_width_m, "left").items():
                    seed[joint_names.index(name)] = position
                checks = {"initial_joint_positions": start,
                          "open_width_m": self.open_width_m,
                          "observation_time_s": self.observation.time_s}
                staging_pose = None
                for label in ("staging_pose_in_target", "grasp_pose_in_target"):
                    relative = self.calibration[label]
                    translation = list(relative["translation_m"])
                    # Move the entire approach with the grasp, so lateral
                    # sampling also changes the first (staging) IK target.
                    translation[1] += lateral_offset
                    if label == "staging_pose_in_target":
                        translation[2] += approach_height
                    offset = Pose(tuple(translation),
                                  tuple(relative["quaternion_wxyz"]))
                    desired = public_pose(drake_pose(target) @ drake_pose(offset))
                    poses = (desired,)
                    if staging_pose is not None and segments > 1:
                        poses = tuple(Pose(
                            tuple(a + (b - a) * index / segments for a, b in zip(
                                staging_pose.translation_m, desired.translation_m, strict=True)),
                            tuple(Quaternion(staging_pose.quaternion_wxyz).slerp(
                                index / segments, Quaternion(desired.quaternion_wxyz)).wxyz()),
                        ) for index in range(1, segments + 1))
                    approach_waypoints = []
                    for waypoint, waypoint_pose in enumerate(poses):
                        supplied = parameters.get("grasp_arm_joint_positions")
                        if label == "grasp_pose_in_target" and waypoint == len(poses) - 1 and supplied is not None:
                            result = self._check_supplied_grasp_configuration(query, waypoint_pose, seed, supplied)
                        else:
                            result = self._timed_check("ik", query.solve_ik,
                                waypoint_pose, frame_name=frame, seed=seed,
                                position_tolerance_m=0.001,
                                orientation_tolerance_rad=math.radians(2.0),
                                contact_policy=self._grasp_contact_policy(),
                            )
                        checks[label] = {"success": result.success, "reason": result.reason,
                                         "solver_result": result.solver_result,
                                         "clearance": dataclasses.asdict(result.clearance),
                                         "position_error_m": result.position_error_m,
                                         "orientation_error_rad": result.orientation_error_rad,
                                         "waypoint": waypoint,
                                         "arm_joint_positions": [result.configuration[i]
                                                                 for i in arm_indices]}
                        if not result.success:
                            return False, "pick_ik_" + label, checks
                        edge = self._timed_check("joint_edge", query.check_edge,
                            start, result.configuration,
                            contact_policy=self._grasp_contact_policy(),
                            maximum_joint_step=0.01,
                        )
                        checks[label]["joint_edge_valid"] = edge.valid
                        checks[label]["joint_edge"] = dataclasses.asdict(edge)
                        if not edge.valid:
                            return False, "pick_joint_edge_" + label, checks
                        approach_waypoints.append({
                            "joint_positions": [result.configuration[i] for i in arm_indices],
                            "joint_edge": dataclasses.asdict(edge),
                        })
                        seed = list(result.configuration)
                        start = list(seed)
                    if staging_pose is None:
                        staging_pose = query.frame_pose_at(
                            seed, adapter.spec.model_instance_name, frame)
                    elif segments > 1:
                        checks["approach_waypoints_world"] = approach_waypoints
                distances = {}
                for pair in query.collision_pairs(
                        seed, influence_distance_m=adapter.spec.grippers["left"].maximum_width_m):
                    bodies = {pair.body_a, pair.body_b}
                    for finger in self.task_config.gripper_contact_bodies:
                        if bodies == {finger, self.task_config.target_contact_body}:
                            distances[finger] = min(distances.get(finger, math.inf), pair.distance_m)
                if len(distances) != len(self.task_config.gripper_contact_bodies):
                    checks["grasp_contacts"] = {"open_finger_distances_m": distances}
                    return False, "pick_target_outside_gripper", checks
                checks["grasp_contacts"] = {
                    "open_finger_distances_m": distances,
                    "gap_imbalance_m": max(distances.values()) - min(distances.values()),
                    "dynamics_pending": True,
                }
                grasp_pose = query.frame_pose_at(
                    seed, adapter.spec.model_instance_name, frame)
                carried = CarriedBody(
                    body_name=self.task_config.target_contact_body,
                    carrier_frame_name=frame,
                    body_pose_world=target,
                    carrier_pose_world=grasp_pose,
                )
                lift_contacts = self._grasp_contact_policy(carried_bodies=(carried,))
                # Start at the solved grasp, not its nominal IK target. Otherwise
                # endpoint tolerances can silently shrink an 8 cm lift to 7.6 cm.
                desired = grasp_pose
                lift_distance = self._lift_distance()
                step_count = max(8, math.ceil(
                    lift_distance / (self.task_config.required_lift_m / 8)))
                if step_count > 32:
                    return False, "pick_lift_path_too_long", checks
                lift_desired = Pose(
                    (desired.translation_m[0], desired.translation_m[1],
                     desired.translation_m[2] + lift_distance),
                    desired.quaternion_wxyz,
                )
                result = self._timed_check("ik", query.solve_ik,
                    lift_desired, frame_name=frame, seed=seed,
                    position_tolerance_m=self.lift_ik_tolerance_m,
                    orientation_tolerance_rad=math.radians(2.0),
                    contact_policy=lift_contacts,
                )
                checks["lift_pose_world"] = {
                    "success": result.success, "reason": result.reason,
                    "position_error_m": result.position_error_m,
                    "orientation_error_rad": result.orientation_error_rad,
                }
                if not result.success:
                    return False, "pick_ik_lift_pose_world", checks
                limits = query.joint_limits()
                waypoints = []
                previous_z = desired.translation_m[2]
                for index in range(1, step_count + 1):
                    height = lift_distance * index / step_count
                    pose = Pose((desired.translation_m[0],
                                 desired.translation_m[1],
                                 desired.translation_m[2] + height),
                                desired.quaternion_wxyz)
                    candidate = self._timed_check("ik", query.solve_ik,
                        pose, frame_name=frame, seed=seed,
                        position_tolerance_m=self.lift_ik_tolerance_m,
                        orientation_tolerance_rad=math.radians(2.0),
                        contact_policy=lift_contacts,
                    )
                    if not candidate.success:
                        checks["lift_waypoints_world"] = {
                            "success": False, "step": index,
                            "height_m": height, "reason": candidate.reason}
                        return False, "pick_ik_lift_waypoint", checks
                    joint_delta = max(abs(candidate.configuration[i] - seed[i])
                                      for i in arm_indices)
                    margin = min(
                        min(candidate.configuration[i] - limits[name][0],
                            limits[name][1] - candidate.configuration[i])
                        for name, i in zip(arm_names, arm_indices, strict=True)
                    )
                    actual_pose = query.frame_pose_at(
                        candidate.configuration,
                        adapter.spec.model_instance_name, frame)
                    if (joint_delta > 0.4 or margin < 0.005
                            or actual_pose.translation_m[2] < previous_z + 0.003):
                        checks["lift_waypoints_world"] = {
                            "success": False, "step": index,
                            "height_m": height,
                            "max_joint_delta_rad": joint_delta,
                            "min_joint_margin_rad": margin,
                            "actual_z_m": actual_pose.translation_m[2]}
                        return False, "pick_lift_path_continuity", checks
                    edge = self._timed_check("joint_edge", query.check_edge,
                        seed, candidate.configuration, contact_policy=lift_contacts,
                        maximum_joint_step=0.01,
                    )
                    if not edge.valid:
                        checks["lift_waypoints_world"] = {
                            "success": False, "step": index,
                            "height_m": height, "joint_edge": dataclasses.asdict(edge)}
                        return False, "pick_joint_edge_lift_waypoint", checks
                    waypoints.append({
                        "height_m": height,
                        "joint_positions": [candidate.configuration[i]
                                            for i in arm_indices],
                        "min_joint_margin_rad": margin,
                        "joint_edge": dataclasses.asdict(edge),
                    })
                    seed = list(candidate.configuration)
                    previous_z = actual_pose.translation_m[2]
                carried_final = (drake_pose(actual_pose) @ drake_pose(grasp_pose).inverse()
                                 @ drake_pose(target))
                actual_lift = float(carried_final.translation()[2] - target.translation_m[2])
                checks["lift_goal"] = {
                    "required_lift_m": self.task_config.required_lift_m,
                    "commanded_lift_m": lift_distance,
                    "predicted_carried_lift_m": actual_lift,
                    "dynamics_pending": True,
                }
                if actual_lift < self.task_config.required_lift_m:
                    return False, "pick_lift_goal_not_met", checks
                checks["lift_waypoints_world"] = {"success": True,
                                                   "carried_object_checked": True,
                                                   "gripper_geometry": "planned_open_width",
                                                   "arm_joint_names": list(arm_names),
                                                   "waypoints": waypoints}
            except GeometryDeadlineExceeded:
                raise
            except (ValueError, RuntimeError) as error:
                return False, "pick_geometry", {"message": str(error)}
            return True, "valid", {"ik": checks, "dynamics_pending": True}
        raise ValueError(f"Unsupported skill: {subgoal.skill}")

    def failure_positions(self, step, index, parameters, reason, details):
        """Expose only checked-control projections, not world poses or joint vectors."""
        from .failures import constraint_category
        from .failure_positions import normalized_failure_positions
        return normalized_failure_positions(self, step, index, parameters,
                                            constraint_category(reason, details))

    def plan_lift_compensation(self, distance_m):
        """Certify one upward correction from the measured grasp attachment.

        This is a geometric certificate, never a prediction of no-slip dynamics.
        The runtime still checks every command and observes task completion.
        """
        self._require_time()
        if not math.isfinite(distance_m) or not 0 < distance_m <= 0.01:
            raise ValueError('Compensation is limited to 10 mm per measured-state plan')
        observation = self.observation
        if (not observation.task.get('bilateral_gripper_contact', False)
                or observation.task.get('unexpected_target_contacts')):
            return None, {'reason': 'compensation_contact_precondition'}
        query = self.snapshot.fork_query()
        if query is None:
            raise ValueError('Compensation requires a synchronized online planning query')
        spec = self.config.robot_adapter.spec
        frame = spec.end_effector_frames['left']
        names = spec.controlled_joint_names
        arm_names = spec.arm_groups['left']
        indices = [names.index(name) for name in arm_names]
        measured = dict(zip(observation.robot.joint_names, observation.robot.q, strict=True))
        commanded = dict(zip(observation.robot.joint_names, observation.robot.q_commanded, strict=True))
        start = tuple(measured[name] for name in names)
        tcp = query.frame_pose_at(start, spec.model_instance_name, frame)
        target = observation.objects[self.target_name].pose
        carried = CarriedBody(self.task_config.target_contact_body, frame, target, tcp)
        contacts = self._grasp_contact_policy(carried_bodies=(carried,))
        desired = Pose((tcp.translation_m[0], tcp.translation_m[1], tcp.translation_m[2] + distance_m),
                       tcp.quaternion_wxyz)
        checks = {'observation_time_s': observation.time_s, 'snapshot_token': self.snapshot.token,
                  'distance_m': distance_m, 'measured_target_pose': target.as_dict(),
                  'measured_tcp_pose': tcp.as_dict(), 'dynamics_pending': True,
                  'position_tolerance_m': .0005, 'orientation_tolerance_rad': math.radians(.5)}
        result = self._timed_check('compensation_ik', query.solve_ik, desired,
            # A 3 mm nominal-lift tolerance absorbs half a 6 mm correction;
            # loaded servo offset can absorb the remainder. Certify the small
            # correction to 0.5 mm / 0.5 degrees before runtime feedback.
            frame_name=frame, seed=start, position_tolerance_m=checks['position_tolerance_m'],
            orientation_tolerance_rad=checks['orientation_tolerance_rad'], contact_policy=contacts)
        checks['ik'] = dataclasses.asdict(result)
        if not result.success:
            return None, {**checks, 'reason': 'compensation_ik_rejected'}
        # Only the arm is commanded; certify that actual command, including
        # unchanged measured finger joints, rather than a whole-robot IK seed.
        goal = list(start)
        for index in indices:
            goal[index] = result.configuration[index]
        limits = query.joint_limits()
        margin = min(min(goal[i] - limits[name][0], limits[name][1] - goal[i])
                     for name, i in zip(arm_names, indices, strict=True))
        actual = query.frame_pose_at(goal, spec.model_instance_name, frame)
        if (max(abs(goal[i] - start[i]) for i in indices) > .4 or margin < .005
                or actual.translation_m[2] < desired.translation_m[2] - checks['position_tolerance_m'] - 1e-9):
            return None, {**checks, 'reason': 'compensation_continuity_rejected'}
        # Uniform clipping starts from q_commanded; check that edge as well as
        # the measured edge. Every subsequent tick is checked by the runtime.
        commanded_arm_start = list(start)
        for name, index in zip(arm_names, indices, strict=True):
            commanded_arm_start[index] = commanded[name]
        # As in Runtime.check_command_edge, compliant finger squeeze targets
        # are not physical geometry. This correction moves only the arm.
        for label, edge_start in (('measured_edge', start),
                                  ('commanded_edge', commanded_arm_start)):
            edge = self._timed_check('compensation_edge', query.check_edge,
                edge_start, goal, contact_policy=contacts, maximum_joint_step=.01)
            checks[label] = dataclasses.asdict(edge)
            if not edge.valid:
                return None, {**checks, 'reason': 'compensation_edge_rejected'}
        checks.update(reason='certified', min_joint_margin_rad=margin,
                      actual_tcp_rise_m=actual.translation_m[2] - tcp.translation_m[2],
                      certified_tcp_pose=actual.as_dict())
        return [tuple(goal[i] for i in indices)], checks

    def predict(self, subgoal: Subgoal, parameters: Mapping, state: dict):
        if subgoal.skill == "NavigateToPick":
            state["base_pose"] = _yaw_pose(parameters["base_x_m"],
                                            parameters["base_y_m"],
                                            state["base_height_m"],
                                            parameters["base_yaw_rad"])
        elif subgoal.skill == "PickLift":
            state["pick_ik_feasible"] = True
        return state
