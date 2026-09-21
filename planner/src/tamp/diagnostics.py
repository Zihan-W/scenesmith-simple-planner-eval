"""Read-only simulation diagnostics; no command, contact or success overrides."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Mapping
from contextlib import contextmanager
import dataclasses
import functools
import math
import time

import numpy as np
from pydrake.all import Role

from simulation.src.robots.adapters.description import drake_pose, public_pose


def json_value(value):
    """Serialize typed commands without deep-copying immutable mapping proxies."""
    if dataclasses.is_dataclass(value):
        return {"type": type(value).__name__, **{
            field.name: json_value(getattr(value, field.name)) for field in dataclasses.fields(value)}}
    if isinstance(value, Mapping):
        return {str(key): json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_value(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def body_identity(plant, body):
    """Return an unambiguous model-qualified body name."""
    return f"{plant.GetModelInstanceName(body.model_instance())}::{body.name()}"


def geometry_identity(plant, inspector, geometry):
    """Bind a geometry ID to its owning body, model and geometry name."""
    body = plant.GetBodyFromFrameId(inspector.GetFrameId(geometry))
    return {"geometry_id": geometry.get_value(), "geometry_name": inspector.GetName(geometry),
            "body_index": int(body.index()), "body": body_identity(plant, body)}


def target_flags(pairs, target, fingers, supports):
    """Classify named pairs; make no inference from a rejected candidate."""
    others = set()
    for a, b in pairs:
        if a == target:
            others.add(b)
        elif b == target:
            others.add(a)
    return {"finger_contacts": [finger in others for finger in fingers],
            "bilateral_gripper_contact": all(finger in others for finger in fingers),
            "support_contact": bool(others & set(supports)),
            "unexpected_target_contacts": sorted(others - set(fingers) - set(supports))}


def measured_contacts(backend, task_config):
    """Read actual ContactResults and instantaneous geometry without mutation.

    A sampled discrete ContactResults output describes the preceding solve. Its
    evaluation timestamp is NOT asserted to be a same-instant geometry timestamp.
    The port does not expose a per-result timestamp; retain this limitation.
    """
    plant, context = backend.plant, backend.plant_context
    now = float(context.get_time())
    query = plant.get_geometry_query_input_port().Eval(context)
    inspector = query.inspector()
    result = plant.get_contact_results_output_port().Eval(context)
    target = task_config.target_contact_body
    fingers, supports = task_config.gripper_contact_bodies, task_config.support_contact_bodies
    responses = []
    for index in range(result.num_point_pair_contacts()):
        contact = result.point_pair_contact_info(index)
        pair = contact.point_pair()
        a, b = geometry_identity(plant, inspector, pair.id_A), geometry_identity(plant, inspector, pair.id_B)
        if target not in (a["body"], b["body"]):
            continue
        # Independently cross-check the force-result body indexes and geometry IDs.
        if (a["body_index"], b["body_index"]) != (int(contact.bodyA_index()), int(contact.bodyB_index())):
            raise RuntimeError("ContactResults geometry/body identity mismatch")
        responses.append({"kind": "point", "a": a, "b": b,
                          "response_geometry_depth_m": float(pair.depth),
                          "force_world_n": contact.contact_force().tolist(),
                          "contact_point_world_m": contact.contact_point().tolist(),
                          "separation_speed_m_s": float(contact.separation_speed()),
                          "slip_speed_m_s": float(contact.slip_speed())})
    for index in range(result.num_hydroelastic_contacts()):
        contact = result.hydroelastic_contact_info(index)
        surface = contact.contact_surface()
        a, b = geometry_identity(plant, inspector, surface.id_M()), geometry_identity(plant, inspector, surface.id_N())
        if target in (a["body"], b["body"]):
            responses.append({"kind": "hydroelastic", "a": a, "b": b,
                              "force_world_n": contact.F_Ac_W().translational().tolist(),
                              "moment_world_nm": contact.F_Ac_W().rotational().tolist()})
    penetrations = []
    for pair in query.ComputePointPairPenetration():
        a, b = geometry_identity(plant, inspector, pair.id_A), geometry_identity(plant, inspector, pair.id_B)
        if target in (a["body"], b["body"]):
            penetrations.append({"a": a, "b": b, "measured_geometry_depth_m": float(pair.depth)})

    def geometries(qualified):
        model, name = qualified.split("::", 1)
        body = plant.GetBodyByName(name, plant.GetModelInstanceByName(model))
        return inspector.GetGeometries(plant.GetBodyFrameIdOrThrow(body.index()), Role.kProximity)

    distances = []
    for finger in fingers:
        for a in geometries(target):
            for b in geometries(finger):
                pair = query.ComputeSignedDistancePairClosestPoints(a, b)
                distances.append({"a": geometry_identity(plant, inspector, a),
                                  "b": geometry_identity(plant, inspector, b),
                                  "measured_distance_m": float(pair.distance)})
    sampled = plant.has_sampled_output_ports()
    return {"geometry_time_s": now, "contact_output_evaluated_at_s": now,
            "sampled_contact_output": sampled, "physics_dt_s": plant.time_step(),
            "contact_timestamp_available": False,
            "response_time_semantics": ("last_discrete_update; not instantaneous geometry" if sampled
                                         else "live output evaluated from current context"),
            "contact_model": str(plant.get_contact_model()),
            "discrete_contact_approximation": str(plant.get_discrete_contact_approximation()),
            "actual_contact_results": responses, "measured_geometry_penetrations": penetrations,
            "measured_finger_target_distances": distances,
            "contact_result_presence": target_flags([(r["a"]["body"], r["b"]["body"]) for r in responses], target, fingers, supports),
            "positive_force_presence": target_flags([(r["a"]["body"], r["b"]["body"]) for r in responses
                                                       if np.linalg.norm(r["force_world_n"]) > 0], target, fingers, supports),
            "geometry_presence": target_flags([(r["a"]["body"], r["b"]["body"]) for r in penetrations], target, fingers, supports)}


def measured_pair_distance(backend, pair):
    """Evaluate a named rejected pair in the actual current simulation context."""
    if pair is None:
        return None
    plant, context = backend.plant, backend.plant_context
    query = plant.get_geometry_query_input_port().Eval(context)
    inspector = query.inspector()
    ids = []
    for suffix in ("a", "b"):
        model, name = pair[f"body_{suffix}"].split("::", 1)
        body = plant.GetBodyByName(name, plant.GetModelInstanceByName(model))
        ids.append(inspector.GetGeometryIdByName(plant.GetBodyFrameIdOrThrow(body.index()),
                                                 Role.kProximity, pair[f"geometry_{suffix}"]))
    return {"time_s": float(context.get_time()),
            "distance_m": float(query.ComputeSignedDistancePairClosestPoints(*ids).distance),
            "a": geometry_identity(plant, inspector, ids[0]), "b": geometry_identity(plant, inspector, ids[1])}


def state_record(env):
    """Record full observed state plus plant q/v; not a restorable snapshot."""
    obs, backend = env.observation, env.backend
    return {"time_s": obs.time_s, "base": json_value(obs.base), "robot": obs.robot.as_dict(),
            "objects": {name: {"pose": obj.pose.as_dict(), "velocity": obj.spatial_velocity.as_dict()}
                        for name, obj in obs.objects.items()},
            "plant_positions": backend.plant.GetPositions(backend.plant_context).tolist(),
            "plant_velocities": backend.plant.GetVelocities(backend.plant_context).tolist(),
            "accepted_joint_command": backend.command.tolist(), "task": json_value(obs.task),
            "restorable_dynamic_snapshot": False}


class WorkMeter:
    """Scoped diagnostic instrumentation, with inclusive AND exclusive wall time."""

    def __init__(self):
        self.stats = defaultdict(lambda: {"calls": 0, "inclusive_s": 0.0, "exclusive_s": 0.0})
        self.stack = []
        self.ik_solver_counts = []
        self.active_ik = []
        self.exit_reasons = defaultdict(int)
        self.events = defaultdict(int)
        self.domain_stack = []

    @contextmanager
    def span(self, name):
        entry = [time.perf_counter(), 0.0]
        self.stack.append(entry)
        try:
            yield
        finally:
            elapsed = time.perf_counter() - entry[0]
            self.stack.pop()
            row = self.stats[name]
            row["calls"] += 1
            row["inclusive_s"] += elapsed
            row["exclusive_s"] += elapsed - entry[1]
            if self.stack:
                self.stack[-1][1] += elapsed

    @contextmanager
    def instrument(self):
        from planner.src.tamp.scenesmith import SceneSmithPickDomain
        from simulation.src.geometry import planning
        from planner.src.bt.generation import OpenAICompatibleChatClient
        from planner.src.tamp.online import JsonlTrace
        from planner.src.tamp.scenesmith_online import SceneSmithSkillExecutor, SceneSmithWorldObserver
        from simulation.src.runtime.runtime import DrakeRuntime
        patches = []

        def wrap(owner, attribute, name):
            original = getattr(owner, attribute)
            @functools.wraps(original)
            def call(*args, **kwargs):
                label = name
                if name == "domain_check":
                    label += ":" + args[1].skill
                    if self.domain_stack and self.domain_stack[-1] == "NavigateToPick":
                        self.events["internal_navigation_pick_witness_checks"] += 1
                    self.domain_stack.append(args[1].skill)
                if name == "jsonl_recording":
                    event = args[1].get("event")
                    if event in ("ccsp_assignment", "ccsp_solved", "ccsp_constraint_check"):
                        self.events[event] += 1
                if name == "ik":
                    self.active_ik.append(0)
                elif name == "ik_solver" and self.active_ik:
                    self.active_ik[-1] += 1
                try:
                    with self.span(label):
                        result = original(*args, **kwargs)
                    if name == "domain_check":
                        self.exit_reasons[result[1]] += 1
                    return result
                finally:
                    if name == "ik":
                        self.ik_solver_counts.append(self.active_ik.pop())
                    if name == "domain_check":
                        self.domain_stack.pop()
            setattr(owner, attribute, call)
            patches.append((owner, attribute, original))

        for method, label in (("solve_ik", "ik"), ("check_edge", "joint_edge"),
                              ("collision_pairs", "geometry_pairs"), ("check_configuration", "configuration")):
            wrap(planning.PlanningQuery, method, label)
        wrap(planning, "Solve", "ik_solver")
        wrap(SceneSmithPickDomain, "check", "domain_check")
        wrap(SceneSmithPickDomain, "sample_candidate", "assignment_sample")
        wrap(OpenAICompatibleChatClient, "complete", "model_call")
        wrap(JsonlTrace, "__call__", "jsonl_recording")
        wrap(SceneSmithSkillExecutor, "execute", "skill_execution_total")
        wrap(SceneSmithWorldObserver, "capture_images", "image_capture_recording")
        wrap(DrakeRuntime, "step", "runtime_step")
        try:
            yield self
        finally:
            for owner, name, original in reversed(patches):
                setattr(owner, name, original)

    def as_dict(self):
        return {"events": dict(self.events), "timings": dict(self.stats), "ik_calls": len(self.ik_solver_counts),
                "ik_solver_calls": sum(self.ik_solver_counts),
                "ik_refinement_calls": sum(max(0, n - 1) for n in self.ik_solver_counts),
                "ik_solver_calls_per_ik": self.ik_solver_counts,
                "check_exit_reasons": dict(self.exit_reasons),
                "time_semantics": "exclusive_s is additive; nested inclusive_s must not be summed"}


class DiagnosticRecorder:
    """Wrap original calls exactly once and align precheck/commit/post-physics."""

    def __init__(self, env, task_config, trace, meter):
        self.env, self.config, self.trace, self.meter = env, task_config, trace, meter
        self.pending = {}
        self.check = {}
        self.last_progress = None
        self.best_width = math.inf
        self.previous_contacts = None
        self.relative_targets = {}

    @contextmanager
    def install(self):
        from planner.src.bt.runtime import JsonBtPolicy
        from planner.src.skills.picklift import JointWaypointPickLiftSkill
        backend = self.env.backend
        original_act, original_expert = JsonBtPolicy.act, JointWaypointPickLiftSkill.act
        original_candidate, original_step = backend._combined_candidate, self.env.step

        def expert(skill, observation):
            command = original_expert(skill, observation)
            self.pending["skill_requested_command"] = json_value(command)
            return command

        def act(policy, observation):
            self.pending = {"request_time_s": observation.time_s, "phase_before":
                            policy.expert.stage if policy.expert else "navigation"}
            command = original_act(policy, observation)
            self.pending.update(bt_requested_command=json_value(command),
                                policy=policy.diagnostics(), closure_state=json_value(policy._leaf_state))
            return command

        def candidate(action, contact_policy):
            command_before = backend.command.copy()
            result, decision = original_candidate(action, contact_policy)
            pair = (decision.get("edge", {}).get("limiting_nonpenetration_pair")
                    if not decision.get("accepted", True) else None)
            self.check = {"check_time_s": float(backend.plant_context.get_time()),
                          "planning_state_time_s": backend.planning.state_time_s,
                          "contact_policy": json_value(contact_policy),
                          "previous_accepted_joint_command": command_before.tolist(),
                          "requested_candidate_joint_command": result.tolist(),
                          "decision": json_value(decision),
                          "measured_rejected_pair_before_step": measured_pair_distance(backend, pair)}
            return result, decision

        def step(command):
            with self.meter.span("diagnostic_capture"):
                before = state_record(self.env)
                precontact = measured_contacts(backend, self.config)
            self.check = {}
            with self.meter.span("simulation_step"):
                result = original_step(command)
            obs = result[0]
            with self.meter.span("diagnostic_capture"):
                after = state_record(self.env)
                contacts = measured_contacts(backend, self.config)
                ee = obs.robot.end_effectors["left"]
                target = obs.objects[self.config.target_observation_name].pose
                relative = public_pose(drake_pose(ee).inverse() @ drake_pose(target))
                flags = tuple(obs.task.get("finger_contacts", ()))
                width = obs.robot.gripper_widths_m["left"]
                events = []
                if width < self.best_width - 0.00001:
                    self.best_width = width
                    events.append("new_minimum_measured_gripper_width")
                if flags != self.previous_contacts and any(flags):
                    events.append("new_reported_finger_contact")
                self.previous_contacts = flags
                if events:
                    self.last_progress = {"time_s": obs.time_s, "events": events, "width_m": width, "finger_flags": flags}
                decision = self.check.get("decision", {})
                pair = (decision.get("edge", {}).get("limiting_nonpenetration_pair")
                        if not decision.get("accepted", True) else None)
                names = backend.spec.controlled_joint_names
                fingers = backend.spec.grippers["left"].joint_names
                requested = self.check.get("requested_candidate_joint_command")
                finger_state = {name: {"measured_position": obs.robot.q[names.index(name)],
                                       "accepted_target": float(backend.command[names.index(name)]),
                                       "requested_target": requested[names.index(name)] if requested else None}
                                for name in fingers}
                desired_tcp = {name: public_pose(drake_pose(target) @ drake_pose(value)).as_dict()
                               for name, value in self.relative_targets.items()}
                row = {"mode": "DIAGNOSTIC_FIXED_GRASP", **self.pending,
                       "pre_state": before, "pre_contact": precontact, "command_check": self.check,
                       "post_state": after, "post_contact": contacts,
                       "accepted_joint_command_after_step": backend.command.tolist(),
                       "actual_target_in_left_tcp": relative.as_dict(),
                       "left_finger_joint_state": finger_state, "desired_tcp_world": desired_tcp,
                       "measured_rejected_pair_after_step": measured_pair_distance(backend, pair),
                       "last_measured_closure_progress": self.last_progress,
                       "task_bilateral_definition": "both configured finger bodies present in observation.contacts",
                       "online_model_success_trial": False}
            with self.meter.span("diagnostic_log_write"):
                self.trace(row)
            return result

        JsonBtPolicy.act, JointWaypointPickLiftSkill.act = act, expert
        backend._combined_candidate, self.env.step = candidate, step
        try:
            yield self
        finally:
            JsonBtPolicy.act, JointWaypointPickLiftSkill.act = original_act, original_expert
            backend._combined_candidate, self.env.step = original_candidate, original_step
