#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
from typing import List, Tuple, Any, Dict
import numpy as np
import re
import copy

import logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

from pydrake.geometry import StartMeshcat, Role, GeometrySet, CollisionFilterDeclaration, RoleAssign, ProximityProperties
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import TrajectorySource, Demultiplexer, Multiplexer, Adder, Gain
from pydrake.trajectories import (
    CompositeTrajectory,
    PathParameterizedTrajectory,
    PiecewisePolynomial,
)
from pydrake.multibody.optimization import Toppra, CalcGridPointsOptions
from pydrake.multibody.plant import MultibodyPlant, CoulombFriction
from pydrake.multibody.tree import BodyIndex
from pydrake.common.yaml import yaml_load, yaml_dump_typed

# Robotic Manipulation course "manipulation" python package.
from manipulation.station import LoadScenario, MakeHardwareStation, MakeMultibodyPlant

# Fix relative paths so contents of the src directory can be imported.
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.item_locking_monitor import ItemLockingMonitorConfig, ApplyItemLockingMonitorConfig

def _ensure_model_drivers(yaml_text: str) -> str:
    if "model_drivers:" in yaml_text:
        return yaml_text.rstrip() + "\n"
    drivers_block = """
model_drivers:
  mobile_iiwa: !InverseDynamicsDriver {}
  wsg_50: !SchunkWsgDriver {}

plant_config:
  time_step: 5e-3
  contact_model: hydroelastic_with_fallback
  discrete_contact_approximation: lagged
"""
    return yaml_text.rstrip() + "\n" + drivers_block.lstrip()


def _load_plan(plan_path: Path) -> Tuple[np.ndarray, int, List[str]]:
    data = json.loads(plan_path.read_text())
    q_list = data["q"]
    nq = int(data["nq"])
    segments = list(data.get("segments", []))
    q = np.array(q_list, dtype=float)
    if q.ndim != 2 or q.shape[1] != nq:
        raise ValueError(f"Plan q has shape {q.shape}, expected (N,{nq}).")
    return q, nq, segments


def _pl_path_to_traj(path: np.ndarray) -> CompositeTrajectory:
    """Turn waypoint list into a CompositeTrajectory with unit-time segments.

    Uses cubic segments with C2 continuity imposed locally via endpoint second-derivs = 0.
    """
    nq = path.shape[1]
    segments = []
    for i in range(1, path.shape[0]):
        # Each segment is [i-1, i] in time (unit duration).
        seg = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            np.array([float(i - 1), float(i)]),
            np.array([path[i - 1], path[i]]).T,
            np.zeros(nq),
            np.zeros(nq),
        )
        segments.append(seg)
    return CompositeTrajectory(segments)


def _retime_toppra(
    plant,
    rough_traj,
    n_gridpoints=200,
    ee_vel_limit=None,
    ee_accel_limit=None,
):
    t0 = rough_traj.start_time()
    t1 = rough_traj.end_time()
    gridpoints = Toppra.CalcGridPoints(rough_traj, CalcGridPointsOptions(min_points=n_gridpoints))

    vel_multiplier = 1.0
    accel_multiplier = 1.0
    effort_multiplier = 1.0

    toppra = Toppra(rough_traj, plant, gridpoints)

    vel_lower = plant.GetVelocityLowerLimits() * vel_multiplier
    vel_upper = plant.GetVelocityUpperLimits() * vel_multiplier

    gripper_close_time = 5 # seconds
    gripper_distance = 0.05 - 0.005
    gripper_vel_limit = gripper_distance / gripper_close_time
    vel_lower[-2:] = -gripper_vel_limit
    vel_upper[-2:] = gripper_vel_limit

    toppra.AddJointVelocityLimit(
        vel_lower,
        vel_upper,
    )

    toppra.AddJointAccelerationLimit(
        accel_multiplier * plant.GetAccelerationLowerLimits(),
        accel_multiplier * plant.GetAccelerationUpperLimits(),
    )
    toppra.AddJointTorqueLimit(
        effort_multiplier * plant.GetEffortLowerLimits(),
        effort_multiplier * plant.GetEffortUpperLimits(),
    )

    # ----------------------------
    # End-effector (frame) limits (optional)
    # Spatial velocity / acceleration measured AND expressed in world frame.
    # ----------------------------
    if ee_vel_limit is not None or ee_accel_limit is not None:
        wsg_instance = plant.GetModelInstanceByName("wsg_50")
        ee_frame = plant.GetFrameByName("body", wsg_instance)  # wsg_50::body

        if ee_vel_limit is not None:
            s = float(ee_vel_limit)
            lower_v = -s * np.ones(6)
            upper_v =  s * np.ones(6)
            toppra.AddFrameVelocityLimit(ee_frame, lower_v, upper_v)

        if ee_accel_limit is not None:
            a = float(ee_accel_limit)
            lower_a = -a * np.ones(6)
            upper_a =  a * np.ones(6)
            toppra.AddFrameAccelerationLimit(ee_frame, lower_a, upper_a)

    time_traj = toppra.SolvePathParameterization()
    if time_traj is None:
        raise RuntimeError("TOPPRA failed to find a time parameterization.")
    return PathParameterizedTrajectory(rough_traj, time_traj)


def _make_piecewise_hold(times: np.ndarray, samples: np.ndarray) -> PiecewisePolynomial:
    """Piecewise-linear hold trajectory through samples at given times.
    samples: shape (N, d)
    returns: PiecewisePolynomial.FirstOrderHold
    """
    return PiecewisePolynomial.FirstOrderHold(times, samples.T)


def body_names_for_instance(plant: MultibodyPlant, model_instance_name):
    return [
        plant.get_body(body_index).name()
        for body_index in plant.GetBodyIndices(plant.GetModelInstanceByName(model_instance_name))
    ]


def _make_angleaxis_block(indent: str, angle_deg: float, axis):
    """
    indent: indentation for the 'rotation:' line
    axis: iterable of 3 floats
    """
    ax = [_format_float(float(axis[0])), _format_float(float(axis[1])), _format_float(float(axis[2]))]
    return [
        f"{indent}rotation: !AngleAxis\n",
        f"{indent}  angle_deg: {_format_float(float(angle_deg))}\n",
        f"{indent}  axis: [{ax[0]}, {ax[1]}, {ax[2]}]\n",
    ]


def _make_translation_line(indent: str, p):
    vals = [_format_float(float(p[0])), _format_float(float(p[1])), _format_float(float(p[2]))]
    return f"{indent}translation: [{vals[0]}, {vals[1]}, {vals[2]}]\n"


def _indent_of(line: str) -> str:
    return line[: len(line) - len(line.lstrip(" "))]


def _format_float(x: float) -> str:
    # Keep it readable and stable-ish; adjust if you prefer more/less precision.
    return f"{x:.16g}"

def update_default_free_body_pose_text_in_place(
    yaml_text: str,
    plant,
    plant_context,
) -> str:
    """
    Pure text rewrite:
    - Only edits add_model.default_free_body_pose.<body>.(translation, rotation)
    - Leaves welds and everything else unchanged
    - Does not use Drake/PyYAML YAML parsing
    """

    lines = yaml_text.splitlines(keepends=True)

    # State for scanning
    in_add_model = False
    add_model_indent = None  # indent string for "- add_model:"
    current_model_name = None

    in_dfbp = False
    dfbp_indent = None  # indent string for "default_free_body_pose:"
    current_body_name = None
    body_indent = None  # indent string for "base_link:" line (body key)

    i = 0
    out = []

    # Helpers to detect block boundaries by indentation
    def indent_len(s): return len(_indent_of(s))

    while i < len(lines):
        line = lines[i]
        stripped = line.lstrip(" ")

        # Detect start of an add_model item: "- add_model:"
        if re.match(r"^\s*-\s+add_model:\s*$", line):
            in_add_model = True
            in_dfbp = False
            current_body_name = None
            body_indent = None
            current_model_name = None
            add_model_indent = _indent_of(line)
            out.append(line)
            i += 1
            continue

        # If we hit a new directive item, end previous add_model context
        if re.match(r"^\s*-\s+\w+:\s*$", line) and not re.match(r"^\s*-\s+add_model:\s*$", line):
            in_add_model = False
            in_dfbp = False
            current_body_name = None
            body_indent = None
            current_model_name = None
            add_model_indent = None
            dfbp_indent = None
            out.append(line)
            i += 1
            continue

        # Within add_model, grab name: <model>
        if in_add_model and current_model_name is None:
            m = re.match(r"^\s*name:\s*([^\s#]+)\s*$", stripped)
            # Careful: name line is "    name: foo" but stripped removes indentation,
            # so match on stripped but keep "name:".
            if m:
                current_model_name = m.group(1)
                out.append(line)
                i += 1
                continue

        # Enter default_free_body_pose:
        if in_add_model and re.match(r"^\s*default_free_body_pose:\s*$", stripped):
            in_dfbp = True
            dfbp_indent = _indent_of(line)
            current_body_name = None
            body_indent = None
            out.append(line)
            i += 1
            continue

        # Exit dfbp when indentation decreases to add_model level or we hit another key at same/lower indent
        if in_dfbp:
            # If blank/comment line, just pass through
            if stripped.strip() == "" or stripped.lstrip().startswith("#"):
                out.append(line)
                i += 1
                continue

            # If indentation is <= dfbp indent, we left the dfbp block
            if indent_len(line) <= indent_len(dfbp_indent):
                in_dfbp = False
                current_body_name = None
                body_indent = None
                dfbp_indent = None
                # Don’t consume; reprocess this line in outer logic
                continue

            # Detect body key line: e.g. "      base_link:"
            # It should be one indent level under default_free_body_pose
            m_body = re.match(r"^\s*([A-Za-z0-9_:\-\.]+):\s*$", stripped)
            if m_body:
                current_body_name = m_body.group(1)
                body_indent = _indent_of(line)
                out.append(line)
                i += 1
                continue

            # If we have a body name, we want to overwrite translation/rotation under it
            if current_body_name is not None and current_model_name is not None:
                # translation line
                if re.match(r"^\s*translation:\s*\[.*\]\s*$", stripped):
                    # Compute pose
                    try:
                        model_instance = plant.GetModelInstanceByName(current_model_name)
                        body = plant.GetBodyByName(current_body_name, model_instance)
                        X_WB = plant.EvalBodyPoseInWorld(plant_context, body)
                    except Exception:
                        # If lookup fails, keep original line unchanged
                        out.append(line)
                        i += 1
                        continue

                    p = X_WB.translation()
                    out.append(_make_translation_line(_indent_of(line), p))
                    i += 1
                    continue

                # rotation line could be:
                # 1) "rotation: !AngleAxis" followed by angle_deg/axis on next lines
                # 2) "rotation: !Rpy { deg: [..] }" one-line
                # 3) "rotation: ..." other — we’ll replace the rotation *block* we recognize
                if re.match(r"^\s*rotation:\s*!AngleAxis\s*$", stripped):
                    # Compute pose
                    try:
                        model_instance = plant.GetModelInstanceByName(current_model_name)
                        body = plant.GetBodyByName(current_body_name, model_instance)
                        X_WB = plant.EvalBodyPoseInWorld(plant_context, body)
                    except Exception:
                        out.append(line)
                        i += 1
                        continue

                    aa = X_WB.rotation().ToAngleAxis()
                    angle_deg = float(aa.angle()) * 180.0 / np.pi
                    axis = aa.axis()

                    rot_indent = _indent_of(line)
                    # Emit new rotation block
                    out.extend(_make_angleaxis_block(rot_indent, angle_deg, axis))

                    # Skip old AngleAxis sublines (angle_deg / axis) if present
                    i += 1
                    while i < len(lines):
                        nxt = lines[i]
                        nxt_stripped = nxt.lstrip(" ")
                        # If next line indentation <= rotation indent, stop skipping
                        if indent_len(nxt) <= indent_len(rot_indent):
                            break
                        # Skip typical angleaxis contents; be permissive and skip all deeper-indented lines
                        i += 1
                    continue

                if re.match(r"^\s*rotation:\s*!Rpy\s*\{.*\}\s*$", stripped):
                    # Replace one-line Rpy with AngleAxis block
                    try:
                        model_instance = plant.GetModelInstanceByName(current_model_name)
                        body = plant.GetBodyByName(current_body_name, model_instance)
                        X_WB = plant.EvalBodyPoseInWorld(plant_context, body)
                    except Exception:
                        out.append(line)
                        i += 1
                        continue

                    aa = X_WB.rotation().ToAngleAxis()
                    angle_deg = float(aa.angle()) * 180.0 / math.pi
                    axis = aa.axis()

                    rot_indent = _indent_of(line)
                    out.extend(_make_angleaxis_block(rot_indent, angle_deg, axis))
                    i += 1
                    continue

        # Default: pass through
        out.append(line)
        i += 1

    return "".join(out)


def strip_trailing_model_drivers_and_plant_config(yaml_text: str) -> str:
    """
    Removes top-level `model_drivers:` and `plant_config:` blocks from the YAML text.
    Pure text-based, indentation-aware, no YAML parsing.
    """
    lines = yaml_text.splitlines(keepends=True)

    def is_top_level_key(line, key):
        return re.match(rf"^{key}:\s*$", line) is not None

    out = []
    i = 0
    n = len(lines)

    while i < n:
        line = lines[i]

        # Detect start of a removable top-level block
        if is_top_level_key(line, "model_drivers") or is_top_level_key(line, "plant_config"):
            block_indent = len(line) - len(line.lstrip(" "))
            i += 1

            # Skip all indented lines belonging to this block
            while i < n:
                next_line = lines[i]
                # Blank lines are considered part of the block
                if next_line.strip() == "":
                    i += 1
                    continue
                indent = len(next_line) - len(next_line.lstrip(" "))
                if indent <= block_indent:
                    break
                i += 1
            continue  # do not emit anything for this block

        out.append(line)
        i += 1

    return "".join(out)


def _collision_geometry_ids_by_name_substr(scene_graph, name_substr: str):
    ids = set()
    inspector = scene_graph.model_inspector()
    for gid in inspector.GetAllGeometryIds():
        if name_substr in inspector.GetName(gid):
            ids.add(gid)
    return ids


def _collision_geometry_ids_for_body_name(plant, scene_graph, body_name: str):
    """Returns collision GeometryIds for the named body."""
    body = plant.GetBodyByName(body_name)
    frame_id = plant.GetBodyFrameIdOrThrow(body.index())
    inspector = scene_graph.model_inspector()
    return set(inspector.GetGeometries(frame_id, role=Role.kProximity))


def apply_collision_filters(
    *,
    plant,
    scene_graph,
    sim_context,
):
    """
    Leaves only (robot) <-> (environment) candidate pairs.
    Excludes: env-env, robot-robot (incl self within arm/gripper), robot lift <-> floor, and optionally arm<->gripper.
    """
    # Robot geometry ids
    F = _collision_geometry_ids_by_name_substr(scene_graph, "floor_collision")
    Z = _collision_geometry_ids_for_body_name(plant, scene_graph, "iiwa_base_z_column")

    setF = GeometrySet(list(F))
    setZ = GeometrySet(list(Z))

    sg_context = scene_graph.GetMyContextFromRoot(sim_context)
    cfm = scene_graph.collision_filter_manager(sg_context)
    decl = CollisionFilterDeclaration()

    # Eliminate mobile iiwa lift joint <-> floor collisions
    decl.ExcludeBetween(setZ, setF)

    cfm.Apply(decl)


def multiply_all_collision_friction(*, plant, scene_graph, root_context, K: float) -> None:
    if K <= 0:
        raise ValueError(f"K must be > 0, got {K}")

    plant_context = plant.GetMyContextFromRoot(root_context)
    sg_context = scene_graph.GetMyContextFromRoot(root_context)
    inspector = scene_graph.model_inspector()

    plant_source_id = plant.get_source_id()

    n_seen = 0
    n_updated = 0

    for bi in range(plant.num_bodies()):
        body = plant.get_body(BodyIndex(bi))
        if body.index() == plant.world_body().index():
            continue

        # Collision geoms “as known by the plant” is the most reliable enumeration.
        try:
            geom_ids = plant.GetCollisionGeometriesForBody(body)
        except TypeError:
            geom_ids = plant.GetCollisionGeometriesForBody(body.index())

        for gid in geom_ids:
            n_seen += 1

            # Optional sanity check: ensure these are plant-owned geoms.
            if hasattr(inspector, "BelongsToSource") and not inspector.BelongsToSource(gid, plant_source_id):
                continue

            curr_props = inspector.GetProximityProperties(gid)
            if curr_props is None:
                continue

            # Copy the properties, then replace just friction.
            new_props = copy.deepcopy(curr_props)
            old_fric = new_props.GetProperty("material", "coulomb_friction")
            new_fric = CoulombFriction(
                float(old_fric.static_friction()) * K,
                float(old_fric.dynamic_friction()) * K,
            )
            new_props.UpdateProperty("material", "coulomb_friction", new_fric)

            # Replace proximity role properties IN PLACE (no RemoveRole needed).
            scene_graph.AssignRole(
                sg_context, plant_source_id, gid, new_props, RoleAssign.kReplace
            )
            n_updated += 1

    logger.info(
        "Plant collision geoms seen: %d; scaled friction by K=%.3g on %d proximity geometries.",
        n_seen, K, n_updated
    )


def audit_friction_runtime(plant, scene_graph, root_context, *, body_name_substr: str, max_print=20):
    plant_context = plant.GetMyContextFromRoot(root_context)
    sg_context = scene_graph.GetMyContextFromRoot(root_context)

    query_object = scene_graph.get_query_output_port().Eval(sg_context)
    inspector = query_object.inspector()

    printed = 0
    for body_index in range(plant.num_bodies()):
        body = plant.get_body(BodyIndex(body_index))
        if body_name_substr not in body.name():
            continue

        try:
            gids = plant.GetCollisionGeometriesForBody(body)
        except TypeError:
            gids = plant.GetCollisionGeometriesForBody(body.index())

        for gid in gids:
            props = inspector.GetProximityProperties(gid)
            if props is None or not props.HasProperty("material", "coulomb_friction"):
                continue
            fric = props.GetProperty("material", "coulomb_friction")
            print(f"{body.name():30s} gid={gid}  mu_s={fric.static_friction():.4g}  mu_d={fric.dynamic_friction():.4g}")
            printed += 1
            if printed >= max_print:
                return


def main():
    parser = argparse.ArgumentParser(
        description="Build a manipulation.station HardwareStation and play a retimed plan"
    )
    parser.add_argument("scenario_yaml", type=str, help="Path to directives/scenario .yaml")
    parser.add_argument("robot_plan_json", type=str, help="Path to robot_plan.json (waypoints)")
    parser.add_argument(
        "--package-xml",
        type=str,
        action="append",
        default=[],
        help="Path to a package.xml file (may be repeated).",
    )
    parser.add_argument(
        "--record-html",
        type=str,
        default="simulation.html",
        help="Write a static Meshcat recording to this HTML file (default: simulation.html).",
    )
    parser.add_argument(
        "--ee-vel",
        type=float,
        default=None,
        help="End-effector spatial velocity component limit (applied to all 6 components) in world frame. "
             "Units: rad/s for rotational, m/s for translational. Default: None (no limit).",
    )
    parser.add_argument(
        "--ee-accel",
        type=float,
        default=None,
        help="End-effector spatial acceleration component limit (applied to all 6 components) in world frame. "
             "Units: rad/s^2 for rotational, m/s^2 for translational. Default: None (no limit).",
    )
    parser.add_argument(
        "--write-updated-scenario",
        type=str,
        default=None,
        help="If set, write a new scenario YAML where default_free_body_pose entries are updated "
             "to the final simulated poses.",
    )
    parser.add_argument(
        "--friction-mult",
        type=float,
        default=5.0,
        help="Multiply Coulomb friction (mu_static, mu_dynamic) for all proximity geometries by this factor.",
    )

    args = parser.parse_args()

    scenario_path = Path(args.scenario_yaml)
    plan_path = Path(args.robot_plan_json)
    if not scenario_path.exists():
        raise FileNotFoundError(scenario_path)
    if not plan_path.exists():
        raise FileNotFoundError(plan_path)

    # Load + patch scenario YAML.
    yaml_text = _ensure_model_drivers(scenario_path.read_text())

    # Meshcat
    meshcat = StartMeshcat()

    # Parse scenario
    scenario = LoadScenario(data=yaml_text)

    # ----------------------------
    # Build a robot-only plant for TOPPRA
    # ----------------------------
    # We only include the robot model instance in this plant so TOPPRA sees
    # exactly the robot DOFs (no scene objects).
    toppra_plant = MakeMultibodyPlant(
        scenario,
        model_instance_names=["mobile_iiwa", "wsg_50"],
        add_frozen_child_instances=True,  # keep anything welded to the robot as frozen if needed
        package_xmls=[str(Path(p)) for p in args.package_xml],
    )

    # ----------------------------
    # Load plan and split robot vs gripper
    # ----------------------------
    q_all, nq, segments = _load_plan(plan_path)

    # Your plan is 13D: [base/arm ... , left_finger, right_finger]
    # We'll assume the last 2 entries are WSG joints; retime the rest.
    if nq < 3:
        raise ValueError("Expected nq >= 3.")
    nq_gripper = 2
    nq_robot = nq - nq_gripper
    q_robot = q_all[:, :nq_robot]
    q_wsg = q_all[:, nq_robot:]  # shape (N,2)

    # Safety check: robot-only plant dimension must match nq_robot.
    # Use num_positions across *all* instances in that plant (it should basically be just the robot).
    if toppra_plant.num_positions() != nq:
        raise ValueError(
            f"Plan nq={nq}, but TOPPRA plant has num_positions={toppra_plant.num_positions()}."
        )

    # Rough path traj (unit-time per segment)
    rough_traj = _pl_path_to_traj(q_all)

    # TOPPRA retime robot traj
    timed_robot_traj = _retime_toppra(
        toppra_plant,
        rough_traj,
        ee_vel_limit=args.ee_vel,
        ee_accel_limit=args.ee_accel,
    )

    # ----------------------------
    # Build station and wire trajectory sources
    # ----------------------------
    station = MakeHardwareStation(
        scenario,
        meshcat=meshcat,
        package_xmls=[str(Path(p)) for p in args.package_xml],
        hardware=False,
    )

    builder = DiagramBuilder()
    station_sys = builder.AddSystem(station)

    # Suppose timed_q_traj is your TOPPRA-retimed POSITION trajectory of size nq_total (=13).
    # If you currently have timed_robot_traj as 13D, rename it for clarity:
    timed_q_traj = timed_robot_traj
    timed_v_traj = timed_q_traj.MakeDerivative(1)   # size 13

    nq_total = timed_q_traj.rows()   # should be 13

    q_source = builder.AddSystem(TrajectorySource(timed_q_traj))
    v_source = builder.AddSystem(TrajectorySource(timed_q_traj.MakeDerivative(1)))

    # Demux full q and v into scalars
    q_demux_all = builder.AddSystem(Demultiplexer([1] * nq_total))
    v_demux_all = builder.AddSystem(Demultiplexer([1] * nq_total))
    builder.Connect(q_source.get_output_port(), q_demux_all.get_input_port())
    builder.Connect(v_source.get_output_port(), v_demux_all.get_input_port())

    # Build robot q,v by muxing outputs 0..(nq_robot-1)
    nq_wsg = 2
    nq_robot = nq_total - nq_wsg

    q_robot_mux = builder.AddSystem(Multiplexer([1] * nq_robot))
    v_robot_mux = builder.AddSystem(Multiplexer([1] * nq_robot))
    for i in range(nq_robot):
        builder.Connect(q_demux_all.get_output_port(i), q_robot_mux.get_input_port(i))
        builder.Connect(v_demux_all.get_output_port(i), v_robot_mux.get_input_port(i))

    # desired_state = [q_robot; v_robot]
    state_mux = builder.AddSystem(Multiplexer([nq_robot, nq_robot]))
    builder.Connect(q_robot_mux.get_output_port(), state_mux.get_input_port(0))
    builder.Connect(v_robot_mux.get_output_port(), state_mux.get_input_port(1))
    builder.Connect(state_mux.get_output_port(),
                    station_sys.GetInputPort("mobile_iiwa.desired_state"))

    neg = builder.AddSystem(Gain(-1.0, 1))     # multiply by -1
    wsg_diff = builder.AddSystem(Adder(2, 1))

    builder.Connect(q_demux_all.get_output_port(12),
                    wsg_diff.get_input_port(0))   # +q12
    builder.Connect(q_demux_all.get_output_port(11),
                    neg.get_input_port())          # q11
    builder.Connect(neg.get_output_port(),
                    wsg_diff.get_input_port(1))   # -q11

    builder.Connect(wsg_diff.get_output_port(),
                    station_sys.GetInputPort("wsg_50.position"))

    iiwa_body_names = ["mobile_iiwa::" + name for name in body_names_for_instance(station.plant(), "mobile_iiwa")]
    wsg_body_names = ["wsg_50::" + name for name in body_names_for_instance(station.plant(), "wsg_50")]
    body_names = iiwa_body_names + wsg_body_names
    locking_cfg = ItemLockingMonitorConfig(
        t_start = 5,
        unlock_near_geometry = body_names,
    )
    locking_monitor = ApplyItemLockingMonitorConfig(
        locking_cfg,
        station.plant(),
        builder,
        station.scene_graph()
    )

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    diagram.ForcedPublish(context)

    sim = Simulator(diagram)
    sim.set_monitor(locking_monitor.Monitor)

    print("=== BEFORE scaling ===")
    audit_friction_runtime(station.plant(), station.scene_graph(), sim.get_mutable_context(), body_name_substr="left_finger")

    # Scale friction before simulation starts.
    multiply_all_collision_friction(
        plant=station.plant(),
        scene_graph=station.scene_graph(),
        root_context=sim.get_mutable_context(),
        K=args.friction_mult,
    )

    print("=== AFTER scaling ===")
    audit_friction_runtime(station.plant(), station.scene_graph(), sim.get_mutable_context(), body_name_substr="left_finger")

    sim.Initialize()

    apply_collision_filters(
        plant=station.plant(),
        scene_graph=station.scene_graph(),
        sim_context=sim.get_mutable_context()
    )

    logger.info("Loaded scenario from: %s", scenario_path)
    logger.info("Loaded plan from: %s", plan_path)
    if segments:
        logger.info("Plan segments:")
        for s in segments:
            logger.info("  - %s", s)

    t0 = timed_q_traj.start_time()
    t1 = timed_q_traj.end_time()
    logger.info("Timed duration: %.3f s", t1 - t0)
    logger.info("Meshcat server running. Ctrl+C to exit.")

    meshcat.StartRecording()

    while sim.get_mutable_context().get_time() < t1:
        # Update the simulation using the monitor.
        locking_monitor.SetItemLockStates(sim.get_mutable_context())

        sim.AdvanceTo(
            sim.get_mutable_context().get_time() + station.plant().time_step()
        )

    meshcat.StopRecording()
    meshcat.PublishRecording()
    html = meshcat.StaticHtml()
    Path(args.record_html).write_text(html)
    logger.info("Wrote Meshcat recording to: %s", args.record_html)

    # --- After sim: optionally write updated scenario YAML ---
    if args.write_updated_scenario is not None:
        root_context = sim.get_mutable_context()

        plant = station.plant()
        plant_context = plant.GetMyContextFromRoot(root_context)

        updated_yaml_text = update_default_free_body_pose_text_in_place(
            yaml_text=yaml_text,
            plant=plant,
            plant_context=plant_context,
        )
        updated_yaml_text = strip_trailing_model_drivers_and_plant_config(updated_yaml_text)

        Path(args.write_updated_scenario).write_text(updated_yaml_text)
        logger.info("Wrote updated scenario YAML to: %s", args.write_updated_scenario)

if __name__ == "__main__":
    main()
