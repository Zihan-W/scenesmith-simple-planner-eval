#!/usr/bin/env python3
"""Plan saved grasp/place waypoints with RRT and export a trajectory JSON.

This noninteractive legacy IIWA pipeline is separate from the online robot
environment. It does not wait for keyboard-driven trajectory replay.
"""

import argparse
import json
import logging
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser, LoadModelDirectives, ProcessModelDirectives
from pydrake.planning import RobotDiagramBuilder
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig
from pydrake.geometry import CollisionFilterDeclaration, GeometrySet, Role
from pydrake.multibody.tree import BodyIndex

# Fix relative paths so contents of the src directory can be imported.
import sys
from pathlib import Path

# Import your RRT and shortcut implementations
from tools.iiwa.rrt import BiRRT, RRTOptions
from tools.iiwa.shortcut import shortcut

def embed_q_prefix(
    *,
    q_prefix: np.ndarray,
    plant,
    plant_context,
    prefix_size: int = 11,
) -> np.ndarray:
    """
    Embed a prefix configuration (e.g., first 11 DOFs) into a full plant
    configuration, using the plant's default positions for the rest.
    """
    q_full = plant.GetPositions(plant_context).copy()
    if q_prefix.shape[0] != prefix_size:
        raise ValueError(
            f"Expected prefix of size {prefix_size}, got {q_prefix.shape[0]}"
        )
    q_full[:prefix_size] = q_prefix
    return q_full

def _compute_knot_times_constant_speed(
    q_knots: np.ndarray, speed: float
) -> np.ndarray:
    """
    Assigns a timestamp to each knot so that moving along the polyline in q-space
    happens at approximately constant speed (units: q-units per second).

    Returns:
        t_knots: shape (K,) with t_knots[0] = 0
    """
    if q_knots.shape[0] < 2:
        return np.array([0.0])

    # Segment lengths in configuration space.
    dq = q_knots[1:] - q_knots[:-1]
    seg_len = np.linalg.norm(dq, axis=1)

    # Avoid division by zero for repeated knots.
    seg_dt = np.where(seg_len > 1e-12, seg_len / speed, 0.0)

    t_knots = np.zeros(q_knots.shape[0], dtype=float)
    t_knots[1:] = np.cumsum(seg_dt)
    return t_knots


def _sample_piecewise_linear(
    q_knots: np.ndarray, t_knots: np.ndarray, t: float
) -> np.ndarray:
    """
    Piecewise-linear interpolation of q(t) over knot times.
    Assumes t_knots is nondecreasing and same length as q_knots.
    """
    if t <= t_knots[0]:
        return q_knots[0]
    if t >= t_knots[-1]:
        return q_knots[-1]

    # Find segment i such that t in [t_i, t_{i+1})
    i = int(np.searchsorted(t_knots, t, side="right") - 1)
    t0, t1 = t_knots[i], t_knots[i + 1]
    q0, q1 = q_knots[i], q_knots[i + 1]

    if t1 <= t0 + 1e-12:
        return q1  # zero-duration segment

    alpha = (t - t0) / (t1 - t0)
    return (1.0 - alpha) * q0 + alpha * q1

def _collision_geometry_ids_for_instance(plant, instance):
    ids = set()
    for body_index in plant.GetBodyIndices(instance):
        body = plant.get_body(body_index)
        ids.update(plant.GetCollisionGeometriesForBody(body))
    return ids


def _all_collision_geometry_ids(plant):
    ids = set()
    for body_index in range(plant.num_bodies()):
        body = plant.get_body(BodyIndex(body_index))
        ids.update(plant.GetCollisionGeometriesForBody(body))
    return ids


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


def apply_robot_environment_only_filters(
    *,
    plant,
    scene_graph,
    sg_context,
    robot_instances,      # list[ModelInstanceIndex] e.g. [mobile_iiwa_instance, wsg_instance]
    ignore_arm_gripper_internal: bool = True,
):
    """
    Leaves only (robot) <-> (environment) candidate pairs.
    Excludes: env-env, robot-robot (incl self within arm/gripper), robot lift <-> floor, and optionally arm<->gripper.
    """
    # Robot geometry ids
    R = set()
    for inst in robot_instances:
        R |= _collision_geometry_ids_for_instance(plant, inst)

    ALL = _all_collision_geometry_ids(plant)
    E = set(ALL) - set(R)
    F = _collision_geometry_ids_by_name_substr(scene_graph, "floor_collision")
    Z = _collision_geometry_ids_for_body_name(plant, scene_graph, "iiwa_base_z_column")

    setR = GeometrySet(list(R))
    setE = GeometrySet(list(E))
    setF = GeometrySet(list(F))
    setZ = GeometrySet(list(Z))

    cfm = scene_graph.collision_filter_manager(sg_context)
    decl = CollisionFilterDeclaration()

    # Never care about env-env or robot-robot
    decl.ExcludeWithin(setE)
    decl.ExcludeWithin(setR)

    # If you *do* care about arm-vs-gripper internal collisions, set this False
    if ignore_arm_gripper_internal and len(robot_instances) >= 2:
        # Exclude between each pair of robot sub-instances
        # (Arm vs gripper, etc.)
        ids_by_inst = [_collision_geometry_ids_for_instance(plant, inst) for inst in robot_instances]
        for i in range(len(ids_by_inst)):
            for j in range(i + 1, len(ids_by_inst)):
                decl.ExcludeBetween(
                    GeometrySet(list(ids_by_inst[i])),
                    GeometrySet(list(ids_by_inst[j])),
                )

    # 5) Eliminate mobile iiwa lift joint <-> floor collisions
    if Z and F:
        decl.ExcludeBetween(setZ, setF)

    # Important: Do NOT exclude between setR and setE.
    # That’s the only family we want to keep.
    cfm.Apply(decl)


class RobotEnvValidityChecker:
    """
    Collision + clearance validity checker for RRT.

    Returns False if:
      1) Any robot-env penetration is found (per your collision filters), OR
      2) The between-fingers point is within `finger_clearance_m` of any
         environment proximity geometry.

    Notes on Drake API compatibility:
      - This version assumes QueryObject.ComputeSignedDistanceToPoint(p_WQ, threshold)
        exists (as in your traceback) and DOES NOT support the GeometrySet overload.
      - Environment geometries are determined once at construction time by:
          * collecting proximity geometries
          * removing geometries owned by any `robot_instances`
    """

    def __init__(
        self,
        *,
        diagram,
        plant,
        scene_graph,
        diagram_context,
        plant_context,
        robot_instances,              # list[ModelInstanceIndex]
        q_full_fixed: np.ndarray,      # full plant positions used as baseline; only first prefix_size overwritten
        prefix_size: int = 11,
        # --- clearance check params ---
        gripper_instance=None,         # ModelInstanceIndex for gripper; defaults to last of robot_instances
        gripper_body_name: str = "body",
        p_GS_G: np.ndarray = np.array([0.054 - 0.01, 0.10625, 0.0]),
        finger_clearance_m: float = 0.05,
        # --- filtering options passed to your helper ---
        ignore_arm_gripper_internal: bool = True,
    ):
        self._diagram = diagram
        self._plant = plant
        self._scene_graph = scene_graph
        self._diagram_context = diagram_context
        self._plant_context = plant_context
        self._prefix_size = int(prefix_size)

        self._robot_instances = list(robot_instances)
        if len(self._robot_instances) == 0:
            raise ValueError("robot_instances must be a non-empty list of model instances.")

        self._q_full_fixed = np.asarray(q_full_fixed, dtype=float).copy()
        if self._q_full_fixed.shape != (plant.num_positions(),):
            raise ValueError("q_full_fixed must have shape (plant.num_positions(),)")

        self._gripper_instance = gripper_instance if gripper_instance is not None else self._robot_instances[-1]
        self._gripper_body = plant.GetBodyByName(gripper_body_name, self._gripper_instance)

        self._p_GS_G = np.asarray(p_GS_G, dtype=float).reshape((3,))
        self._finger_clearance_m = float(finger_clearance_m)

        # Apply collision filters once (they live in SceneGraph context).
        sg_context = scene_graph.GetMyContextFromRoot(diagram_context)
        apply_robot_environment_only_filters(
            plant=plant,
            scene_graph=scene_graph,
            sg_context=sg_context,
            robot_instances=robot_instances,
            ignore_arm_gripper_internal=ignore_arm_gripper_internal,
        )

        # Cache environment proximity geometries for point-distance queries.
        self._env_geometry_set = self._build_environment_geometry_set()
        # Additionally cache as a Python set for fast membership tests in the distance results.
        # Some Drake versions don't expose GeometrySet.geometries(); we store ids ourselves.
        # (Set in _build_environment_geometry_set)
        if not hasattr(self, "_env_geometry_ids"):
            raise RuntimeError("Internal error: _env_geometry_ids not initialized.")

    def plant(self):
        return self._plant

    def embed_prefix(self, q_prefix: np.ndarray) -> np.ndarray:
        q_prefix = np.asarray(q_prefix, dtype=float)
        if q_prefix.shape != (self._prefix_size,):
            raise ValueError(f"Expected q_prefix shape ({self._prefix_size},), got {q_prefix.shape}")
        q_full = self._q_full_fixed.copy()
        q_full[: self._prefix_size] = q_prefix
        return q_full

    def _build_environment_geometry_set(self) -> GeometrySet:
        """
        Build a GeometrySet of all proximity geometries that do NOT belong to any
        instance in self._robot_instances. Also caches the ids in self._env_geometry_ids.
        """
        sg_context = self._scene_graph.GetMyContextFromRoot(self._diagram_context)
        query_object = self._scene_graph.get_query_output_port().Eval(sg_context)
        inspector = query_object.inspector()

        # Drake API (your build): GetGeometryIds(geometry_set, role=None) -> set[GeometryId]
        all_geoms = GeometrySet(inspector.GetAllGeometryIds())
        prox_ids = inspector.GetGeometryIds(all_geoms, Role.kProximity)

        env_ids = []
        for gid in prox_ids:
            frame_id = inspector.GetFrameId(gid)

            # Try to map geometry -> body; if it fails (anchored/non-body), treat as environment.
            try:
                body = self._plant.GetBodyFromFrameId(frame_id)
                if body.model_instance() in self._robot_instances:
                    continue
            except Exception:
                pass

            env_ids.append(gid)

        if not env_ids:
            raise RuntimeError(
                "No environment proximity geometries found. "
                "Check that your environment has proximity roles (collision geometry)."
            )

        self._env_geometry_ids = set(env_ids)
        return GeometrySet(env_ids)

    def _between_fingers_point_W(self) -> np.ndarray:
        """
        Compute p_WQ for the point between fingers:
            p_WQ = X_WG * p_GS_G
        where X_WG is the gripper body pose in world.
        """
        X_WG = self._plant.EvalBodyPoseInWorld(self._plant_context, self._gripper_body)
        p_WQ = X_WG.multiply(self._p_GS_G)  # RigidTransform * point
        return np.asarray(p_WQ, dtype=float).reshape((3,))

    def CheckConfigCollisionFreePrefix(self, q_prefix: np.ndarray) -> bool:
        q_full = self.embed_prefix(q_prefix)

        # Set positions in plant context.
        self._plant.SetPositions(self._plant_context, q_full)

        # Query collisions.
        sg_context = self._scene_graph.GetMyContextFromRoot(self._diagram_context)
        query_object = self._scene_graph.get_query_output_port().Eval(sg_context)

        penetrations = query_object.ComputePointPairPenetration()
        # With your filters applied, any penetration means robot-env collision.
        if len(penetrations) != 0:
            return False

        if self._finger_clearance_m == 0:
            return True  # If zero, then we're done.

        # Clearance check: reject if the between-fingers point is within finger_clearance_m
        # of any *environment* proximity geometry.
        p_WQ = self._between_fingers_point_W()

        # Your build supports: ComputeSignedDistanceToPoint(p_WQ, threshold)
        try:
            dists_all = query_object.ComputeSignedDistanceToPoint(p_WQ, self._finger_clearance_m)
        except:
            # Handle errors like "RuntimeError: DistanceToPoint from meshes: FeatureNormalSet: Cannot compute an edge normal because the two triangles sharing the edge make a very sharp edge."
            return False

        # Filter to environment geometries only. If any are returned, we're too close.
        for d in dists_all:
            # In Drake python this field is typically id_G (geometry id of the measured object).
            # If your build differs, print(dir(d)) once and adjust this name.
            if d.id_G in self._env_geometry_ids:
                return False

        return True

def make_prefix_sampler(
    checker,
    *,
    rng: np.random.Generator,
    world_xy_bounds: tuple[float, float, float, float],
    prefix_size: int = 11,
):
    """
    Samples q_prefix (size=prefix_size) with special handling for:
      q[0] = base x in [world_x_min, world_x_max]
      q[1] = base y in [world_y_min, world_y_max]
      q[2] = base theta in [-pi, pi]

    Remaining prefix DOFs use plant position limits. Raises if any remaining
    bounds are non-finite (so you notice immediately).
    """
    q_lb = np.asarray(checker.plant().GetPositionLowerLimits()[:prefix_size], dtype=float)
    q_ub = np.asarray(checker.plant().GetPositionUpperLimits()[:prefix_size], dtype=float)

    x_min, x_max, y_min, y_max = world_xy_bounds

    # Override unbounded base x/y/theta with task-derived / chosen bounds.
    q_lb[0], q_ub[0] = x_min, x_max
    q_lb[1], q_ub[1] = y_min, y_max

    # Safety: make sure the rest are finite.
    bad = ~np.isfinite(q_lb) | ~np.isfinite(q_ub)
    if np.any(bad):
        bad_idx = np.where(bad)[0].tolist()
        raise ValueError(
            f"Non-finite joint bounds in prefix after overrides at indices {bad_idx}. "
            "You need to provide finite bounds for these DOFs too."
        )

    def sample():
        return rng.uniform(q_lb, q_ub)

    return sample

# -----------------------------------------------------------------------------
# Utilities copied / aligned with your previous script
# -----------------------------------------------------------------------------

def register_package_xml(parser: Parser, package_xml_path: Path):
    """
    Register a ROS-style package.xml with Drake's PackageMap.
    """
    if not package_xml_path.exists():
        raise ValueError(f"package.xml does not exist: {package_xml_path}")

    tree = ET.parse(package_xml_path)
    root = tree.getroot()

    name_elem = root.find("name")
    if name_elem is None or not name_elem.text:
        raise ValueError(f"Could not find <name> tag in {package_xml_path}")

    package_name = name_elem.text.strip()
    package_dir = str(package_xml_path.parent)

    parser.package_map().Add(package_name, package_dir)
    logger.info("Registered package '%s' at %s", package_name, package_dir)

def load_task_json(task_file: Path) -> dict:
    if not task_file.exists():
        raise ValueError(f"Task file does not exist: {task_file}")
    with open(task_file, "r") as f:
        return json.load(f)

# -----------------------------------------------------------------------------
# Waypoints / plan data structures
# -----------------------------------------------------------------------------

@dataclass
class Waypoint:
    name: str
    q: np.ndarray  # (nq,)

@dataclass
class TrajectorySegment:
    name: str
    q_knots: np.ndarray  # shape (K, nq)
    # You can later add timestamps, costs, etc.


# -----------------------------------------------------------------------------
# Step (1): Load world + set up simulation/diagram/visualization  (DONE)
# -----------------------------------------------------------------------------

def build_world_diagram(
    dmd_file: Path,
    package_xmls: List[Path],
    *,
    add_visualization: bool = True,
):
    """
    Builds the RobotDiagram from a .dmd.yaml file, registering any user package.xml.

    Returns:
        meshcat, builder, diagram, diagram_context, plant, plant_context, scene_graph
    """
    if not dmd_file.exists():
        raise ValueError(f"DMD file does not exist: {dmd_file}")

    meshcat = StartMeshcat()
    meshcat.Delete()

    builder = RobotDiagramBuilder()
    parser_drake = builder.parser()

    # Register packages (same approach as your previous script).
    for pkg_xml in package_xmls:
        register_package_xml(parser_drake, pkg_xml)

    # Load DMD directives into the builder's parser.
    directives = LoadModelDirectives(str(dmd_file))
    ProcessModelDirectives(directives, parser_drake)

    plant = builder.plant()
    scene_graph = builder.scene_graph()

    # If you need to add extra models for planning/visualization, do it here,
    # before Finalize(). (Example: a "ghost" robot, gripper, etc.)

    plant.Finalize()

    if add_visualization:
        ApplyVisualizationConfig(
            config=VisualizationConfig(),
            plant=plant,
            scene_graph=scene_graph,
            builder=builder.builder(),
            meshcat=meshcat,
        )

    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetSubsystemContext(plant, diagram_context)

    # Make sure the world is drawn once.
    diagram.ForcedPublish(diagram_context)

    return meshcat, builder, diagram, diagram_context, plant, plant_context, scene_graph


# -----------------------------------------------------------------------------
# Step (2): Load waypoints  (TODO)
# -----------------------------------------------------------------------------

def load_waypoints_json(path: Path) -> List[Waypoint]:
    """
    Expected format (matches what your grasp/place script wrote):
    {
      "waypoints": [
        {"name": "start", "q": [...]},
        {"name": "grasp", "q": [...]},
        ...
      ]
    }
    """
    if not path.exists():
        raise ValueError(f"Waypoints file does not exist: {path}")

    with open(path, "r") as f:
        data = json.load(f)

    if "waypoints" not in data or not isinstance(data["waypoints"], list):
        raise ValueError(f"Invalid waypoints format in {path}")

    waypoints: List[Waypoint] = []
    for wp in data["waypoints"]:
        name = wp.get("name", "unnamed")
        q_list = wp.get("q", None)
        if q_list is None:
            raise ValueError(f"Waypoint {name} missing 'q'")
        q = np.asarray(q_list, dtype=float)
        waypoints.append(Waypoint(name=name, q=q))
    return waypoints


# -----------------------------------------------------------------------------
# Step (3): Plan each segment with RRT  (TODO skeleton)
# -----------------------------------------------------------------------------

def plan_rrt_segment(
    *,
    checker,
    q_start,
    q_goal,
    rng,
    world_xy_bounds,
    max_verts=2000,
    max_iters=10000,
    step_size=0.1,
    do_shortcut: bool = False,
    shortcut_tries: int = 200,
    shortcut_check_size: float = 1e-2,
    add_gripper_clearance: bool = False
) -> TrajectorySegment:
    """
    Plans in the first 11 positions only. Remaining positions held constant.
    Returns full-q knots for visualization.
    """
    start11 = np.asarray(q_start.q[:11], dtype=float).copy()
    goal11  = np.asarray(q_goal.q[:11], dtype=float).copy()

    if add_gripper_clearance:
        checker._finger_clearance_m = 0.05
        while not checker.CheckConfigCollisionFreePrefix(start11) or not checker.CheckConfigCollisionFreePrefix(goal11):
            checker._finger_clearance_m -= 0.01
        if checker._finger_clearance_m < 0.0:
            checker._finger_clearance_m = 0.0
    else:
        checker._finger_clearance_m = 0.0

    # (Optional but helpful) ensure endpoints are valid
    if not checker.CheckConfigCollisionFreePrefix(start11):
        raise RuntimeError("Start waypoint is in collision (robot-environment).")
    if not checker.CheckConfigCollisionFreePrefix(goal11):
        raise RuntimeError("Goal waypoint is in collision (robot-environment).")

    rrt_options = RRTOptions(
        step_size=step_size,
        check_size=min(1e-2, step_size / 10.0),
        max_vertices=int(max_verts),
        max_iters=int(max_iters),
        goal_sample_frequency=0.01,
        always_swap=False,
    )

    RandomConfig = make_prefix_sampler(
        checker,
        rng=rng,
        world_xy_bounds=world_xy_bounds,
        prefix_size=11,
    )

    ValidityChecker = lambda q11: checker.CheckConfigCollisionFreePrefix(q11)

    rrt_planner = BiRRT(RandomConfig, ValidityChecker)
    path11 = rrt_planner.plan(start11, goal11, rrt_options)
    if path11 is None or len(path11) == 0:
        raise RuntimeError("RRT failed to find a path.")

    # --- Optional shortcutting (in 11-DOF space) ---
    if do_shortcut:
        path11 = shortcut_refine_prefix(
            checker,
            path11,
            num_tries=shortcut_tries,
            check_size=shortcut_check_size,
        )

    return TrajectorySegment(name=(q_start.name + " -> " + q_goal.name), q_knots=np.array(path11))

def shortcut_refine_prefix(
    checker,
    path11,
    *,
    num_tries: int = 200,
    check_size: float = 1e-2,
):
    ValidityChecker = lambda q11: checker.CheckConfigCollisionFreePrefix(q11)
    return shortcut(path11, ValidityChecker, num_tries=num_tries, check_size=check_size)

def promote_segment_to_13dof(
    seg: TrajectorySegment,
    *,
    gripper_left: float,
    gripper_right: float,
) -> TrajectorySegment:
    """Appends constant gripper joints to every knot in seg.q_knots."""
    q = seg.q_knots
    logger.debug("Segment shape: %s", q.shape)
    if q.shape[1] != 11:
        raise ValueError(f"Expected 11DoF segment, got {q.shape[1]}DoF")

    gr = np.tile(np.array([[gripper_left, gripper_right]], dtype=float), (q.shape[0], 1))
    q13 = np.hstack([q, gr])
    return TrajectorySegment(name=seg.name, q_knots=q13)

def make_gripper_segment(
    *,
    name: str,
    q_robot_11: np.ndarray,
    gripper_start: tuple[float, float],
    gripper_goal: tuple[float, float],
    num_knots: int = 2,
) -> TrajectorySegment:
    """
    Robot holds still at q_robot_11 while gripper linearly moves start->goal.
    Returns 13DoF knots.
    """
    q_robot_11 = np.asarray(q_robot_11, dtype=float).reshape(-1)
    if q_robot_11.shape[0] != 11:
        raise ValueError(f"Expected robot prefix size 11, got {q_robot_11.shape[0]}")

    l0, r0 = gripper_start
    l1, r1 = gripper_goal

    alphas = np.linspace(0.0, 1.0, num_knots)
    q13 = np.zeros((num_knots, 13), dtype=float)
    q13[:, :11] = q_robot_11[None, :]
    q13[:, 11] = (1.0 - alphas) * l0 + alphas * l1
    q13[:, 12] = (1.0 - alphas) * r0 + alphas * r1
    return TrajectorySegment(name=name, q_knots=q13)


# -----------------------------------------------------------------------------
# Step (4): Concatenate segments  (TODO skeleton)
# -----------------------------------------------------------------------------

def concatenate_segments(segments: List[TrajectorySegment]) -> np.ndarray:
    """
    Concatenate segments into one (T, nq) array.
    Avoid duplicating the first knot of each subsequent segment.
    """
    if not segments:
        raise ValueError("No segments to concatenate")

    pieces = [segments[0].q_knots]
    for seg in segments[1:]:
        pieces.append(seg.q_knots[1:, :])
    return np.vstack(pieces)


# -----------------------------------------------------------------------------
# Step (5): Visualize plan interactively  (TODO skeleton)
# -----------------------------------------------------------------------------

def dump_piecewise_linear_trajectory_json(
    *,
    path: Path,
    segments: List[TrajectorySegment],
):
    """
    Writes a single piecewise-linear joint trajectory as JSON.
    Concatenates all segment knots into one (N,13) array, avoiding duplicate
    boundary knots between segments.
    """
    if not segments:
        raise ValueError("No segments to dump.")

    # Concatenate knots; avoid duplicating first knot of each subsequent segment.
    pieces = [segments[0].q_knots]
    for seg in segments[1:]:
        if seg.q_knots.shape[0] == 0:
            continue
        pieces.append(seg.q_knots[1:, :])

    q_traj = np.vstack(pieces)

    if q_traj.ndim != 2 or q_traj.shape[1] != 13:
        raise ValueError(f"Expected trajectory shape (N,13); got {q_traj.shape}")

    out = {
        "q": q_traj.tolist(),      # (N,13)
        "nq": 13,
        "num_knots": int(q_traj.shape[0]),
        "segments": [seg.name for seg in segments],  # helpful provenance
    }

    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(out, f, indent=2)

    logger.info("Wrote piecewise-linear trajectory: %s to %s", q_traj.shape, path)

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Plan a path through saved robot waypoints using RRT and visualize it."
    )
    parser.add_argument("task_file", type=str, help="Path to task JSON (contains world_bounds)")
    parser.add_argument("dmd_file", type=str, help="Path to .dmd.yaml directives file")
    parser.add_argument(
        "waypoints_file",
        type=str,
        help="Path to robot_waypoints.json (produced by grasp/place script)",
    )
    parser.add_argument(
        "--package-xml",
        type=str,
        action="append",
        default=[],
        help="Path(s) to ROS-style package.xml to register with Drake",
    )
    parser.add_argument("--rrt-verts", type=int, default=10000)
    parser.add_argument("--rrt-iters", type=int, default=100000)
    parser.add_argument("--rrt-step", type=float, default=0.1)
    parser.add_argument("--render-rate", type=float, default=60.0)
    parser.add_argument("--q-speed", type=float, default=1.0)
    parser.add_argument(
        "--no-shortcut",
        action="store_true",
        help="Disable shortcutting (default: shortcutting is enabled).",
    )
    parser.add_argument("--shortcut-tries", type=int, default=25)
    parser.add_argument("--shortcut-check", type=float, default=1e-2)
    parser.add_argument("--gripper-clearance", action="store_true")
    parser.add_argument(
        "--out-traj",
        type=str,
        default="robot_plan.json",
        help="Output JSON path for the concatenated Nx13 piecewise-linear trajectory.",
    )

    args = parser.parse_args()

    dmd_file = Path(args.dmd_file)
    waypoints_file = Path(args.waypoints_file)
    package_xmls = [Path(p) for p in args.package_xml]

    task = load_task_json(Path(args.task_file))
    wb_min = task["world_bounds"]["min"]
    wb_max = task["world_bounds"]["max"]
    world_xy_bounds = (float(wb_min[0]), float(wb_max[0]), float(wb_min[1]), float(wb_max[1]))

    # ---------------------------------------------------------------------
    # (1) Load world + set up simulation/diagram
    # ---------------------------------------------------------------------
    meshcat, builder, diagram, diagram_context, plant, plant_context, scene_graph = (
        build_world_diagram(dmd_file=dmd_file, package_xmls=package_xmls)
    )

    # ---------------------------------------------------------------------
    # Build a robot-vs-environment validity checker for RRT
    # ---------------------------------------------------------------------
    # TODO: replace these names with the ones in your directives if different.
    mobile_iiwa_instance = plant.GetModelInstanceByName("mobile_iiwa")
    wsg_instance = plant.GetModelInstanceByName("wsg_50")

    q_full_fixed = plant.GetPositions(plant_context).copy()

    checker = RobotEnvValidityChecker(
        diagram=diagram,
        plant=plant,
        scene_graph=scene_graph,
        diagram_context=diagram_context,
        plant_context=plant_context,
        robot_instances=[mobile_iiwa_instance, wsg_instance],
        q_full_fixed=q_full_fixed,
        prefix_size=11,
    )

    # ---------------------------------------------------------------------
    # (2) Load waypoints
    # ---------------------------------------------------------------------
    waypoints = load_waypoints_json(waypoints_file)
    if len(waypoints) < 2:
        raise ValueError("Need at least 2 waypoints to plan.")

    # Embed waypoint prefixes into full plant configurations
    embedded_waypoints: List[Waypoint] = []

    for wp in waypoints:
        q_full = embed_q_prefix(
            q_prefix=wp.q,
            plant=plant,
            plant_context=plant_context,
            prefix_size=11,
        )
        embedded_waypoints.append(Waypoint(name=wp.name, q=q_full))

    waypoints = embedded_waypoints

    logger.info("Loaded %d waypoints from %s", len(waypoints), waypoints_file)
    for i, wp in enumerate(waypoints):
        logger.info("  %d: %s", i, wp.name)

    # ---------------------------------------------------------------------
    # (3) For each consecutive waypoint pair, plan with RRT
    # ---------------------------------------------------------------------
    # Gripper joint targets
    GRIPPER_OPEN = (-0.05, 0.05)    # (left, right)
    GRIPPER_CLOSED = (-0.005, 0.005)    # (left, right)

    rng = np.random.default_rng(0)

    segments: List[TrajectorySegment] = []

    # We will keep track of the gripper state as we build segments.
    current_gripper = GRIPPER_OPEN

    for a, b in zip(waypoints[:-1], waypoints[1:]):
        logger.info("Planning segment: %s -> %s", a.name, b.name)

        add_gripper_clearance = False
        if a.name == "postgrasp" and args.gripper_clearance:
            logger.info("Using extra gripper clearance in this plan.")
            add_gripper_clearance = True

        seg11 = plan_rrt_segment(
            checker=checker,
            q_start=a,
            q_goal=b,
            rng=rng,
            world_xy_bounds=world_xy_bounds,
            max_verts=args.rrt_verts,
            max_iters=args.rrt_iters,
            step_size=args.rrt_step,
            do_shortcut=not args.no_shortcut,
            shortcut_tries=args.shortcut_tries,
            shortcut_check_size=args.shortcut_check,
            add_gripper_clearance=add_gripper_clearance
        )

        # Name the motion segment and promote it to 13DoF with the CURRENT gripper setting.
        seg11.name = f"{a.name} -> {b.name}"
        seg13 = promote_segment_to_13dof(
            seg11,
            gripper_left=current_gripper[0],
            gripper_right=current_gripper[1],
        )

        logger.info("  Segment knots: %d", seg13.q_knots.shape[0])
        segments.append(seg13)

        # Insert gripper-only segments at grasp and place (after arriving there).
        # We key off the destination waypoint name b.name.
        if b.name.lower() == "grasp":
            # Close while holding robot still at grasp pose.
            q_robot_11_at_grasp = np.asarray(b.q[:11], dtype=float)
            segments.append(
                make_gripper_segment(
                    name="gripper close",
                    q_robot_11=q_robot_11_at_grasp,
                    gripper_start=current_gripper,
                    gripper_goal=GRIPPER_CLOSED,
                    num_knots=2,
                )
            )
            current_gripper = GRIPPER_CLOSED

        if b.name.lower() == "place":
            # Open while holding robot still at place pose.
            q_robot_11_at_place = np.asarray(b.q[:11], dtype=float)
            segments.append(
                make_gripper_segment(
                    name="gripper open",
                    q_robot_11=q_robot_11_at_place,
                    gripper_start=current_gripper,
                    gripper_goal=GRIPPER_OPEN,
                    num_knots=2,
                )
            )
            current_gripper = GRIPPER_OPEN

    dump_piecewise_linear_trajectory_json(
        path=Path(args.out_traj),
        segments=segments,
    )

if __name__ == "__main__":
    main()
