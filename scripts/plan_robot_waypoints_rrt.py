#!/usr/bin/env python3
"""
plan_robot_waypoints_rrt.py

Planning script skeleton:
1) Load the world and set up the simulation (DONE)
2) Load waypoints from task/waypoints file (TODO)
3) For each consecutive waypoint pair, plan a path using RRT (TODO)
4) Concatenate the paths (TODO)
5) Visualize the full plan in a loop: <enter> replays, 'q'+<enter> quits (TODO)
"""

import argparse
import json
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np

from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser, LoadModelDirectives, ProcessModelDirectives
from pydrake.planning import RobotDiagramBuilder
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig
from pydrake.geometry import CollisionFilterDeclaration, GeometrySet
from pydrake.multibody.tree import BodyIndex

# Fix relative paths so contents of the src directory can be imported.
import sys
sys.path.append("..")

# Import your RRT implementation (adjust the import path to your repo layout)
from src.rrt import BiRRT, RRTOptions

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
    Excludes: env-env, robot-robot (incl self within arm/gripper), and optionally arm<->gripper.
    """
    # Robot geometry ids
    R = set()
    for inst in robot_instances:
        R |= _collision_geometry_ids_for_instance(plant, inst)

    ALL = _all_collision_geometry_ids(plant)
    E = set(ALL) - set(R)

    setR = GeometrySet(list(R))
    setE = GeometrySet(list(E))

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

    # Important: Do NOT exclude between setR and setE.
    # That’s the only family we want to keep.
    cfm.Apply(decl)

class RobotEnvValidityChecker:
    def __init__(
        self,
        *,
        diagram,
        plant,
        scene_graph,
        diagram_context,
        plant_context,
        robot_instances,      # [mobile_iiwa_instance, wsg_instance]
        q_full_fixed: np.ndarray,  # full plant positions used as baseline; only first 11 overwritten
        prefix_size: int = 11,
    ):
        self._diagram = diagram
        self._plant = plant
        self._scene_graph = scene_graph
        self._diagram_context = diagram_context
        self._plant_context = plant_context
        self._prefix_size = prefix_size

        self._q_full_fixed = np.asarray(q_full_fixed, dtype=float).copy()
        if self._q_full_fixed.shape != (plant.num_positions(),):
            raise ValueError("q_full_fixed must have shape (plant.num_positions(),)")

        # Apply collision filters once (they live in SceneGraph context).
        sg_context = scene_graph.GetMyContextFromRoot(diagram_context)
        apply_robot_environment_only_filters(
            plant=plant,
            scene_graph=scene_graph,
            sg_context=sg_context,
            robot_instances=robot_instances,
            ignore_arm_gripper_internal=True,
        )

    def plant(self):
        return self._plant

    def embed_prefix(self, q_prefix: np.ndarray) -> np.ndarray:
        q_prefix = np.asarray(q_prefix, dtype=float)
        if q_prefix.shape != (self._prefix_size,):
            raise ValueError(f"Expected q_prefix shape ({self._prefix_size},), got {q_prefix.shape}")
        q_full = self._q_full_fixed.copy()
        q_full[: self._prefix_size] = q_prefix
        return q_full

    def CheckConfigCollisionFreePrefix(self, q_prefix: np.ndarray) -> bool:
        q_full = self.embed_prefix(q_prefix)

        # Set positions in plant context
        self._plant.SetPositions(self._plant_context, q_full)

        # Query collisions
        sg_context = self._scene_graph.GetMyContextFromRoot(self._diagram_context)
        query_object = self._scene_graph.get_query_output_port().Eval(sg_context)

        penetrations = query_object.ComputePointPairPenetration()
        # With filters applied, any penetration means robot-env collision.
        return len(penetrations) == 0

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
    print(f"Registered package '{package_name}' at {package_dir}")

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
    max_iters=2000,
    step_size=0.1,
) -> TrajectorySegment:
    """
    Plans in the first 11 positions only. Remaining positions held constant.
    Returns full-q knots for visualization.
    """
    start11 = np.asarray(q_start[:11], dtype=float).copy()
    goal11  = np.asarray(q_goal[:11], dtype=float).copy()

    # (Optional but helpful) ensure endpoints are valid
    if not checker.CheckConfigCollisionFreePrefix(start11):
        raise RuntimeError("Start waypoint is in collision (robot-environment).")
    if not checker.CheckConfigCollisionFreePrefix(goal11):
        raise RuntimeError("Goal waypoint is in collision (robot-environment).")

    rrt_options = RRTOptions(
        step_size=step_size,
        check_size=min(1e-2, step_size / 10.0),
        max_vertices=int(1e4),
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

    # Convert 11-DOF path back to full-q knots for playback/concatenation.
    q_knots_full = np.vstack([checker.embed_prefix(q11) for q11 in path11])
    return TrajectorySegment(q_knots=q_knots_full)



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

def playback_plan_interactive(
    *,
    diagram,
    diagram_context,
    plant,
    plant_context,
    q_traj: np.ndarray,
    render_rate_hz: float = 60.0,
    q_speed: float = 0.6,
):
    """
    Visualize the plan with roughly constant speed in configuration space.

    - Builds a piecewise-linear trajectory through q_traj
    - Assigns knot times proportional to ||dq|| so motion is ~constant speed
    - Renders at a fixed rate and interpolates between knots

    Controls:
      - <enter> : replay trajectory
      - q<enter>: quit
    """
    if q_traj.shape[0] < 1:
        raise ValueError("q_traj is empty")

    dt_render = 1.0 / render_rate_hz
    t_knots = _compute_knot_times_constant_speed(q_traj, speed=q_speed)
    T = float(t_knots[-1])

    print("\nControls:")
    print("  <enter> : replay trajectory")
    print("  q + <enter> : quit")
    print()
    print(f"Playback: {q_traj.shape[0]} knots, duration ~ {T:.2f}s, "
          f"render_rate={render_rate_hz:.1f} Hz, q_speed={q_speed:.3f}")

    def play_once():
        # Always start exactly at first knot
        plant.SetPositions(plant_context, q_traj[0])
        diagram.ForcedPublish(diagram_context)

        t = 0.0
        # Use wall-clock sleep to regulate rendering.
        while t < T:
            q = _sample_piecewise_linear(q_traj, t_knots, t)
            plant.SetPositions(plant_context, q)
            diagram.ForcedPublish(diagram_context)
            time.sleep(dt_render)
            t += dt_render

        # End exactly at last knot
        plant.SetPositions(plant_context, q_traj[-1])
        diagram.ForcedPublish(diagram_context)

    play_once()

    while True:
        s = input().strip("\n").lower()
        if s == "q":
            break
        if s == "":
            play_once()

    print("Exiting playback.")


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
    parser.add_argument("--rrt-iters", type=int, default=2000)
    parser.add_argument("--rrt-step", type=float, default=0.1)
    parser.add_argument("--render-rate", type=float, default=60.0)
    parser.add_argument("--q-speed", type=float, default=1.0)
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

    print(f"Loaded {len(waypoints)} waypoints from {waypoints_file}")
    for i, wp in enumerate(waypoints):
        print(f"  {i}: {wp.name}")

    # ---------------------------------------------------------------------
    # (3) For each consecutive waypoint pair, plan with RRT
    # ---------------------------------------------------------------------
    rng = np.random.default_rng(0)
    segments: List[TrajectorySegment] = []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        print(f"\nPlanning segment: {a.name} -> {b.name}")
        seg = plan_rrt_segment(
            checker=checker,
            q_start=a.q,
            q_goal=b.q,
            rng=rng,
            world_xy_bounds=world_xy_bounds,
            max_iters=args.rrt_iters,
            step_size=args.rrt_step,
        )

        print(f"  Segment knots: {seg.q_knots.shape[0]}")
        segments.append(seg)

    # ---------------------------------------------------------------------
    # (4) Concatenate
    # ---------------------------------------------------------------------
    q_traj = concatenate_segments(segments)
    print(f"\nFull plan knots: {q_traj.shape[0]} (nq={q_traj.shape[1]})")

    # ---------------------------------------------------------------------
    # (5) Visualize interactively
    # ---------------------------------------------------------------------
    playback_plan_interactive(
        diagram=diagram,
        diagram_context=diagram_context,
        plant=plant,
        plant_context=plant_context,
        q_traj=q_traj,
        render_rate_hz=args.render_rate,
        q_speed=args.q_speed,
    )


if __name__ == "__main__":
    main()
