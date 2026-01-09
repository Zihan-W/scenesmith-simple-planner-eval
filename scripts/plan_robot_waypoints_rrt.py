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
    plant,
    plant_context,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    rng: np.random.Generator,
    max_iters: int = 2000,
    step_size: float = 0.1,
) -> TrajectorySegment:
    """
    Placeholder for an RRT planner in configuration space.

    Contract:
      - Returns a sequence of configurations from q_start to q_goal.
      - Should do collision checking using the plant/scene_graph context you already built.

    TODO: Implement:
      - sampling in joint limits
      - nearest neighbor
      - steer step_size
      - collision check along edge
      - goal connection
      - path extraction
    """
    # --- TEMP: straight-line fallback so the script structure works ---
    # Replace this with real RRT output once implemented.
    K = 20
    qs = np.linspace(q_start, q_goal, K)
    return TrajectorySegment(q_knots=qs)


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
    parser.add_argument("--playback-dt", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    dmd_file = Path(args.dmd_file)
    waypoints_file = Path(args.waypoints_file)
    package_xmls = [Path(p) for p in args.package_xml]

    # ---------------------------------------------------------------------
    # (1) Load world + set up simulation/diagram
    # ---------------------------------------------------------------------
    meshcat, builder, diagram, diagram_context, plant, plant_context, scene_graph = (
        build_world_diagram(dmd_file=dmd_file, package_xmls=package_xmls)
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
    rng = np.random.default_rng(args.seed)
    segments: List[TrajectorySegment] = []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        print(f"\nPlanning segment: {a.name} -> {b.name}")
        seg = plan_rrt_segment(
            plant=plant,
            plant_context=plant_context,
            q_start=a.q,
            q_goal=b.q,
            rng=rng,
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
        render_rate_hz=60,
        q_speed=1.0,
    )


if __name__ == "__main__":
    main()
