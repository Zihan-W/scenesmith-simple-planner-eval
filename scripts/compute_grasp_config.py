#!/usr/bin/env python3
import json
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as ET

from pydrake.all import (
    Parser,
    MultibodyPlant,
    SceneGraph,
    StartMeshcat,
    LoadModelDirectives,
    ProcessModelDirectives,
    Mesh,
    PointCloud,
    Rgba
)


def load_task(task_file: Path):
    """Load a JSON task file."""
    if not task_file.exists():
        raise FileNotFoundError(f"Task file does not exist: {task_file}")
    with open(task_file) as f:
        return json.load(f)


def register_package_xml(parser: Parser, package_xml_path: Path):
    """
    Register a ROS-style package.xml with Drake's PackageMap.

    Args:
        parser: Drake Parser instance
        package_xml_path: Path to package.xml
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

def sample_points_from_body(
    plant: MultibodyPlant,
    scene_graph: SceneGraph,
    model_instance_name: str,
    n_points=500,
):
    import trimesh

    inspector = scene_graph.model_inspector()

    model_instance = plant.GetModelInstanceByName(model_instance_name)
    body = plant.GetBodyByName("base_link", model_instance)

    # USE VISUAL GEOMETRY (single mesh, no internal faces)
    geometry_ids = plant.GetVisualGeometriesForBody(body)
    if not geometry_ids:
        raise RuntimeError(
            f"No visual geometry found for body '{body.name()}' "
            f"in model '{model_instance_name}'"
        )

    points_B_all = []

    for geom_id in geometry_ids:
        shape = inspector.GetShape(geom_id)
        X_BG = inspector.GetPoseInFrame(geom_id)  # Geometry → Body

        if isinstance(shape, Mesh):
            mesh_path = shape.source().path()

            print("Loading visual mesh:", mesh_path)

            mesh = trimesh.load(mesh_path, force="mesh")
            if not isinstance(mesh, trimesh.Trimesh):
                mesh = trimesh.util.concatenate(mesh.dump())

            pts_G, _ = trimesh.sample.sample_surface(mesh, n_points)

            # G → B
            pts_B = (
                X_BG.rotation().matrix() @ pts_G.T
            ).T + X_BG.translation()

            points_B_all.append(pts_B)

        else:
            # Extremely rare for visuals, but keep fallback
            bbox_min, bbox_max = inspector.GetBoundingBox(geom_id)
            pts_G = np.random.uniform(
                low=bbox_min, high=bbox_max, size=(n_points, 3)
            )
            pts_B = (
                X_BG.rotation().matrix() @ pts_G.T
            ).T + X_BG.translation()
            points_B_all.append(pts_B)

    return np.vstack(points_B_all)


def transform_points_to_world(
    plant: MultibodyPlant,
    context,
    model_instance_name: str,
    points_B: np.ndarray,
):
    # Resolve model instance
    model_instance = plant.GetModelInstanceByName(model_instance_name)

    # Resolve body *within* that model instance
    body = plant.GetBodyByName("base_link", model_instance)

    # Body → World transform
    X_WB = plant.EvalBodyPoseInWorld(context, body)

    # Transform points
    points_W = (
        X_WB.rotation().matrix() @ points_B.T
    ).T + X_WB.translation()

    return points_W


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Compute and visualize a sparse point cloud for the task's target object"
    )
    parser.add_argument("task_file", type=str, help="Path to task JSON file")
    parser.add_argument("dmd_file", type=str, help="Path to Drake .dmd.yaml file")
    parser.add_argument(
        "--package-xml",
        type=str,
        action="append",
        default=[],
        help="Path to a package.xml file. May be specified multiple times.",
    )
    args = parser.parse_args()

    task_file = Path(args.task_file)
    dmd_file = Path(args.dmd_file)
    package_xmls = [Path(p) for p in args.package_xml]

    # --- Load task ---
    task = load_task(task_file)
    target_obj_name = task["commands"][0]["drake_model_name"]

    # --- Load Drake scene ---
    plant = MultibodyPlant(time_step=0.0)
    scene_graph = SceneGraph()
    plant.RegisterAsSourceForSceneGraph(scene_graph)

    parser_drake = Parser(plant)
    # Register all package.xml files
    for pkg_xml in package_xmls:
        register_package_xml(parser_drake, pkg_xml)

    directives = LoadModelDirectives(str(dmd_file))
    ProcessModelDirectives(directives, parser_drake)

    plant.Finalize()
    context = plant.CreateDefaultContext()

    # --- Sample points in body frame ---
    points_body = sample_points_from_body(
        plant,
        scene_graph,
        target_obj_name,
        n_points=200,
    )

    # --- Transform to world frame ---
    points_world = transform_points_to_world(plant, context, target_obj_name, points_body)

    # --- Start Meshcat and visualize ---
    meshcat = StartMeshcat()

    # Visualize point cloud
    pc = PointCloud(points_world.shape[0])
    pc.mutable_xyzs()[:] = points_world.T

    meshcat.SetObject(
        "point_cloud",
        pc,
        point_size=0.01,
        rgba=Rgba(1.0, 1.0, 1.0, 1.0),
    )

    print(f"Visualizing: {dmd_file}")
    print(f"Point cloud for '{target_obj_name}' with {points_world.shape[0]} points")
    print("Meshcat server running. Press Ctrl+C to exit.")

    # Keep objects alive
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\nExiting.")


if __name__ == "__main__":
    main()
