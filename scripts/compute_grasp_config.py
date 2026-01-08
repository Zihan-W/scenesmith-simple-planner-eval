#!/usr/bin/env python3
import json
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as ET
import argparse
import trimesh
import trimesh.transformations as tf

from pydrake.all import (
    Parser,
    StartMeshcat,
    LoadModelDirectives,
    ProcessModelDirectives,
    Mesh,
    PointCloud,
    Rgba,
    RobotDiagramBuilder,
    VisualizationConfig,
    ApplyVisualizationConfig,
    RigidTransform,
    RotationMatrix,
    ModelInstanceIndex,
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
    plant,
    scene_graph,
    model_instance_name: str,
    n_points=500,
):
    inspector = scene_graph.model_inspector()

    model_instance = plant.GetModelInstanceByName(model_instance_name)
    body = plant.GetBodyByName("base_link", model_instance)

    geometry_ids = plant.GetVisualGeometriesForBody(body)
    if not geometry_ids:
        raise RuntimeError(
            f"No visual geometry found for body '{body.name()}' "
            f"in model '{model_instance_name}'"
        )

    points_B_all = []

    for geom_id in geometry_ids:
        shape = inspector.GetShape(geom_id)
        X_BG = inspector.GetPoseInFrame(geom_id)

        if not isinstance(shape, Mesh):
            raise RuntimeError(
                f"Geometry {geom_id} on body '{body.name()}' is not a mesh."
            )

        mesh_path = shape.source().path()
        print("Loading visual mesh:", mesh_path)

        mesh = trimesh.load(mesh_path, force="mesh", process=False)
        if not isinstance(mesh, trimesh.Trimesh):
            mesh = trimesh.util.concatenate(mesh.dump())

        # --- Fix glTF Y-up → Drake Z-up ---
        X_correction = trimesh.transformations.rotation_matrix(
            np.pi / 2, [1, 0, 0]
        )
        mesh.apply_transform(X_correction)
        # Brittle, but hopefull works for now?

        pts_G, _ = trimesh.sample.sample_surface(mesh, n_points)

        # Geometry → Body
        pts_B = (X_BG.rotation().matrix() @ pts_G.T).T + X_BG.translation()

        points_B_all.append(pts_B)

    return np.vstack(points_B_all)

def transform_points_to_world(
    plant,
    diagram_context,
    model_instance_name: str,
    points_B: np.ndarray,
):
    # Get the plant's *own* context from the Diagram context
    plant_context = plant.GetMyContextFromRoot(diagram_context)

    model_instance = plant.GetModelInstanceByName(model_instance_name)
    body = plant.GetBodyByName("base_link", model_instance)

    X_WB = plant.EvalBodyPoseInWorld(plant_context, body)

    points_W = (
        X_WB.rotation().matrix() @ points_B.T
    ).T + X_WB.translation()

    return points_W

def generate_single_antipodal_grasp(
    diagram,
    plant,
    scene_graph,
    context,
    gripper_model_name: str,
    points_world: np.ndarray,
    meshcat,
    target_model_name: str,
    visualize=True,
):
    """
    Sample a single antipodal grasp on a point cloud and visualize it.

    Args:
        diagram: Parent Diagram object
        plant: MultibodyPlant
        scene_graph: SceneGraph
        context: diagram's Context
        gripper_model_name: str, name of the gripper model instance (e.g., "wsg_50")
        points_world: (N,3) ndarray of object points in world frame
        meshcat: Meshcat instance for visualization
        visualize: whether to publish the gripper pose
    Returns:
        RigidTransform of the gripper pose in world frame
    """
    # 1. Sample a random point
    idx = np.random.randint(points_world.shape[0])
    point = points_world[idx]
    print("Point cloud min:", points_world.min(axis=0))
    print("Point cloud max:", points_world.max(axis=0))
    print("Sampled point:", point)

    # 2. Estimate a normal at that point (simple approximation: use vector to mean)
    normal = point - points_world.mean(axis=0)
    normal /= np.linalg.norm(normal)

    # 3. Choose a gripper frame aligned to the estimated surface normal.
    # We set the gripper x-axis to align with the (outward) normal at the sampled point.
    # To fix the remaining rotation about x, we construct the gripper y-axis as the
    # cross product of a world-up reference direction and the approach direction,
    # then compute z to complete a right-handed frame.

    approach = normal
    approach /= np.linalg.norm(approach)

    # Choose world z as gripper up
    world_z = np.array([0.0, 0.0, 1.0])
    if np.abs(np.dot(approach, world_z)) > 0.95:  # avoid degenerate
        world_z = np.array([0.0, 1.0, 0.0])

    gripper_y = np.cross(world_z, approach)
    gripper_y /= np.linalg.norm(gripper_y)
    gripper_z = np.cross(approach, gripper_y)

    # Column order: [x-axis, y-axis, z-axis]
    gripper_rot = RotationMatrix(np.column_stack([approach, gripper_y, gripper_z]))

    # 4. Place the gripper so the sampled point lands between the fingers.
    # p_GS_G is the position of the sampled point S expressed in the gripper frame G when
    # the object is correctly centered between the fingers (taken from Drake's bin-picking example).
    # Therefore, the gripper origin position is: p_WG = p_WS - R_WG * p_GS_G.

    p_GS_G = np.array([0.054 - 0.01, 0.10625, 0.0])  # [x, y, z] in gripper frame

    # Construct R_WG: x = approach, y = orthogonal, z = cross
    Gx = approach
    Gy = np.array([0.0, 0.0, -1.0])
    Gy -= np.dot(Gy, Gx) * Gx  # make orthogonal
    Gy /= np.linalg.norm(Gy)
    Gz = np.cross(Gx, Gy)
    R_WG = RotationMatrix(np.column_stack([Gx, Gy, Gz]))

    # Sample a single roll about the gripper x-axis (approach axis).
    min_roll = -np.pi / 3.0
    max_roll =  np.pi / 3.0
    theta = np.random.uniform(min_roll, max_roll)

    # Apply roll in the gripper frame: R_WG2 = R_WG * Rx(theta)
    R_WG2 = R_WG.multiply(RotationMatrix.MakeXRotation(theta))

    # Transform finger-box offset into world using the rolled rotation
    p_WG = point - R_WG2.multiply(p_GS_G)
    X_WG = RigidTransform(R_WG2, p_WG)

    # 5. Temporarily move the gripper in the plant to this pose
    gripper_instance = (
        gripper_model_name
        if isinstance(gripper_model_name, ModelInstanceIndex)
        else plant.GetModelInstanceByName(gripper_model_name)
    )
    gripper_body = plant.GetBodyByName("body", gripper_instance)
    plant.SetFreeBodyPose(plant.GetMyContextFromRoot(context), gripper_body, X_WG)
    diagram.ForcedPublish(context)

    # 6. Check collisions: gripper vs everything else (proximity role only)
    sg_context = scene_graph.GetMyContextFromRoot(context)
    query_object = scene_graph.get_query_output_port().Eval(sg_context)
    inspector = query_object.inspector()

    gripper_geometry_ids = set()
    for body_index in plant.GetBodyIndices(gripper_instance):
        body = plant.get_body(body_index)
        gripper_geometry_ids.update(plant.GetCollisionGeometriesForBody(body))

    penetrations = query_object.ComputePointPairPenetration()

    def body_from_geom(gid):
        frame_id = inspector.GetFrameId(gid)
        return plant.GetBodyFromFrameId(frame_id)

    for pen in penetrations:
        a = pen.id_A
        b = pen.id_B

        # Only care about penetrations where exactly one geom is the gripper
        if (a in gripper_geometry_ids) ^ (b in gripper_geometry_ids):
            # Identify which side is gripper
            g = a if a in gripper_geometry_ids else b
            o = b if g == a else a

            # Ignore self-collisions inside the gripper model instance
            body_g = body_from_geom(g)
            body_o = body_from_geom(o)
            if body_g.model_instance() == body_o.model_instance():
                continue

            print("Gripper in collision with scene!")
            print(f"  depth={pen.depth:.6f}")
            print(f"  gripper geom: {inspector.GetName(g)}")
            print(f"  other geom:   {inspector.GetName(o)}")
            return None

    print("Grasp candidate is collision-free!")

    return X_WG

def main():
    parser = argparse.ArgumentParser(
        description="Compute and visualize a sparse point cloud for the task's target object and a grasp"
    )
    parser.add_argument("task_file", type=str)
    parser.add_argument("dmd_file", type=str)
    parser.add_argument(
        "--package-xml",
        type=str,
        action="append",
        default=[],
    )
    args = parser.parse_args()

    task_file = Path(args.task_file)
    dmd_file = Path(args.dmd_file)
    package_xmls = [Path(p) for p in args.package_xml]

    task = load_task(task_file)
    target_obj_name = task["commands"][0]["drake_model_name"]

    # ---------------------------------------------------------------------
    # Build full Drake diagram (world + robot + objects)
    # ---------------------------------------------------------------------
    meshcat = StartMeshcat()
    meshcat.Delete()

    builder = RobotDiagramBuilder()
    parser_drake = builder.parser()

    # Register packages
    for pkg_xml in package_xmls:
        register_package_xml(parser_drake, pkg_xml)

    # Load DMD directives
    directives = LoadModelDirectives(str(dmd_file))
    ProcessModelDirectives(directives, parser_drake)

    plant = builder.plant()
    scene_graph = builder.scene_graph()

    # ---------------------------------------------------------------------
    # Add a free "ghost" WSG50 gripper for visualization
    # ---------------------------------------------------------------------
    ghost_parser = Parser(plant)
    ghost_gripper_instances = ghost_parser.AddModelsFromUrl(
        "package://drake_models/wsg_50_description/sdf/schunk_wsg_50_welded_fingers.sdf"
    )
    ghost_gripper_instance = ghost_gripper_instances[0]
    ghost_body = plant.GetBodyByName("body", ghost_gripper_instance)

    plant.Finalize()

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

    # ---------------------------------------------------------------------
    # Sample points and visualize point cloud
    # ---------------------------------------------------------------------
    points_body = sample_points_from_body(
        plant,
        scene_graph,
        target_obj_name,
        n_points=1000,
    )

    points_world = transform_points_to_world(
        plant, diagram_context, target_obj_name, points_body
    )

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

    # ---------------------------------------------------------------------
    # Interactive grasp sampling loop
    # ---------------------------------------------------------------------
    print("Press <space> then <enter> to sample a new grasp. Type 'q' then <enter> to quit.")

    # Make sure the world is drawn once.
    diagram.ForcedPublish(diagram_context)

    grasp_count = 0
    try:
        while True:
            s = input().strip("\n")
            if s.lower() == "q":
                break
            if s != "" and s != " ":
                # Ignore other inputs; only accept blank or a single space.
                continue

            # Optional: clear previous visualization path if you want
            # meshcat.Delete("gripper_candidate")

            X_grasp = generate_single_antipodal_grasp(
                diagram,
                plant,
                scene_graph,
                diagram_context,
                gripper_model_name=ghost_gripper_instance,  # ModelInstanceIndex
                points_world=points_world,
                meshcat=meshcat,
                target_model_name=target_obj_name,
                visualize=True,
            )

            if X_grasp is not None:
                grasp_count += 1
                print(f"\nGrasp #{grasp_count} candidate pose (world frame):")
                print(X_grasp)
            else:
                print("\nRejected grasp (collision).")

            # Publish so Meshcat updates any SceneGraph visuals (not strictly
            # required for SetTransform, but good practice if you add more later).
            diagram.ForcedPublish(diagram_context)

    except KeyboardInterrupt:
        pass

    print("\nExiting.")


if __name__ == "__main__":
    main()
