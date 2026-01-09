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
    InverseKinematics,
    JointIndex,
    Solve,
    CollisionFilterDeclaration,
    GeometrySet,
    MinimumDistanceLowerBoundConstraint,
    BodyIndex,
    SnoptSolver,
    SolverOptions,
    CommonSolverOption,
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
    # Reset all generalized positions (q) for the entire plant.
    plant.SetPositions(plant.GetMyContextFromRoot(context), plant.GetDefaultPositions())

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

def lock_joints_outside_first_n_positions(plant, plant_context, n_active_positions: int):
    """
    Locks any joint whose position indices are not entirely within [0, n_active_positions-1].
    This is a robust way to keep only a prefix of the plant's position vector free.

    Example:
      n_active_positions=11 => positions[0:11] are free, all others are locked.
    """
    for i in range(plant.num_joints()):
        joint = plant.get_joint(JointIndex(i))
        npos = joint.num_positions()
        if npos == 0:
            continue

        start = joint.position_start()
        pos_indices = range(start, start + npos)

        keep_unlocked = all(k < n_active_positions for k in pos_indices)
        if not keep_unlocked:
            joint.Lock(plant_context)

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

def apply_robot_environment_collision_filters(
    *,
    plant,
    scene_graph,
    sg_context,
    mobile_iiwa_instance,
    wsg_instance,
    ghost_gripper_instance,
    mode: str,  # "arm_only" or "gripper_only"
):
    """
    Configures SceneGraph collision filters so that the only remaining candidate
    pairs are:
      - mode="arm_only":     mobile_iiwa <-> environment
      - mode="gripper_only": wsg_50      <-> environment

    Always excludes anything involving the ghost gripper, and excludes env<->env,
    robot<->robot, etc.
    """
    if mode not in ("arm_only", "gripper_only"):
        raise ValueError(f"Unknown mode: {mode}")

    A = _collision_geometry_ids_for_instance(plant, mobile_iiwa_instance)   # arm
    G = _collision_geometry_ids_for_instance(plant, wsg_instance)          # real gripper
    H = _collision_geometry_ids_for_instance(plant, ghost_gripper_instance)  # ghost
    ALL = _all_collision_geometry_ids(plant)
    E = set(ALL) - set(A) - set(G) - set(H)  # environment

    setA = GeometrySet(list(A))
    setG = GeometrySet(list(G))
    setH = GeometrySet(list(H))
    setE = GeometrySet(list(E))

    cfm = scene_graph.collision_filter_manager(sg_context)
    decl = CollisionFilterDeclaration()

    # 1) Ghost should never collide with anything (robot or env).
    decl.ExcludeWithin(setH)
    decl.ExcludeBetween(setH, setA)
    decl.ExcludeBetween(setH, setG)
    decl.ExcludeBetween(setH, setE)

    # 2) Never care about env-env or self collisions within components.
    decl.ExcludeWithin(setE)
    decl.ExcludeWithin(setA)
    decl.ExcludeWithin(setG)

    # 3) Never care about robot internal collisions between arm and gripper here.
    decl.ExcludeBetween(setA, setG)

    # 4) Remove whichever robot-vs-env pairs we *don't* want, leaving only one family.
    if mode == "arm_only":
        # Leave only A <-> E; so exclude G <-> E.
        decl.ExcludeBetween(setG, setE)
    else:
        # Leave only G <-> E; so exclude A <-> E.
        decl.ExcludeBetween(setA, setE)

    cfm.Apply(decl)

def solve_ik_for_grasp(X_grasp, diagram, plant, scene_graph, ghost_gripper_instance):
    # -------------------------
    # Main IK context (decision variables live here)
    # -------------------------
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

    arm_position_count = 11
    lock_joints_outside_first_n_positions(plant, plant_context, arm_position_count)

    ik = InverseKinematics(plant, plant_context, with_joint_limits=True)

    # -------------------------
    # End-effector pose constraint: "wsg_50" body frame matches X_grasp
    # -------------------------
    wsg_instance = plant.GetModelInstanceByName("wsg_50")
    ee_body = plant.GetBodyByName("body", wsg_instance)
    frame_E = ee_body.body_frame()
    frame_W = plant.world_frame()

    X_WE = X_grasp

    # Position constraint (origin of E)
    p_tol = 0.002
    p_WE = X_WE.translation()
    p_BQ = np.zeros((3, 1))
    p_AQ_lower = (p_WE - p_tol).reshape(3, 1)
    p_AQ_upper = (p_WE + p_tol).reshape(3, 1)

    ik.AddPositionConstraint(
        frameB=frame_E,
        p_BQ=p_BQ,
        frameA=frame_W,
        p_AQ_lower=p_AQ_lower,
        p_AQ_upper=p_AQ_upper,
    )

    # Orientation constraint
    theta_tol = 0.02
    R_WE = X_WE.rotation()
    ik.AddOrientationConstraint(
        frameAbar=frame_W,
        R_AbarA=R_WE,
        frameBbar=frame_E,
        R_BbarB=RotationMatrix(),
        theta_bound=theta_tol,
    )

    prog = ik.prog()
    q = ik.q()

    # -------------------------
    # Quadratic posture cost on q[3:12]
    # -------------------------
    q_nom = plant.GetPositions(plant_context).copy()
    idx = np.arange(3, 12)  # 3..11 inclusive
    w = 1.0
    Q = w * np.eye(len(idx))
    Q[0] *= 10.0
    prog.AddQuadraticErrorCost(Q, q_nom[idx], q[idx])
    prog.SetInitialGuess(q, q_nom)

    # -------------------------
    # Collision constraints: two contexts + two explicit constraints
    # -------------------------
    mobile_iiwa_instance = plant.GetModelInstanceByName("mobile_iiwa")
    influence_distance = 0.02  # tune

    # Context A: arm<->environment, lower bound 1 cm
    diagram_context_arm = diagram.CreateDefaultContext()
    plant_context_arm = diagram.GetMutableSubsystemContext(plant, diagram_context_arm)
    sg_context_arm = diagram.GetMutableSubsystemContext(scene_graph, diagram_context_arm)
    lock_joints_outside_first_n_positions(plant, plant_context_arm, arm_position_count)

    apply_robot_environment_collision_filters(
        plant=plant,
        scene_graph=scene_graph,
        sg_context=sg_context_arm,
        mobile_iiwa_instance=mobile_iiwa_instance,
        wsg_instance=wsg_instance,
        ghost_gripper_instance=ghost_gripper_instance,
        mode="arm_only",
    )

    c_arm = MinimumDistanceLowerBoundConstraint(
        plant,
        0.01,              # bound (1 cm)
        plant_context_arm,
        influence_distance_offset=0.02,
    )
    prog.AddConstraint(c_arm, q)

    # Context B: gripper<->environment, lower bound 0 cm
    diagram_context_grip = diagram.CreateDefaultContext()
    plant_context_grip = diagram.GetMutableSubsystemContext(plant, diagram_context_grip)
    sg_context_grip = diagram.GetMutableSubsystemContext(scene_graph, diagram_context_grip)
    lock_joints_outside_first_n_positions(plant, plant_context_grip, arm_position_count)

    apply_robot_environment_collision_filters(
        plant=plant,
        scene_graph=scene_graph,
        sg_context=sg_context_grip,
        mobile_iiwa_instance=mobile_iiwa_instance,
        wsg_instance=wsg_instance,
        ghost_gripper_instance=ghost_gripper_instance,
        mode="gripper_only",
    )

    c_grip = MinimumDistanceLowerBoundConstraint(
        plant,
        0.0,               # bound (0 cm)
        plant_context_grip,
        influence_distance_offset=0.02,
    )
    prog.AddConstraint(c_grip, q)

    # -------------------------
    # Solve
    # -------------------------

    solver = SnoptSolver()
    options = SolverOptions()

    options.SetOption(CommonSolverOption.kPrintFileName, "snopt.log")
    options.SetOption(SnoptSolver().solver_id(), "Major print level", 1)  # 1 = summary, 0 = none, >1 = verbose
    options.SetOption(SnoptSolver().solver_id(), "Timing level", 3)  # Need to enable timing for time limits to work
    options.SetOption(SnoptSolver().solver_id(), "Time Limit", 60)
    options.SetOption(SnoptSolver().solver_id(), "Major optimality tolerance", 1e-1)

    result = solver.Solve(prog, None, options)
    if not result.is_success():
        print("IK failed")
        return None

    print("IK succeeded")

    return result.GetSolution(q)

def compute_target_pose(
    *,
    task,
    plant,
    diagram_context,
    target_obj_name: str,
    X_grasp: RigidTransform,
    z_offset: float = 0.002,
    rng: np.random.Generator | None = None,
):
    """
    Samples an object placement target pose within [placement_bounds_min, placement_bounds_max],
    then returns the corresponding gripper target pose that preserves the grasp transform.

    Returns:
      X_WG_goal (RigidTransform): world->gripper pose
    """
    if rng is None:
        rng = np.random.default_rng()

    cmd = task["commands"][0]
    lo = np.array(cmd["placement_bounds_min"], dtype=float)
    hi = np.array(cmd["placement_bounds_max"], dtype=float)

    # Sample object target position uniformly in the AABB
    p_WO_goal = rng.uniform(lo, hi)
    p_WO_goal[2] += z_offset

    # Current object pose in world (for X_OG computation)
    obj_instance = plant.GetModelInstanceByName(target_obj_name)
    body_indices = plant.GetBodyIndices(obj_instance)
    obj_body = plant.get_body(body_indices[0])  # ok for single-body objects; refine if needed

    X_WO = plant.EvalBodyPoseInWorld(
        plant.GetMyContextFromRoot(diagram_context), obj_body
    )

    # Preserve relative grasp: X_OG
    X_OG = X_WO.inverse() @ X_grasp

    # Build goal object pose: keep current orientation, new sampled translation
    X_WO_goal = RigidTransform(X_WO.rotation(), p_WO_goal)

    # Convert to desired gripper pose
    X_WG_goal = X_WO_goal @ X_OG
    return X_WG_goal

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
    last_grasp_pose = None   # stores last successful grasp pose (X_WG)
    last_grasp_q = None      # stores last successful grasp configuration

    # Save at the base level of the repository (parent directory of the folder containing this .py file)
    waypoints_path = Path(__file__).resolve().parent.parent / "robot_waypoints.json"

    print("Controls:")
    print("  <enter> or <space> + <enter> : sample grasp + solve IK")
    print("  p + <enter>                  : sample place target + solve IK")
    print("  s + <enter>                  : save grasp+place waypoints to robot_waypoints.json")
    print("  q + <enter>                  : quit")

    # Make sure the world is drawn once.
    diagram.ForcedPublish(diagram_context)

    grasp_count = 0
    try:
        while True:
            s = input().strip("\n").lower()

            if s == "q":
                break

            # Save waypoints
            if s == "s":
                if q_grasp_last is None or q_place_last is None:
                    print("Need both a successful grasp and place IK before saving.")
                    continue

                data = {
                    "waypoints": [
                        {"name": "grasp", "q": q_grasp_last[:11].tolist()},
                        {"name": "place", "q": q_place_last[:11].tolist()},
                    ]
                }

                with open(waypoints_path, "w") as f:
                    json.dump(data, f, indent=2)

                print(f"Saved waypoints to: {waypoints_path}")
                continue

            # Place IK
            if s == "p":
                if last_grasp_pose is None:
                    print("No successful grasp yet — sample a grasp first.")
                    continue

                X_target = compute_target_pose(
                    task=task,
                    plant=plant,
                    diagram_context=diagram_context,
                    target_obj_name=target_obj_name,
                    X_grasp=last_grasp_pose,
                    z_offset=0.002,
                    # rng=rng,   # if you added deterministic sampling
                )

                q_place = solve_ik_for_grasp(
                    X_target, diagram, plant, scene_graph, ghost_gripper_instance
                )
                if q_place is None:
                    print("Place IK failed.")
                    continue

                q_place_last = np.asarray(q_place).copy()
                plant.SetPositions(plant_context, q_place_last)
                diagram.ForcedPublish(diagram_context)
                print("Place IK succeeded.")
                continue

            # Grasp sampling: accept "" or " " only
            if s != "" and s != " ":
                continue

            X_grasp = generate_single_antipodal_grasp(
                diagram,
                plant,
                scene_graph,
                diagram_context,
                gripper_model_name=ghost_gripper_instance,
                points_world=points_world,
                meshcat=meshcat,
                target_model_name=target_obj_name,
                visualize=True,
            )

            if X_grasp is None:
                print("\nRejected grasp (collision).")
                diagram.ForcedPublish(diagram_context)
                continue

            grasp_count += 1
            print(f"\nGrasp #{grasp_count} candidate pose (world frame):")
            print(X_grasp)

            q_grasp = solve_ik_for_grasp(
                X_grasp, diagram, plant, scene_graph, ghost_gripper_instance
            )
            if q_grasp is None:
                print("Grasp IK failed.")
                diagram.ForcedPublish(diagram_context)
                continue

            q_grasp_last = np.asarray(q_grasp).copy()
            last_grasp_pose = X_grasp

            plant.SetPositions(plant_context, q_grasp_last)
            diagram.ForcedPublish(diagram_context)
            print("Grasp IK succeeded.")

    except KeyboardInterrupt:
        pass

    print("\nExiting.")


if __name__ == "__main__":
    main()
