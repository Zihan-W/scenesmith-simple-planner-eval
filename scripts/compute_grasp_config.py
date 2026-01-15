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
    Role,
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


def _collision_geometry_ids_for_instance(plant, model_instance: ModelInstanceIndex) -> set:
    geom_ids = set()
    for body_index in plant.GetBodyIndices(model_instance):
        body = plant.get_body(body_index)
        geom_ids.update(plant.GetCollisionGeometriesForBody(body))
    return geom_ids


def gripper_pose_in_collision(
    *,
    plant,
    scene_graph,
    diagram_context,
    gripper_instance: ModelInstanceIndex,
    X_WG: RigidTransform,
    ignore_instances: set[ModelInstanceIndex] | None = None,
    _cached_gripper_geom_ids: set | None = None,
) -> bool:
    """
    Returns True if gripper penetrates anything (except itself, and optionally
    anything belonging to ignore_instances).
    """
    if ignore_instances is None:
        ignore_instances = set()

    plant_context = plant.GetMyContextFromRoot(diagram_context)

    # Move the free-floating gripper body to X_WG (ghost gripper should be floating)
    gripper_body = plant.GetBodyByName("body", gripper_instance)
    plant.SetFreeBodyPose(plant_context, gripper_body, X_WG)

    sg_context = scene_graph.GetMyContextFromRoot(diagram_context)
    query_object = scene_graph.get_query_output_port().Eval(sg_context)
    inspector = query_object.inspector()

    gripper_geom_ids = (
        _cached_gripper_geom_ids
        if _cached_gripper_geom_ids is not None
        else _collision_geometry_ids_for_instance(plant, gripper_instance)
    )

    penetrations = query_object.ComputePointPairPenetration()

    def body_from_geom(gid):
        frame_id = inspector.GetFrameId(gid)
        return plant.GetBodyFromFrameId(frame_id)

    for pen in penetrations:
        a = pen.id_A
        b = pen.id_B

        # exactly one side is gripper
        if (a in gripper_geom_ids) ^ (b in gripper_geom_ids):
            g = a if a in gripper_geom_ids else b
            o = b if g == a else a

            body_g = body_from_geom(g)
            body_o = body_from_geom(o)

            # ignore gripper self-collisions
            if body_g.model_instance() == body_o.model_instance():
                continue

            # ignore collisions with specified model instances (e.g., the target object)
            if body_o.model_instance() in ignore_instances:
                continue

            return True

    return False


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
    # print("Point cloud min:", points_world.min(axis=0))
    # print("Point cloud max:", points_world.max(axis=0))
    # print("Sampled point:", point)

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

            # print("Gripper in collision with scene!")
            # print(f"  depth={pen.depth:.6f}")
            # print(f"  gripper geom: {inspector.GetName(g)}")
            # print(f"  other geom:   {inspector.GetName(o)}")
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


def _collision_geometry_ids_by_name_substr(scene_graph, name_substr: str):
    ids = set()
    inspector = scene_graph.model_inspector()
    for gid in inspector.GetAllGeometryIds():
        if name_substr in inspector.GetName(gid):
            ids.add(gid)
    return ids


def _collision_geometry_ids_for_body_name(plant, scene_graph, model_instance, body_name: str):
    """Returns collision GeometryIds for the named body in the given model instance."""
    body = plant.GetBodyByName(body_name, model_instance)
    frame_id = plant.GetBodyFrameIdOrThrow(body.index())
    inspector = scene_graph.model_inspector()
    return set(inspector.GetGeometries(frame_id, role=Role.kProximity))


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
    F = _collision_geometry_ids_by_name_substr(scene_graph, "floor_collision")
    Z = _collision_geometry_ids_for_body_name(plant, scene_graph, mobile_iiwa_instance, "iiwa_base_z_column")
    ALL = _all_collision_geometry_ids(plant)
    E = set(ALL) - set(A) - set(G) - set(H)  # environment

    setA = GeometrySet(list(A))
    setG = GeometrySet(list(G))
    setH = GeometrySet(list(H))
    setE = GeometrySet(list(E))
    setF = GeometrySet(list(F))
    setZ = GeometrySet(list(Z))

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

    # 5) Eliminate mobile iiwa lift joint <-> floor collisions
    if Z and F:
        decl.ExcludeBetween(setZ, setF)

    cfm.Apply(decl)

def solve_ik_for_pose(
    X_WE,
    diagram,
    plant,
    scene_graph,
    ghost_gripper_instance,
    world_xy_bounds=[-10, 10, -10, 10], # should specify if q_initial_guess or q_ref is not given
    q_ref=None,                 # <-- center cost around this
    q_initial_guess=None,       # <-- initial guess
    arm_position_count=11,
):
    # -------------------------
    # Main IK context
    # -------------------------
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

    lock_joints_outside_first_n_positions(plant, plant_context, arm_position_count)
    ik = InverseKinematics(plant, plant_context, with_joint_limits=True)

    # -------------------------
    # End-effector pose constraint
    # -------------------------
    wsg_instance = plant.GetModelInstanceByName("wsg_50")
    ee_body = plant.GetBodyByName("body", wsg_instance)
    frame_E = ee_body.body_frame()
    frame_W = plant.world_frame()

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
    # Quadratic cost: stay near q_ref (on arm dofs)
    # -------------------------
    if q_ref is None:
        q_ref = plant.GetPositions(plant_context).copy()
    else:
        q_ref = np.asarray(q_ref).copy()
        prog.AddQuadraticErrorCost(np.eye(3), q_ref[:3], q[:3])

    if q_initial_guess is None:
        q_initial_guess = np.random.random(q_ref.shape)
        low = (world_xy_bounds[0], world_xy_bounds[2])
        high = (world_xy_bounds[1], world_xy_bounds[3])
        q_initial_guess[:2] = np.random.uniform(low, high)

    idx = np.arange(3, 12)  # arm dofs in your convention
    Q = np.eye(len(idx))
    Q[0] *= 10.0  # keep your previous weighting choice
    prog.AddQuadraticErrorCost(Q, q_ref[idx], q[idx])
    prog.SetInitialGuess(q, q_initial_guess)

    # -------------------------
    # Collision constraints (same as your existing code)
    # -------------------------
    mobile_iiwa_instance = plant.GetModelInstanceByName("mobile_iiwa")
    influence_distance = 0.02

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
        0.01,
        plant_context_arm,
        influence_distance_offset=influence_distance,
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
        0.0,
        plant_context_grip,
        influence_distance_offset=influence_distance,
    )
    prog.AddConstraint(c_grip, q)

    # -------------------------
    # Solve
    # -------------------------
    solver = SnoptSolver()
    options = SolverOptions()
    options.SetOption(CommonSolverOption.kPrintFileName, "snopt.log")
    options.SetOption(solver.solver_id(), "Major print level", 1)
    options.SetOption(solver.solver_id(), "Timing level", 3)
    options.SetOption(solver.solver_id(), "Time Limit", 10)
    options.SetOption(solver.solver_id(), "Major optimality tolerance", 1e-1)

    result = solver.Solve(prog, None, options)
    if not result.is_success():
        print("IK failed")
        return None

    print("IK succeeded")
    return result.GetSolution(q)

def solve_ik_for_grasp(X_grasp, diagram, plant, scene_graph, ghost_gripper_instance, world_xy_bounds):
    return solve_ik_for_pose(X_grasp, diagram, plant, scene_graph, ghost_gripper_instance, world_xy_bounds)


def hat(w):
    wx, wy, wz = w
    return np.array([[0, -wz, wy],
                     [wz, 0, -wx],
                     [-wy, wx, 0]], dtype=float)

def so3_exp(w):
    """Exponential map from axis-angle vector w to rotation matrix."""
    theta = np.linalg.norm(w)
    if theta < 1e-12:
        # First-order approximation
        return np.eye(3) + hat(w)
    K = hat(w / theta)
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

def sample_rotation_gaussian(R0, Sigma, n=1, side="right", rng=None):
    """
    R = R0 * Exp([eps]_x)  (side='right')  or  R = Exp([eps]_x) * R0 (side='left')
    eps ~ N(0, Sigma) in R^3.
    """
    if rng is None:
        rng = np.random.default_rng()
    eps = rng.multivariate_normal(mean=np.zeros(3), cov=Sigma, size=n)
    Rs = []
    for e in eps:
        dR = so3_exp(e)
        Rs.append(R0 @ dR if side == "right" else dR @ R0)
    return Rs[0] if n == 1 else np.stack(Rs, axis=0)


def compute_target_pose_collision_free(
    *,
    task,
    plant,
    scene_graph,
    diagram_context,
    target_obj_name: str,
    X_grasp: RigidTransform,          # world->gripper at grasp time
    ghost_gripper_instance,           # ModelInstanceIndex or name
    z_offset: float = 0.002,
    ignore_target_object: bool = True,
    visualize: bool = False,
    diagram=None,                     # only needed if visualize=True
):
    """
    Samples an object placement target pose within the bounds, rejects samples where
    the *ghost gripper at the implied goal pose* is in penetration with the scene.
    Returns X_WG_goal or None if no collision-free sample is found.
    """
    rng = np.random.default_rng()

    cmd = task["commands"][0]
    lo = np.array(cmd["placement_bounds_min"], dtype=float)
    hi = np.array(cmd["placement_bounds_max"], dtype=float)

    plant_context = plant.GetMyContextFromRoot(diagram_context)

    # Resolve instances
    obj_instance = plant.GetModelInstanceByName(target_obj_name)
    gripper_instance = (
        ghost_gripper_instance
        if isinstance(ghost_gripper_instance, ModelInstanceIndex)
        else plant.GetModelInstanceByName(ghost_gripper_instance)
    )

    # Get single-body handles (same assumption you made)
    obj_body = plant.get_body(plant.GetBodyIndices(obj_instance)[0])
    gripper_body = plant.GetBodyByName("body", gripper_instance)

    # Cache gripper geometry ids (speed)
    gripper_geom_ids = _collision_geometry_ids_for_instance(plant, gripper_instance)

    X_WO_orig = plant.EvalBodyPoseInWorld(plant_context, obj_body)

    # Compute grasp transform relative to object at the *current* context
    # (This assumes X_grasp corresponds to this same scene state, which it likely does right after grasping.)
    X_WO = X_WO_orig
    X_OG = X_WO.inverse() @ X_grasp

    ignore_instances = {obj_instance} if ignore_target_object else set()

    # Sample object target position uniformly in AABB
    p_WO_goal = rng.uniform(lo, hi)
    p_WO_goal[2] += z_offset

    random_orientation = sample_rotation_gaussian(
        X_WO.rotation().matrix(),
        Sigma = 0.5 * np.eye(3)
    )

    # Keep current object orientation, only change translation
    X_WO_goal = RigidTransform(RotationMatrix(random_orientation), p_WO_goal)

    # Implied gripper target pose preserving grasp
    X_WG_goal = X_WO_goal @ X_OG

    # Temporarily set object to goal pose for collision checking
    plant.SetFreeBodyPose(plant_context, obj_body, X_WO_goal)

    # Check gripper penetrations at the implied goal pose
    in_collision = gripper_pose_in_collision(
        plant=plant,
        scene_graph=scene_graph,
        diagram_context=diagram_context,
        gripper_instance=gripper_instance,
        X_WG=X_WG_goal,
        ignore_instances=ignore_instances,
        _cached_gripper_geom_ids=gripper_geom_ids,
    )

    plant.SetFreeBodyPose(plant_context, obj_body, X_WO_orig)

    if not in_collision:
        return X_WG_goal

    return None


def retreat_along_gripper_y(X_WG: RigidTransform, distance: float) -> RigidTransform:
    return X_WG @ RigidTransform([0.0, -distance, 0.0])

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
    parser.add_argument(
        "--approach-distance",
        type=float,
        default=0.10,  # 10 cm
        help="Distance (m) to retreat along gripper +x axis for pregrasp/postplace IK targets.",
    )
    args = parser.parse_args()

    task_file = Path(args.task_file)
    dmd_file = Path(args.dmd_file)
    package_xmls = [Path(p) for p in args.package_xml]
    approach_distance = float(args.approach_distance)

    task = load_task(task_file)
    target_obj_name = task["commands"][0]["drake_model_name"]
    wb_min = task["world_bounds"]["min"]
    wb_max = task["world_bounds"]["max"]
    world_xy_bounds = (float(wb_min[0]), float(wb_max[0]), float(wb_min[1]), float(wb_max[1]))

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
    for pkg_xml in package_xmls:
        register_package_xml(ghost_parser, pkg_xml)
    ghost_gripper_instances = ghost_parser.AddModelsFromUrl(
        "package://mobile_iiwa/schunk_wsg_50_welded_fingers_and_wrist_geometry.sdf"
    )
    ghost_gripper_instance = ghost_gripper_instances[0]

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
    q_start = plant.GetPositions(plant_context).copy()[:11]

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
    q_grasp_last = None      # stores last successful grasp configuration
    q_place_last = None      # stores last successful place configuration
    q_pregrasp_last = None
    q_postplace_last = None

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
                if q_pregrasp_last is None or q_postplace_last is None:
                    print("Need successful pregrasp and postplace IK before saving.")
                    continue

                data = {
                    "waypoints": [
                        {"name": "start",     "q": q_start.tolist()},
                        {"name": "pregrasp",  "q": q_pregrasp_last[:11].tolist()},
                        {"name": "grasp",     "q": q_grasp_last[:11].tolist()},
                        {"name": "place",     "q": q_place_last[:11].tolist()},
                        {"name": "postplace", "q": q_postplace_last[:11].tolist()},
                        {"name": "start",     "q": q_start.tolist()},
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

                while True:
                    X_target = compute_target_pose_collision_free(
                        task=task,
                        plant=plant,
                        scene_graph=scene_graph,
                        diagram_context=diagram_context,
                        target_obj_name=target_obj_name,
                        X_grasp=last_grasp_pose,
                        ghost_gripper_instance=ghost_gripper_instance,
                        z_offset=0.002,
                        ignore_target_object=True,   # usually yes: you expect the gripper to be “touching” the object
                        visualize=True,             # set True if you pass diagram=diagram
                        diagram=diagram,
                    )
                    diagram.ForcedPublish(diagram_context)

                    if X_target is None:
                        print("Rejected place (collision).")
                    else:
                        break


                print(f"\nPlace candidate pose (world frame):")
                print(X_target)

                q_place = solve_ik_for_grasp(
                    X_target, diagram, plant, scene_graph, ghost_gripper_instance, world_xy_bounds
                )
                if q_place is None:
                    print("Place IK failed.")
                    continue

                q_place_last = np.asarray(q_place).copy()

                # Solve postplace: retreat along gripper y-axis, cost centered at place q
                X_postplace = retreat_along_gripper_y(X_target, approach_distance)
                q_postplace = solve_ik_for_pose(
                    X_postplace,
                    diagram, plant, scene_graph, ghost_gripper_instance,
                    q_ref=q_place_last,
                    q_initial_guess=q_place_last,
                )
                if q_postplace is None:
                    print("Postplace IK failed (keeping place anyway).")
                    q_postplace_last = None
                    continue
                else:
                    q_postplace_last = np.asarray(q_postplace).copy()
                    print("Postplace IK succeeded.")

                plant.SetPositions(plant_context, q_place_last)
                diagram.ForcedPublish(diagram_context)
                print("Place IK succeeded.")
                continue

            # Grasp sampling: accept "" or " " only
            if s != "" and s != " ":
                continue

            while True:
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
                    print("Rejected grasp (collision).")
                    diagram.ForcedPublish(diagram_context)
                else:
                    break

            grasp_count += 1
            print(f"\nGrasp #{grasp_count} candidate pose (world frame):")
            print(X_grasp)

            q_grasp = solve_ik_for_grasp(
                X_grasp, diagram, plant, scene_graph, ghost_gripper_instance, world_xy_bounds
            )
            if q_grasp is None:
                print("Grasp IK failed.")
                diagram.ForcedPublish(diagram_context)
                continue

            q_grasp_last = np.asarray(q_grasp).copy()
            last_grasp_pose = X_grasp

            # Solve pregrasp: retreat along gripper y-axis, cost centered at grasp q
            X_pregrasp = retreat_along_gripper_y(last_grasp_pose, approach_distance)
            q_pregrasp = solve_ik_for_pose(
                X_pregrasp,
                diagram, plant, scene_graph, ghost_gripper_instance,
                q_ref=q_grasp_last,
                q_initial_guess=q_grasp_last,
            )
            if q_pregrasp is None:
                print("Pregrasp IK failed (keeping grasp anyway).")
                q_pregrasp_last = None
            else:
                q_pregrasp_last = np.asarray(q_pregrasp).copy()
                print("Pregrasp IK succeeded.")

            plant.SetPositions(plant_context, q_grasp_last)
            diagram.ForcedPublish(diagram_context)
            print("Grasp IK succeeded.")

    except KeyboardInterrupt:
        pass

    print("\nExiting.")


if __name__ == "__main__":
    main()
