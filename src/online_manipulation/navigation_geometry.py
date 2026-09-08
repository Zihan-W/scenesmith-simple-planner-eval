"""Conservative static navigation map derived from actual proximity geometry."""

import itertools
import numpy as np
from pydrake.all import Box, Sphere, Cylinder, Capsule, Mesh, Convex, Role

from src.online_manipulation.navigation import StaticNavigationMap
from src.online_manipulation.scene_geometry import resolve_ground_geometries


def shape_corners(shape):
    """Return enclosing local vertices; fail loudly on unsupported geometry."""
    if isinstance(shape, (Mesh, Convex)):
        hull = shape.GetConvexHull()
        return np.array([hull.vertex(i) for i in range(hull.num_vertices())])
    if isinstance(shape, Box):
        half = np.array([shape.width(), shape.depth(), shape.height()]) / 2
    elif isinstance(shape, Sphere):
        half = np.full(3, shape.radius())
    elif isinstance(shape, (Cylinder, Capsule)):
        half = np.array([shape.radius(), shape.radius(), shape.length() / 2])
        if isinstance(shape, Capsule):
            half[2] += shape.radius()
    else:
        raise TypeError(f"Unsupported navigation shape: {type(shape).__name__}")
    return np.array(list(itertools.product((-1, 1), repeat=3))) * half


def build_navigation_map(
    planning_query,
    *,
    navigation_frame,
    ground_body_names=(),
    ground_geometries=(),
    bounds=(-4.5, -4.5, 4.5, 4.5),
    margin_m=0.03,
):
    """Project scene geometry using the complete configured robot silhouette.

    Includes mast, arms and grippers, not just chassis. The swept disk is
    deliberately conservative and admits all yaw orientations. Ground is
    explicitly identified by body and geometry name; the legacy body-only
    selector is accepted only for a body with exactly one collision geometry.
    Free objects are snapshotted
    as static obstacles; moving obstacles are outside this navigator's scope.
    """
    plant, context = planning_query.plant, planning_query.context
    query = plant.get_geometry_query_input_port().Eval(context)
    inspector = query.inspector()
    floor_ids = resolve_ground_geometries(plant, inspector, ground_body_names, ground_geometries)
    robot = set(plant.GetBodyIndices(planning_query.robot_model_instance))
    X_WN = plant.GetFrameByName(
        navigation_frame, planning_query.robot_model_instance
    ).CalcPoseInWorld(context)
    robot_points, environment_bounds = [], []
    for geometry in inspector.GetAllGeometryIds():
        if inspector.GetProximityProperties(geometry) is None:
            continue
        frame_id = inspector.GetFrameId(geometry)
        body = plant.GetBodyFromFrameId(frame_id)
        qualified = (
            f"{plant.GetModelInstanceName(body.model_instance())}::{body.name()}"
        )
        if geometry in floor_ids:
            continue
        X_WG = query.GetPoseInWorld(geometry)
        vertices = (X_WG @ shape_corners(inspector.GetShape(geometry)).T).T
        if body.index() in robot:
            robot_points.extend(vertices)
        else:
            environment_bounds.append((vertices.min(axis=0), vertices.max(axis=0)))
    if not robot_points:
        raise ValueError("Robot has no proximity geometry for a navigation footprint")
    points = np.array(robot_points)
    radius = float(
        np.linalg.norm(points[:, :2] - X_WN.translation()[:2], axis=1).max() + margin_m
    )
    z_low, z_high = points[:, 2].min(), points[:, 2].max()
    obstacles = tuple(
        (lo[0], lo[1], hi[0], hi[1])
        for lo, hi in environment_bounds
        if hi[2] >= z_low and lo[2] <= z_high
    )
    return StaticNavigationMap(
        obstacles=obstacles, robot_radius_m=radius, bounds=bounds
    )
