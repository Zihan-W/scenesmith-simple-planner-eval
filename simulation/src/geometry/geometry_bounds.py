"""Shared collision-geometry bounds, independent of item-locking behavior."""
import dataclasses
import numpy as np
from pydrake.all import (Box, Capsule, Cylinder, Ellipsoid, Sphere, Convex, Mesh,
    Shape, TriangleSurfaceMesh, VolumeMesh, ConvertVolumeToSurfaceMesh,
    RigidTransform, SceneGraphInspector, RigidBody)

@dataclasses.dataclass(kw_only=True)
class BoundingBox:
    """A bounding box representation as a 3d center point and 3d overall size
    (i.e., using diameters not radii).
    """

    center: np.array
    size: np.array


# TODO(dale.mcconachie) Replace this dispatch once
# https://github.com/RobotLocomotion/drake/issues/20565 is addressed.
def _GetShapeOabb(shape: Shape) -> BoundingBox:
    """Returns the bounding box in the geometry frame G."""
    center = np.zeros(3)
    if isinstance(shape, Box):
        size = shape.size()
    elif isinstance(shape, Capsule):
        diameter = 2.0 * shape.radius()
        size = np.array([diameter, diameter, shape.length() + diameter])
    elif isinstance(shape, Cylinder):
        diameter = 2.0 * shape.radius()
        size = np.array([diameter, diameter, shape.length()])
    elif isinstance(shape, Ellipsoid):
        size = 2.0 * np.array([shape.a(), shape.b(), shape.c()])
    elif isinstance(shape, Sphere):
        diameter = 2.0 * shape.radius()
        size = np.array([diameter, diameter, diameter])
    elif isinstance(shape, Convex) or isinstance(shape, Mesh):
        center, size = shape.GetConvexHull().CalcBoundingBox()
    else:
        raise NotImplementedError(f"Unimplemented shape {type(shape)}")
    return BoundingBox(center=center, size=size)


def _GetMeshOabb(mesh: TriangleSurfaceMesh | VolumeMesh) -> BoundingBox:
    """Returns the bounding box in the geometry frame G."""
    assert mesh is not None
    if isinstance(mesh, VolumeMesh):
        mesh = ConvertVolumeToSurfaceMesh(mesh)
    assert isinstance(mesh, TriangleSurfaceMesh), mesh
    center, size = mesh.CalcBoundingBox()
    return BoundingBox(center=center, size=size)


def _OabbToAabb(oabb_G: BoundingBox, X_BG: RigidTransform) -> BoundingBox:
    """Converts bounding box from geometry frame G to body frame B."""
    aabb_center_B = X_BG @ oabb_G.center
    # Rather than creating all points and looking for the element-wise maximum
    # of those points, we observe that the x coordinate of the rotated points
    # will follow the pattern   +/- r11 * e_x +/- r12 * e_y +/- r13 * e_z   .
    # Thus the maximum of this value will occur when all of the terms are
    # positive, which is guaranteed to happen given the +/- in the pattern
    # above.  Thus we can directly skip to the final result by taking the
    # absolute value of the elements in the rotation matrix and multiplying
    # them with the size vector which is already known to be positive by
    # definition. The same holds for the y and z components.
    # See https://zeux.io/2010/10/17/aabb-from-obb-with-component-wise-abs/ for
    # a more detailed breakdown.
    aabb_size_B = np.abs(X_BG.rotation().matrix()) @ oabb_G.size
    return BoundingBox(center=aabb_center_B, size=aabb_size_B)


def _MergeAabbs(aabb_list: list[BoundingBox]) -> BoundingBox:
    count = len(aabb_list)
    max_points = np.ndarray(shape=(3, count))
    min_points = np.ndarray(shape=(3, count))
    for i, bbox in enumerate(aabb_list):
        half_size = bbox.size * 0.5
        max_points[:, i] = bbox.center + half_size
        min_points[:, i] = bbox.center - half_size
    max_point = np.max(max_points, axis=1)
    min_point = np.min(min_points, axis=1)
    center = 0.5 * (max_point + min_point)
    size = max_point - min_point
    return BoundingBox(center=center, size=size)


def _CalcAabb(inspector: SceneGraphInspector, body: RigidBody):
    aabb_B_list = list()
    for geom_id in body.GetParentPlant().GetCollisionGeometriesForBody(body):
        # Frame G is the frame of the current geometry.
        maybe_mesh = inspector.maybe_get_hydroelastic_mesh(geom_id)
        if maybe_mesh is not None:
            oabb_G = _GetMeshOabb(maybe_mesh)
        else:
            oabb_G = _GetShapeOabb(inspector.GetShape(geom_id))
        # Convert from the frame G of each individual geometry to the frame B
        # of the body so that they can be aggregated in the common body frame.
        X_BG = inspector.GetPoseInFrame(geom_id)
        aabb_B = _OabbToAabb(oabb_G, X_BG)
        aabb_B_list.append(aabb_B)
    return _MergeAabbs(aabb_B_list)
