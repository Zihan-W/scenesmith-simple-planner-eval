"""Reproducible support extents for the worker's static oriented boxes."""
import math
import numpy as np
import trimesh


def static_oriented_bounds(mesh):
    """Use one OBB frame and fixed-order support projections in float64.

    Trimesh's OBB search chooses a frame, but its reported extents combine
    intermediate 2-D and 3-D projections. Recompute all three from the same
    frame with fsum, avoiding the old independently computed height's last bit.
    This is still a conservative box approximation, not exact mesh collision.
    """
    to_box, _ = trimesh.bounds.oriented_bounds(mesh)
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    projected = np.array([[math.fsum(float(row[j]) * float(v[j]) for j in range(3))
                           for row in to_box[:3, :3]] for v in vertices])
    low, high = projected.min(axis=0), projected.max(axis=0)
    center = (low + high) * 0.5
    to_box[:3, 3] = -center
    # Include the rounding of the center subtraction, then round outwards.
    half = np.maximum(high - center, center - low)
    extents = np.nextafter(2.0 * half, np.inf)
    return np.linalg.inv(to_box), extents
