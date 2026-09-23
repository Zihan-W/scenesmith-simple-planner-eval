"""Exact convex hull of the existing rounded constant-heading parking anchors."""

import numpy as np
from scipy.spatial import ConvexHull


def base_domain_halfspaces(candidates, yaw):
    """Return normalized world-XY halfspaces A x + b <= 0 in meters."""
    points = np.asarray(candidates, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("Expected finite world x/y/yaw parking candidates")
    if not np.allclose(np.cos(points[:, 2]), np.cos(yaw), atol=1e-7, rtol=0) or not np.allclose(
            np.sin(points[:, 2]), np.sin(yaw), atol=1e-7, rtol=0):
        raise ValueError("Parking domain heading differs from the observed anchor")
    return ConvexHull(points[:, :2]).equations
