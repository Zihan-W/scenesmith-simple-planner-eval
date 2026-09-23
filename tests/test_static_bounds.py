"""Static box producer reproducibility and mesh containment."""
import unittest
import numpy as np
import trimesh
from planner.src.tamp.cutamp_backend.static_bounds import static_oriented_bounds


class StaticBoundsTests(unittest.TestCase):
    def test_deterministic_containing_box_for_rotated_asymmetric_mesh(self):
        mesh=trimesh.creation.box(extents=[.0083,.0087,.045])
        transform=trimesh.transformations.euler_matrix(.3,.5,.7)
        transform[:3,3]=[.21,.15,.02]
        mesh.apply_transform(transform)
        first=static_oriented_bounds(mesh)
        for _ in range(10):
            actual=static_oriented_bounds(mesh.copy())
            np.testing.assert_array_equal(first[0],actual[0])
            np.testing.assert_array_equal(first[1],actual[1])
        box_from_mesh=np.linalg.inv(first[0])
        points=trimesh.transform_points(mesh.vertices,box_from_mesh)
        self.assertLessEqual(float((abs(points)-first[1]/2).max()),1e-15)
        np.testing.assert_allclose(np.sort(first[1]),[.0083,.0087,.045],atol=1e-15,rtol=0)
