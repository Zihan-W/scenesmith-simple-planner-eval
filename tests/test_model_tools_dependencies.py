"""Check declared model-tool dependencies and the mesh operations they support."""

import logging
from pathlib import Path
import tomllib
import unittest


class ModelToolsDependenciesTest(unittest.TestCase):
    """Prevent model preparation from relying on preinstalled dev extras."""

    def test_model_tools_declares_scipy(self):
        """The user-facing extra must install both mesh and numerical tools."""
        root = Path(__file__).resolve().parents[1]
        project = tomllib.loads((root / "pyproject.toml").read_text())["project"]
        requirements = project["optional-dependencies"]["model-tools"]
        self.assertIn("trimesh==4.11.0", requirements)
        self.assertIn("scipy==1.15.3", requirements)

    def test_normals_and_convex_hull_without_fallback(self):
        """Exercise OBJ normals and convex proxies without Trimesh warnings."""
        import numpy as np
        import scipy.sparse
        import trimesh

        self.assertTrue(callable(scipy.sparse.coo_matrix))
        with self.assertNoLogs("trimesh", level=logging.WARNING):
            mesh = trimesh.creation.icosphere(subdivisions=1)
            obj = mesh.export(file_type="obj", include_normals=True)
            hull = trimesh.convex.convex_hull(mesh.vertices)
        self.assertIn("\nvn ", obj)
        self.assertTrue(np.isfinite(mesh.vertex_normals).all())
        self.assertGreater(hull.volume, 0)


if __name__ == "__main__":
    unittest.main()
