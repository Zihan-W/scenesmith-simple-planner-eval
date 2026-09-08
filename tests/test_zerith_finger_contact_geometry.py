"""Regression checks against the visual CAD, independent of grasp success."""

import dataclasses
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import ConvexHull

from src.online_manipulation.recipes.minimal import make_config
from src.online_manipulation import GripperAction, make_env
from src.zerith_gripper_config import FINGER_CLOSING_TRAVEL_M


class FingerContactGeometryTest(unittest.TestCase):
    """Ensure contact envelopes enclose material without filling the jaw gap."""

    def test_distal_visual_surface_is_enclosed_by_its_contact_mesh(self):
        """Reject collision surfaces inset from the visible finger material."""
        root = Path(__file__).resolve().parents[1] / "models/zerith_drake"
        for arm in ("left", "right"):
            for jaw in ("left", "right"):
                name = f"{arm}_jaw_{jaw}_finger_link"
                visual = trimesh.load_mesh(root / "meshes" / f"{name}.obj")
                # Include face centroids so the test is not limited to extrema.
                samples = np.vstack((visual.vertices, visual.triangles_center))
                for region, low, high in (
                    ("middle", 0.033, 0.059), ("tip", 0.059, 0.10)
                ):
                    with self.subTest(link=name, region=region):
                        path = root / "meshes" / f"{name}_{region}_collision.obj"
                        mesh = trimesh.load_mesh(path)
                        planes = ConvexHull(mesh.vertices).equations
                        inside = (samples[:, 0] >= low) & (samples[:, 0] <= high)
                        points = samples[inside]
                        self.assertGreater(len(points), 0)
                        # Plane normals are unit length; tolerance is 0.1 um.
                        for chunk in np.array_split(points, 20):
                            distances = chunk @ planes[:, :3].T + planes[:, 3]
                            self.assertLessEqual(float(np.max(distances)), 1e-7)
                        sign = 1 if jaw == "left" else -1
                        inward_y = mesh.vertices[:, 1] * sign
                        self.assertGreater(
                            float(inward_y.min()), FINGER_CLOSING_TRAVEL_M
                        )

    def test_derived_joint_stops_match_calibrated_closure(self):
        """Check both jaws stop before their CAD envelopes overlap."""
        root = Path(__file__).resolve().parents[1] / "models/zerith_drake"
        urdf = ET.parse(root / "urdf/zerith_drake.urdf").getroot()
        for arm in ("left", "right"):
            for jaw, attribute, sign in (
                ("left", "lower", -1), ("right", "upper", 1)
            ):
                name = f"{arm}_jaw_{jaw}_finger_joint"
                limit = urdf.find(f"joint[@name='{name}']/limit")
                self.assertEqual(
                    float(limit.attrib[attribute]), sign * FINGER_CLOSING_TRAVEL_M
                )

    def test_empty_closure_is_stable_without_finger_penetration(self):
        """Close using actual limited actuation and retain a stable empty jaw."""
        config = dataclasses.replace(
            make_config(Path(__file__).resolve().parents[1]),
            episode_duration=4.0,
        )
        env = make_env(config)
        env.reset(seed=0)
        for _ in range(30):
            observation, _, terminated, truncated, _ = env.step(
                GripperAction(0.0)
            )
            self.assertFalse(terminated or truncated)
            self.assertTrue(np.all(np.isfinite(observation.robot.q)))
            for contact in observation.contacts:
                if "finger" in contact.body_a or "finger" in contact.body_b:
                    self.assertLess(contact.penetration_depth_m, 1e-4)
        self.assertLess(abs(observation.robot.gripper_width_m), 1e-4)


if __name__ == "__main__":
    unittest.main()
