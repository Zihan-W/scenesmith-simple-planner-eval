"""Numerical projection tests for all three Zerith simulation cameras."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.calibration.validate_zerith_camera_geometry import validate_calibration


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class ZerithCameraCalibrationTest(unittest.TestCase):
    """Validate explicit mount geometry using rendered calibration targets."""

    @classmethod
    def setUpClass(cls) -> None:
        """Render the three self-contained calibration scenes once."""
        cls._temporary_directory = tempfile.TemporaryDirectory()
        cls.output_directory = Path(cls._temporary_directory.name)
        cls.result = validate_calibration(
            REPOSITORY_ROOT,
            cls.output_directory,
        )

    @classmethod
    def tearDownClass(cls) -> None:
        """Remove generated calibration artifacts."""
        cls._temporary_directory.cleanup()

    def test_all_camera_projection_checks_pass(self) -> None:
        self.assertTrue(self.result["passed"])
        self.assertEqual(
            set(self.result["cameras"]),
            {"head_camera", "left_wrist_camera", "right_wrist_camera"},
        )

    def test_center_right_and_down_targets_follow_optical_axes(self) -> None:
        for camera in self.result["cameras"].values():
            targets = camera["targets"]
            center = targets["center_target"]["actual_label_bbox"][
                "center_uv"
            ]
            right = targets["right_target"]["actual_label_bbox"][
                "center_uv"
            ]
            down = targets["down_target"]["actual_label_bbox"][
                "center_uv"
            ]
            intrinsics = camera["intrinsics"]
            self.assertLess(
                abs(center[0] - intrinsics["center_x_px"]),
                1.0,
            )
            self.assertLess(
                abs(center[1] - intrinsics["center_y_px"]),
                1.0,
            )
            self.assertGreater(right[0], center[0] + 30.0)
            self.assertLess(abs(right[1] - center[1]), 1.0)
            self.assertGreater(down[1], center[1] + 20.0)
            self.assertLess(abs(down[0] - center[0]), 1.0)

    def test_target_depth_and_modalities_are_aligned(self) -> None:
        for camera in self.result["cameras"].values():
            for target in camera["targets"].values():
                self.assertLessEqual(target["pixel_error_px"], 1.0)
                self.assertLessEqual(target["depth_error_m"], 1e-3)
                self.assertTrue(target["rgb_depth_label_aligned"])
                self.assertGreater(
                    target["actual_label_bbox"]["pixel_count"],
                    100,
                )

    def test_mount_to_optical_is_explicit_and_composition_is_nontrivial(self):
        for camera in self.result["cameras"].values():
            extrinsics = camera["extrinsics"]
            np.testing.assert_allclose(
                extrinsics["X_mount_camera_optical"],
                np.eye(4),
                atol=1e-12,
            )
            self.assertFalse(
                np.allclose(
                    extrinsics["X_parent_camera_optical"],
                    np.eye(4),
                )
            )

    def test_calibration_outputs_all_visualizations(self) -> None:
        self.assertTrue(
            (self.output_directory / "calibration_metrics.json").is_file()
        )
        for camera_name in self.result["cameras"]:
            for suffix in (
                "rgb.png",
                "depth_m.npy",
                "depth_color.png",
                "label_color.png",
            ):
                self.assertTrue(
                    (self.output_directory / f"{camera_name}_{suffix}").is_file()
                )


if __name__ == "__main__":
    unittest.main()
