"""Repository-external smoke test for the public camera example."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CLIENT = (
    REPOSITORY_ROOT
    / "examples"
    / "online_manipulation"
    / "camera_public_api_client.py"
)


class CameraExternalClientTest(unittest.TestCase):
    """Run the public client with no repository working-directory access."""

    def test_external_client_saves_rgb_and_metric_depth(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            temporary_root = Path(directory)
            output = temporary_root / "camera_output"
            environment = dict(os.environ)
            environment.update(
                PYTHONPATH=str(REPOSITORY_ROOT),
                MPLCONFIGDIR=str(temporary_root / "matplotlib"),
                MESA_SHADER_CACHE_DIR=str(temporary_root / "mesa"),
                PYTHONPYCACHEPREFIX=str(temporary_root / "pycache"),
            )
            result = subprocess.run(
                (
                    sys.executable,
                    str(CLIENT),
                    "--repository-root",
                    str(REPOSITORY_ROOT),
                    "--output-dir",
                    str(output),
                    "--seed",
                    "0",
                ),
                cwd=temporary_root,
                env=environment,
                check=True,
                capture_output=True,
                text=True,
            )
            payload = json.loads(result.stdout)

            self.assertEqual(payload["action_status"], "accepted")
            self.assertEqual(payload["rgb_shape"], [240, 320, 3])
            self.assertEqual(payload["rgb_dtype"], "uint8")
            self.assertEqual(payload["depth_shape"], [240, 320])
            self.assertEqual(payload["depth_dtype"], "float32")
            self.assertEqual(payload["depth_unit"], "m")
            rgb = np.asarray(Image.open(payload["rgb_path"]))
            depth = np.load(payload["depth_path"])
            self.assertEqual(rgb.shape, (240, 320, 3))
            self.assertEqual(depth.shape, (240, 320))
            self.assertGreater(int(rgb.max()), 0)
            self.assertGreater(int(np.count_nonzero(depth > 0.0)), 0)


if __name__ == "__main__":
    unittest.main()
