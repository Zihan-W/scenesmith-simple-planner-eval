"""Tests for selective Drake directives pose write-back."""

import tempfile
import unittest
from pathlib import Path

from pydrake.all import LoadModelDirectives, RigidTransform, RollPitchYaw

from src.online_manipulation import ObservedBodySpec, write_updated_dmd


_SOURCE_DMD = """directives:
- add_model:
    name: movable
    file: package://test/movable.sdf
    default_free_body_pose:
      base_link:
        base_frame: room_frame
        translation: [1.0, 2.0, 3.0]
        rotation: !AngleAxis
          angle_deg: 0.0
          axis: [0.0, 0.0, 1.0]
- add_model:
    name: untouched
    file: package://test/untouched.sdf
    default_free_body_pose:
      base_link:
        base_frame: room_frame
        translation: [4.0, 5.0, 6.0]
        rotation: !Rpy
          deg: [1.0, 2.0, 3.0]
"""


class _FakePlant:
    """Expose the small pose-query surface required by the finalizer."""

    def __init__(self) -> None:
        self.world_from_base = RigidTransform([10.0, 20.0, 30.0])
        self.world_from_body = self.world_from_base @ RigidTransform(
            RollPitchYaw(0.1, -0.2, 0.3),
            [0.25, -0.5, 0.75],
        )

    def GetModelInstanceByName(self, name):
        self.model_name = name
        return 7

    def GetBodyByName(self, name, model_instance):
        self.body_name = name
        self.model_instance = model_instance
        return "body"

    def EvalBodyPoseInWorld(self, context, body):
        del context, body
        return self.world_from_body

    def GetFrameByName(self, name):
        self.frame_name = name
        return "base_frame"

    def world_frame(self):
        return "world"

    def CalcRelativeTransform(self, context, frame_a, frame_b):
        del context, frame_a, frame_b
        return self.world_from_base


class DmdFinalizerTest(unittest.TestCase):
    """Validate explicit selection, frame conversion, and source safety."""

    def test_cached_block_translation_reloads_in_drake(self):
        """Scene preparation emits block lists, including indentless YAML lists."""
        for sequence_indent in (8, 10):
            with self.subTest(sequence_indent=sequence_indent):
                text = _SOURCE_DMD.replace(
                    "translation: [1.0, 2.0, 3.0]",
                    "translation:\n" + "\n".join(
                        " " * sequence_indent + "- " + value
                        for value in ("1.0", "2.0", "3.0")
                    ),
                )
                with tempfile.TemporaryDirectory() as directory:
                    source = Path(directory) / "source.dmd.yaml"
                    output = Path(directory) / "final.dmd.yaml"
                    source.write_text(text)
                    write_updated_dmd(
                        input_path=source, output_path=output, plant=_FakePlant(),
                        plant_context=object(), body_specs=(
                            ObservedBodySpec("target", "movable", "base_link", write_back=True),
                        ),
                    )
                    directives = LoadModelDirectives(str(output))
                    self.assertEqual(len(directives.directives), 2)
                    self.assertEqual(source.read_text(), text)

    def test_only_selected_body_is_rewritten_in_original_base_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "input.dmd.yaml"
            output = root / "output.dmd.yaml"
            source.write_text(_SOURCE_DMD, encoding="utf-8")
            plant = _FakePlant()
            updated = write_updated_dmd(
                input_path=source,
                output_path=output,
                plant=plant,
                plant_context=object(),
                body_specs=(
                    ObservedBodySpec(
                        "target",
                        "movable",
                        "base_link",
                        write_back=True,
                    ),
                    ObservedBodySpec(
                        "reference",
                        "untouched",
                        "base_link",
                    ),
                ),
            )

            result = output.read_text(encoding="utf-8")
            self.assertEqual(updated, ("target",))
            self.assertEqual(source.read_text(encoding="utf-8"), _SOURCE_DMD)
            self.assertIn("translation: [0.25, -0.5, 0.75]", result)
            self.assertIn("deg: [5.729577951308", result)
            untouched_start = _SOURCE_DMD.index(
                "- add_model:\n    name: untouched"
            )
            untouched_block = _SOURCE_DMD[untouched_start:]
            self.assertTrue(result.endswith(untouched_block))
            self.assertEqual(plant.model_name, "movable")
            self.assertEqual(plant.body_name, "base_link")
            self.assertEqual(plant.frame_name, "room_frame")

    def test_rejects_overwriting_source(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "scene.dmd.yaml"
            source.write_text(_SOURCE_DMD, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "distinct output"):
                write_updated_dmd(
                    input_path=source,
                    output_path=source,
                    plant=_FakePlant(),
                    plant_context=object(),
                    body_specs=(
                        ObservedBodySpec(
                            "target",
                            "movable",
                            "base_link",
                            write_back=True,
                        ),
                    ),
                )

    def test_requires_explicit_write_back_selection(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "input.dmd.yaml"
            output = Path(directory) / "output.dmd.yaml"
            source.write_text(_SOURCE_DMD, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "write_back=True"):
                write_updated_dmd(
                    input_path=source,
                    output_path=output,
                    plant=_FakePlant(),
                    plant_context=object(),
                    body_specs=(
                        ObservedBodySpec(
                            "target",
                            "movable",
                            "base_link",
                        ),
                    ),
                )


if __name__ == "__main__":
    unittest.main()
