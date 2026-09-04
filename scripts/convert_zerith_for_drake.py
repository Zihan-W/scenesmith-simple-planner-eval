#!/usr/bin/env python3
"""Convert the upstream Zerith STL package into a Drake-compatible package."""

import argparse
import json
import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from pathlib import Path

import numpy as np
import trimesh

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
SOURCE_PACKAGE_RELATIVE_PATH = Path("models/Zerith_Model/ZERITH_H1_PRO_URDF")
SOURCE_PACKAGE_NAME = "ZR_H1PRO-1.2.00.H.V4.3_URDF_2025.12.02"
SOURCE_URDF_RELATIVE_PATH = Path(
    "urdf/ZR_H1PRO-1.2.00.H.V4.3_URDF_2025.12.02.urdf"
)
OUTPUT_PACKAGE_RELATIVE_PATH = Path("models/zerith_drake")
OUTPUT_PACKAGE_NAME = "zerith_drake"
OUTPUT_URDF_NAME = "zerith_drake.urdf"
EXPECTED_MESH_COUNT = 35
EXPECTED_SOURCE_MESH_REFERENCE_COUNT = 70
EXPECTED_OUTPUT_MESH_REFERENCE_COUNT = 57
EXPECTED_COLLISION_GEOMETRY_COUNT = 53
CONVERTER_VERSION = 3
DIPAN_COLLISION_BOXES = (
    {
        "name": "dipan_lower_base",
        "xyz": (0.0569, 0.0, -0.0660),
        "size": (0.6200, 0.4520, 0.1700),
    },
    {
        "name": "dipan_rear_mast",
        "xyz": (-0.0764, 0.0, 0.5551),
        "size": (0.2350, 0.2300, 1.1000),
    },
    {
        "name": "dipan_top_cap",
        "xyz": (-0.0764, 0.0, 1.1182),
        "size": (0.1880, 0.1960, 0.0350),
    },
)


def _arm_collision_proxies(
    side: str,
) -> dict[str, tuple[dict[str, object], ...]]:
    """Return primitive wrist and gripper proxies for one arm."""
    if side not in ("left", "right"):
        raise ValueError(f"Unexpected arm side: {side}")

    pitch_bracket_y = 0.0295 if side == "left" else -0.0295
    return {
        f"{side}_wrist_roll_link": (
            {
                "name": f"{side}_wrist_roll_body",
                "type": "cylinder",
                "xyz": (0.071, 0.0, -0.002),
                "rpy": (0.0, math.pi / 2.0, 0.0),
                "radius": 0.031,
                "length": 0.096,
            },
        ),
        f"{side}_wrist_yaw_link": (
            {
                "name": f"{side}_wrist_yaw_body",
                "type": "cylinder",
                "xyz": (0.0, 0.0, -0.00025),
                "rpy": (0.0, 0.0, 0.0),
                "radius": 0.025,
                "length": 0.065,
            },
        ),
        f"{side}_wrist_pitch_link": (
            {
                "name": f"{side}_wrist_pitch_bracket",
                "type": "box",
                "xyz": (0.005, pitch_bracket_y, 0.0),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.045, 0.009, 0.03),
            },
            {
                "name": f"{side}_wrist_pitch_body",
                "type": "box",
                "xyz": (0.05, 0.0, 0.005),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.034, 0.06, 0.05),
            },
            {
                "name": f"{side}_wrist_pitch_palm",
                "type": "box",
                "xyz": (0.099, 0.0, 0.005),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.018, 0.145, 0.04),
            },
        ),
        f"{side}_jaw_left_finger_link": (
            {
                "name": f"{side}_jaw_left_finger_base",
                "type": "box",
                "xyz": (0.024, 0.0545, 0.0),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.018, 0.027, 0.055),
            },
            {
                "name": f"{side}_jaw_left_finger_middle",
                "type": "box",
                "xyz": (0.0455, 0.047, -0.0045),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.025, 0.013, 0.035),
            },
            {
                "name": f"{side}_jaw_left_finger_tip",
                "type": "box",
                "xyz": (0.0765, 0.0425, -0.0015),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.035, 0.004, 0.014),
            },
        ),
        f"{side}_jaw_right_finger_link": (
            {
                "name": f"{side}_jaw_right_finger_base",
                "type": "box",
                "xyz": (0.024, -0.0545, 0.0),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.018, 0.027, 0.055),
            },
            {
                "name": f"{side}_jaw_right_finger_middle",
                "type": "box",
                "xyz": (0.0455, -0.047, -0.0045),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.025, 0.013, 0.035),
            },
            {
                "name": f"{side}_jaw_right_finger_tip",
                "type": "box",
                "xyz": (0.0765, -0.0425, -0.0015),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.035, 0.004, 0.014),
            },
        ),
        f"{side}_end_effector_link": (
            {
                "name": f"{side}_end_effector_center",
                "type": "box",
                "xyz": (-0.029, 0.0, 0.0),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.0015, 0.069, 0.008),
            },
            {
                "name": f"{side}_end_effector_left_mount",
                "type": "box",
                "xyz": (-0.029, 0.055, 0.0),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.0015, 0.029, 0.008),
            },
            {
                "name": f"{side}_end_effector_right_mount",
                "type": "box",
                "xyz": (-0.029, -0.055, 0.0),
                "rpy": (0.0, 0.0, 0.0),
                "size": (0.0015, 0.029, 0.008),
            },
        ),
    }


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Convert Zerith STL meshes and URDF references for Drake."
    )
    parser.add_argument(
        "--source-package",
        type=Path,
        default=REPOSITORY_ROOT / SOURCE_PACKAGE_RELATIVE_PATH,
        help="Upstream directory containing Zerith's urdf/ and meshes/.",
    )
    parser.add_argument(
        "--output-package",
        type=Path,
        default=REPOSITORY_ROOT / OUTPUT_PACKAGE_RELATIVE_PATH,
        help="Destination directory for the generated Drake package.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate the generated package without modifying files.",
    )
    return parser.parse_args()


def _convert_mesh(source_mesh: Path, output_mesh: Path) -> None:
    """Convert one STL mesh to OBJ and verify that its bounds are unchanged."""
    mesh = trimesh.load_mesh(source_mesh, process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one triangle mesh in {source_mesh}")

    output_mesh.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(output_mesh)

    converted_mesh = trimesh.load_mesh(output_mesh, process=False)
    if not isinstance(converted_mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one triangle mesh in {output_mesh}")
    if not np.allclose(mesh.bounds, converted_mesh.bounds, rtol=0.0, atol=1e-8):
        raise ValueError(
            f"Mesh bounds changed during conversion: {source_mesh} -> {output_mesh}"
        )


def _validate_converted_mesh(source_mesh: Path, output_mesh: Path) -> None:
    """Validate one generated OBJ against its source STL."""
    source = trimesh.load_mesh(source_mesh, process=False)
    converted = trimesh.load_mesh(output_mesh, process=False)
    if not isinstance(source, trimesh.Trimesh):
        raise TypeError(f"Expected one triangle mesh in {source_mesh}")
    if not isinstance(converted, trimesh.Trimesh):
        raise TypeError(f"Expected one triangle mesh in {output_mesh}")
    if not np.allclose(source.bounds, converted.bounds, rtol=0.0, atol=1e-8):
        raise ValueError(
            f"Mesh bounds differ: {source_mesh} -> {output_mesh}"
        )


def _rewrite_urdf(
    source_urdf: Path,
    output_urdf: Path,
    source_package_name: str,
) -> int:
    """Rewrite visual and collision STL references to generated OBJ files."""
    tree = ET.parse(source_urdf)
    package_prefix = f"package://{source_package_name}/meshes/"
    replacement_count = 0

    for mesh_element in tree.getroot().findall("./link/*/geometry/mesh"):
        mesh_uri = mesh_element.attrib["filename"]
        if not mesh_uri.startswith(package_prefix):
            raise ValueError(f"Unexpected mesh URI in Zerith URDF: {mesh_uri}")

        mesh_name = Path(mesh_uri.removeprefix(package_prefix))
        if mesh_name.suffix.lower() != ".stl":
            raise ValueError(f"Expected an STL reference, got: {mesh_uri}")
        mesh_element.set(
            "filename",
            f"package://{OUTPUT_PACKAGE_NAME}/meshes/{mesh_name.with_suffix('.obj')}",
        )
        replacement_count += 1

    if replacement_count != EXPECTED_SOURCE_MESH_REFERENCE_COUNT:
        raise ValueError(
            f"Expected {EXPECTED_SOURCE_MESH_REFERENCE_COUNT} mesh references, "
            f"rewrote {replacement_count}"
        )

    left_wrist_limit = tree.getroot().find(
        "./joint[@name='left_wrist_roll_joint']/limit"
    )
    if left_wrist_limit is None:
        raise ValueError("Missing limit for left_wrist_roll_joint")
    if (
        left_wrist_limit.attrib.get("effort") != "0"
        or left_wrist_limit.attrib.get("velocity") != "0"
    ):
        raise ValueError("Unexpected original limits for left_wrist_roll_joint")
    left_wrist_limit.set("effort", "9")
    left_wrist_limit.set("velocity", "16.747")
    _replace_dipan_collision(tree.getroot())
    _replace_wrist_and_gripper_collisions(tree.getroot())

    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)
    return replacement_count


def _replace_dipan_collision(root: ET.Element) -> None:
    """Replace dipan_link's whole-mesh convex hull with three box proxies."""
    dipan_link = root.find("./link[@name='dipan_link']")
    if dipan_link is None:
        raise ValueError("Missing dipan_link")

    collisions = dipan_link.findall("collision")
    if len(collisions) != 1:
        raise ValueError(
            f"Expected one source dipan_link collision, found {len(collisions)}"
        )
    dipan_link.remove(collisions[0])

    for box_spec in DIPAN_COLLISION_BOXES:
        collision = ET.SubElement(
            dipan_link,
            "collision",
            {"name": box_spec["name"]},
        )
        ET.SubElement(
            collision,
            "origin",
            {
                "xyz": " ".join(str(value) for value in box_spec["xyz"]),
                "rpy": "0 0 0",
            },
        )
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(
            geometry,
            "box",
            {"size": " ".join(str(value) for value in box_spec["size"])},
        )


def _replace_wrist_and_gripper_collisions(root: ET.Element) -> None:
    """Replace whole-mesh wrist and gripper hulls with primitives."""
    for side in ("left", "right"):
        for link_name, proxy_specs in _arm_collision_proxies(side).items():
            link = root.find(f"./link[@name='{link_name}']")
            if link is None:
                raise ValueError(f"Missing {link_name}")

            collisions = link.findall("collision")
            if len(collisions) != 1:
                raise ValueError(
                    f"Expected one source {link_name} collision, "
                    f"found {len(collisions)}"
                )
            link.remove(collisions[0])

            for proxy_spec in proxy_specs:
                _append_collision_proxy(link, proxy_spec)


def _append_collision_proxy(
    link: ET.Element,
    proxy_spec: dict[str, object],
) -> None:
    """Append one box or cylinder collision proxy to a link."""
    collision = ET.SubElement(
        link,
        "collision",
        {"name": str(proxy_spec["name"])},
    )
    ET.SubElement(
        collision,
        "origin",
        {
            "xyz": " ".join(str(value) for value in proxy_spec["xyz"]),
            "rpy": " ".join(str(value) for value in proxy_spec["rpy"]),
        },
    )
    geometry = ET.SubElement(collision, "geometry")
    geometry_type = proxy_spec["type"]
    if geometry_type == "box":
        ET.SubElement(
            geometry,
            "box",
            {
                "size": " ".join(
                    str(value) for value in proxy_spec["size"]
                )
            },
        )
    elif geometry_type == "cylinder":
        ET.SubElement(
            geometry,
            "cylinder",
            {
                "radius": str(proxy_spec["radius"]),
                "length": str(proxy_spec["length"]),
            },
        )
    else:
        raise ValueError(f"Unsupported collision proxy type: {geometry_type}")


def _package_xml_contents() -> str:
    """Return the deterministic ROS package metadata."""
    return """<?xml version="1.0"?>
<package format="2">
  <name>zerith_drake</name>
  <version>0.1.0</version>
  <description>Drake-compatible Zerith H1 Pro model.</description>
  <maintainer email="noreply@example.com">SceneSmith</maintainer>
  <license>See upstream Zerith_Model repository</license>
</package>
"""


def _get_source_commit(source_package: Path) -> str:
    """Return the Git commit of the upstream Zerith submodule."""
    result = subprocess.run(
        ["git", "-C", str(source_package.parent), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _manifest_contents(source_commit: str) -> str:
    """Return deterministic conversion metadata."""
    manifest = {
        "converter_version": CONVERTER_VERSION,
        "dipan_collision_proxy": "three_boxes",
        "expected_collision_geometry_count": EXPECTED_COLLISION_GEOMETRY_COUNT,
        "mesh_count": EXPECTED_MESH_COUNT,
        "output_mesh_reference_count": EXPECTED_OUTPUT_MESH_REFERENCE_COUNT,
        "output_format": "obj",
        "source_mesh_reference_count": EXPECTED_SOURCE_MESH_REFERENCE_COUNT,
        "source_commit": source_commit,
        "source_repository": "https://github.com/inFpZero/Zerith_Model.git",
        "wrist_gripper_collision_proxy": "primitive_boxes_and_cylinders",
    }
    return json.dumps(manifest, indent=2, sort_keys=True) + "\n"


def _write_metadata(output_package: Path, source_commit: str) -> None:
    """Write package.xml and the conversion manifest."""
    (output_package / "package.xml").write_text(
        _package_xml_contents(),
        encoding="utf-8",
    )
    (output_package / "conversion_manifest.json").write_text(
        _manifest_contents(source_commit),
        encoding="utf-8",
    )


def _check_generated_package(
    source_urdf: Path,
    source_meshes: list[Path],
    output_package: Path,
    source_commit: str,
) -> None:
    """Check metadata, derived URDF, OBJ count, and mesh bounds."""
    output_urdf = output_package / "urdf" / OUTPUT_URDF_NAME
    package_xml = output_package / "package.xml"
    manifest = output_package / "conversion_manifest.json"

    with tempfile.TemporaryDirectory(prefix="zerith_drake_check_") as temp_dir:
        expected_urdf = Path(temp_dir) / OUTPUT_URDF_NAME
        replacement_count = _rewrite_urdf(
            source_urdf,
            expected_urdf,
            SOURCE_PACKAGE_NAME,
        )
        if replacement_count != EXPECTED_SOURCE_MESH_REFERENCE_COUNT:
            raise ValueError(f"Unexpected reference count: {replacement_count}")
        if output_urdf.read_bytes() != expected_urdf.read_bytes():
            raise ValueError(f"Derived URDF has drifted: {output_urdf}")

    output_root = ET.parse(output_urdf).getroot()
    output_mesh_references = output_root.findall("./link/*/geometry/mesh")
    if len(output_mesh_references) != EXPECTED_OUTPUT_MESH_REFERENCE_COUNT:
        raise ValueError(
            f"Expected {EXPECTED_OUTPUT_MESH_REFERENCE_COUNT} output mesh "
            f"references, found {len(output_mesh_references)}"
        )

    output_collisions = output_root.findall("./link/collision")
    if len(output_collisions) != EXPECTED_COLLISION_GEOMETRY_COUNT:
        raise ValueError(
            f"Expected {EXPECTED_COLLISION_GEOMETRY_COUNT} collision "
            f"geometries, found {len(output_collisions)}"
        )
    for side in ("left", "right"):
        for link_name, proxy_specs in _arm_collision_proxies(side).items():
            link = output_root.find(f"./link[@name='{link_name}']")
            if link is None:
                raise ValueError(f"Missing generated {link_name}")
            collisions = link.findall("collision")
            expected_names = [str(spec["name"]) for spec in proxy_specs]
            actual_names = [collision.attrib.get("name") for collision in collisions]
            if actual_names != expected_names:
                raise ValueError(
                    f"Unexpected collision proxies for {link_name}: "
                    f"{actual_names}"
                )
            if any(
                collision.find("./geometry/mesh") is not None
                for collision in collisions
            ):
                raise ValueError(
                    f"Generated {link_name} still has a mesh collision"
                )

    if package_xml.read_text(encoding="utf-8") != _package_xml_contents():
        raise ValueError(f"package.xml has drifted: {package_xml}")
    if manifest.read_text(encoding="utf-8") != _manifest_contents(source_commit):
        raise ValueError(f"Conversion manifest has drifted: {manifest}")

    output_mesh_dir = output_package / "meshes"
    output_meshes = sorted(output_mesh_dir.glob("*.obj"))
    if len(output_meshes) != EXPECTED_MESH_COUNT:
        raise ValueError(
            f"Expected {EXPECTED_MESH_COUNT} OBJ meshes, found {len(output_meshes)}"
        )
    for source_mesh in source_meshes:
        output_mesh = output_mesh_dir / source_mesh.with_suffix(".obj").name
        if not output_mesh.is_file():
            raise FileNotFoundError(f"Missing converted mesh: {output_mesh}")
        _validate_converted_mesh(source_mesh, output_mesh)

    print(f"Check passed: {output_package}")
    print(f"Source commit: {source_commit}")
    print(f"OBJ meshes: {len(output_meshes)}")
    print(f"Source URDF mesh references: {EXPECTED_SOURCE_MESH_REFERENCE_COUNT}")
    print(f"Generated URDF mesh references: {EXPECTED_OUTPUT_MESH_REFERENCE_COUNT}")
    print(f"Collision geometries: {EXPECTED_COLLISION_GEOMETRY_COUNT}")


def main() -> None:
    """Generate the Drake-compatible Zerith package."""
    args = _parse_args()
    source_package = args.source_package.resolve()
    output_package = args.output_package.resolve()
    source_urdf = source_package / SOURCE_URDF_RELATIVE_PATH
    source_mesh_dir = source_package / "meshes"

    if not source_urdf.is_file():
        raise FileNotFoundError(f"Zerith URDF does not exist: {source_urdf}")

    source_meshes = sorted(source_mesh_dir.glob("*.STL"))
    if len(source_meshes) != EXPECTED_MESH_COUNT:
        raise ValueError(
            f"Expected {EXPECTED_MESH_COUNT} STL meshes, found {len(source_meshes)}"
        )

    source_commit = _get_source_commit(source_package)
    if args.check:
        _check_generated_package(
            source_urdf,
            source_meshes,
            output_package,
            source_commit,
        )
        return

    output_package.mkdir(parents=True, exist_ok=True)
    output_mesh_dir = output_package / "meshes"

    for index, source_mesh in enumerate(source_meshes, start=1):
        output_mesh = output_mesh_dir / source_mesh.with_suffix(".obj").name
        print(f"[{index:02d}/{len(source_meshes)}] {source_mesh.name}")
        _convert_mesh(source_mesh, output_mesh)

    output_urdf = output_package / "urdf" / OUTPUT_URDF_NAME
    replacement_count = _rewrite_urdf(
        source_urdf,
        output_urdf,
        SOURCE_PACKAGE_NAME,
    )
    _write_metadata(output_package, source_commit)
    _check_generated_package(
        source_urdf,
        source_meshes,
        output_package,
        source_commit,
    )

    print(f"Generated package: {output_package}")
    print(f"Source commit: {source_commit}")
    print(f"Converted meshes: {len(source_meshes)}")
    print(f"Rewritten URDF references: {replacement_count}")
    print(f"Generated URDF: {output_urdf}")


if __name__ == "__main__":
    main()
