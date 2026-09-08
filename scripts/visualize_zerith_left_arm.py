#!/usr/bin/env python3
"""Visualize Zerith in a SceneSmith scene and inspect its left arm.

This script is a kinematic model-integration check. It welds Zerith's
``dipan_link`` to the world and exposes Meshcat sliders for the vertical rail,
seven left-arm joints, and two left-gripper joints. The generated Drake URDF
includes collision geometry, but this viewer does not run a dynamics
simulation.
"""

import argparse
import json
import sys
import time
import xml.etree.ElementTree as ET

from pathlib import Path

import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    LoadModelDirectives,
    Meshcat,
    MeshcatVisualizer,
    Parser,
    ProcessModelDirectives,
    RigidTransform,
    RollPitchYaw,
)

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent


_INITIAL = json.loads((REPOSITORY_ROOT / "experiments/profiles.json").read_text())["initial_state"]["picklift"]
ROBOT_BASE_XYZ_METERS = tuple(_INITIAL["robot_xyz"])
ROBOT_BASE_YAW_DEG = _INITIAL["robot_yaw_deg"]

ZERITH_PACKAGE_NAME = "zerith_drake"
ZERITH_MODEL_RELATIVE_PATH = Path("models/zerith_drake")
ZERITH_URDF_RELATIVE_PATH = Path("urdf/zerith_drake.urdf")

RAIL_JOINTS = ("daogui_joint",)
LEFT_ARM_JOINTS = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
)
LEFT_GRIPPER_JOINTS = (
    "left_jaw_left_finger_joint",
    "left_jaw_right_finger_joint",
)


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Visualize Zerith and inspect its vertical rail and left arm "
            "with kinematic sliders."
        )
    )
    parser.add_argument(
        "scene_dmd",
        type=Path,
        help="Path to house_furniture_welded.dmd.yaml or another scene DMD.",
    )
    parser.add_argument(
        "--scene-package-xml",
        type=Path,
        help="Scene package.xml; inferred from scene_dmd when omitted.",
    )
    parser.add_argument(
        "--additional-package-xml",
        action="append",
        default=[],
        type=Path,
        help="Additional package.xml to register; may be repeated.",
    )
    parser.add_argument(
        "--robot-model-dir",
        type=Path,
        default=REPOSITORY_ROOT / ZERITH_MODEL_RELATIVE_PATH,
        help="Generated Drake package containing Zerith's urdf/ and meshes/.",
    )
    parser.add_argument(
        "--robot-xyz",
        type=float,
        nargs=3,
        default=ROBOT_BASE_XYZ_METERS,
        metavar=("X", "Y", "Z"),
        help=(
            "World position of dipan_link in meters "
            f"(default: {ROBOT_BASE_XYZ_METERS})."
        ),
    )
    parser.add_argument(
        "--robot-yaw-deg",
        type=float,
        default=ROBOT_BASE_YAW_DEG,
        help=(
            "World yaw of dipan_link in degrees "
            "(default: facing the coffee table)."
        ),
    )
    parser.add_argument(
        "--rail-position",
        type=float,
        default=0.0,
        help="Initial daogui_joint position in meters.",
    )
    parser.add_argument(
        "--q-left",
        type=float,
        nargs=7,
        default=(0.0,) * 7,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        help="Initial seven-joint left-arm posture in radians.",
    )
    parser.add_argument(
        "--configuration-json",
        type=Path,
        help=(
            "JSON containing a saved q_pick_home or q_pregrasp vector. "
            "When supplied, this overrides --q-left."
        ),
    )
    parser.add_argument(
        "--configuration-key",
        choices=("q_pick_home", "q_pregrasp"),
        default="q_pregrasp",
        help="Seven-joint vector to load from --configuration-json.",
    )
    parser.add_argument(
        "--meshcat-port",
        type=int,
        help="Meshcat port; Drake selects an available port when omitted.",
    )
    return parser.parse_args()


def _load_q_left(args: argparse.Namespace) -> np.ndarray:
    """Return the requested initial seven-joint left-arm posture."""
    if args.configuration_json is None:
        return np.asarray(args.q_left, dtype=float)
    configuration_json = args.configuration_json.resolve()
    payload = json.loads(configuration_json.read_text(encoding="utf-8"))
    q_left = np.asarray(payload[args.configuration_key], dtype=float)
    if q_left.shape != (7,) or not np.all(np.isfinite(q_left)):
        raise ValueError(
            f"{configuration_json} contains an invalid "
            f"{args.configuration_key}"
        )
    return q_left


def _find_package_xml(scene_dmd: Path) -> Path:
    """Find the nearest package.xml containing a scene DMD."""
    for directory in scene_dmd.parents:
        package_xml = directory / "package.xml"
        if package_xml.is_file():
            return package_xml
    raise FileNotFoundError(
        f"Could not find package.xml above {scene_dmd}. "
        "Pass --scene-package-xml explicitly."
    )


def _register_package_xml(parser: Parser, package_xml: Path) -> None:
    """Register a ROS-style package.xml in a Drake parser."""
    root = ET.parse(package_xml).getroot()
    name = root.findtext("name")
    if name is None:
        raise ValueError(f"Missing <name> in {package_xml}")
    parser.package_map().Add(name.strip(), str(package_xml.parent))


def _add_joint_sliders(
    meshcat: Meshcat,
    plant,
    plant_context,
    zerith,
) -> list[str]:
    """Add Meshcat sliders for the rail, left arm, and gripper joints."""
    positions = plant.GetPositions(plant_context)
    position_lower_limits = plant.GetPositionLowerLimits()
    position_upper_limits = plant.GetPositionUpperLimits()
    slider_names = []

    for joint_name in RAIL_JOINTS + LEFT_ARM_JOINTS + LEFT_GRIPPER_JOINTS:
        joint = plant.GetJointByName(joint_name, zerith)
        if joint.num_positions() != 1:
            raise ValueError(f"Expected one position for joint {joint_name}")

        position_index = joint.position_start()
        step = 0.001 if joint_name in LEFT_GRIPPER_JOINTS else 0.01
        meshcat.AddSlider(
            joint_name,
            min=position_lower_limits[position_index],
            max=position_upper_limits[position_index],
            step=step,
            value=positions[position_index],
        )
        slider_names.append(joint_name)

    return slider_names


def _run_slider_loop(meshcat: Meshcat, diagram, context, plant, zerith) -> None:
    """Update the plant configuration from the left-arm Meshcat sliders."""
    plant_context = plant.GetMyMutableContextFromRoot(context)
    slider_names = _add_joint_sliders(meshcat, plant, plant_context, zerith)
    meshcat.AddButton("Stop", "Escape")

    try:
        while meshcat.GetButtonClicks("Stop") < 1:
            positions = plant.GetPositions(plant_context).copy()
            for joint_name in slider_names:
                joint = plant.GetJointByName(joint_name, zerith)
                positions[joint.position_start()] = meshcat.GetSliderValue(joint_name)
            plant.SetPositions(plant_context, positions)
            diagram.ForcedPublish(context)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        for slider_name in slider_names:
            meshcat.DeleteSlider(slider_name)
        meshcat.DeleteButton("Stop")


def main() -> None:
    """Load the scene and Zerith model, then run the slider interface."""
    args = _parse_args()
    scene_dmd = args.scene_dmd.resolve()
    package_xml = (
        args.scene_package_xml.resolve()
        if args.scene_package_xml is not None
        else _find_package_xml(scene_dmd)
    )
    robot_model_dir = args.robot_model_dir.resolve()
    robot_urdf = robot_model_dir / ZERITH_URDF_RELATIVE_PATH
    q_left = _load_q_left(args)

    if not scene_dmd.is_file():
        raise FileNotFoundError(f"Scene DMD does not exist: {scene_dmd}")
    if not package_xml.is_file():
        raise FileNotFoundError(f"Scene package.xml does not exist: {package_xml}")
    additional_package_xmls = tuple(
        path.resolve() for path in args.additional_package_xml
    )
    for additional_package_xml in additional_package_xmls:
        if not additional_package_xml.is_file():
            raise FileNotFoundError(
                "Additional package.xml does not exist: "
                f"{additional_package_xml}"
            )
    if not robot_urdf.is_file():
        raise FileNotFoundError(
            f"Zerith URDF does not exist: {robot_urdf}\n"
            "Run `python scripts/convert_zerith_for_drake.py`."
        )

    meshcat = Meshcat(args.meshcat_port)
    meshcat.Delete()
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.SetAutoRenaming(True)
    _register_package_xml(parser, package_xml)
    for additional_package_xml in additional_package_xmls:
        _register_package_xml(parser, additional_package_xml)
    parser.package_map().Add(ZERITH_PACKAGE_NAME, str(robot_model_dir))

    directives = LoadModelDirectives(str(scene_dmd))
    ProcessModelDirectives(directives, parser)
    model_instances = parser.AddModels(str(robot_urdf))
    if len(model_instances) != 1:
        raise RuntimeError(f"Expected one Zerith model, got {len(model_instances)}")

    zerith = model_instances[0]
    collision_geometry_count = sum(
        len(plant.GetCollisionGeometriesForBody(plant.get_body(body_index)))
        for body_index in plant.GetBodyIndices(zerith)
    )
    if collision_geometry_count == 0:
        raise ValueError(
            "Zerith must provide collision geometry for calibration"
        )

    base_frame = plant.GetFrameByName("dipan_link", zerith)
    world_from_base = RigidTransform(
        RollPitchYaw(0.0, 0.0, np.deg2rad(args.robot_yaw_deg)),
        args.robot_xyz,
    )
    plant.WeldFrames(plant.world_frame(), base_frame, world_from_base)

    plant.Finalize()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    positions = plant.GetPositions(plant_context).copy()
    rail = plant.GetJointByName(RAIL_JOINTS[0], zerith)
    if not (
        rail.position_lower_limits()[0]
        <= args.rail_position
        <= rail.position_upper_limits()[0]
    ):
        raise ValueError("rail-position is outside the URDF joint limits")
    positions[rail.position_start()] = args.rail_position
    for joint_name, value in zip(LEFT_ARM_JOINTS, q_left, strict=True):
        joint = plant.GetJointByName(joint_name, zerith)
        if not (
            joint.position_lower_limits()[0]
            <= value
            <= joint.position_upper_limits()[0]
        ):
            raise ValueError(f"{joint_name} value is outside its joint limits")
        positions[joint.position_start()] = value
    plant.SetPositions(plant_context, positions)
    diagram.ForcedPublish(context)

    print(f"Meshcat URL: {meshcat.web_url()}")
    print(f"Scene: {scene_dmd}")
    print(f"Zerith URDF: {robot_urdf}")
    print(f"Zerith base frame: dipan_link at XYZ {tuple(args.robot_xyz)}")
    print(f"Zerith yaw: {args.robot_yaw_deg} degrees")
    if args.configuration_json is not None:
        print(
            "Loaded posture: "
            f"{args.configuration_key} from "
            f"{args.configuration_json.resolve()}"
        )
    print("Active end effector: left_end_effector_link")
    print(
        "Active joints: "
        f"{', '.join(RAIL_JOINTS + LEFT_ARM_JOINTS + LEFT_GRIPPER_JOINTS)}"
    )
    print(
        "Rail calibration status: "
        f"range={rail.position_lower_limits()[0]:.3f} to "
        f"{rail.position_upper_limits()[0]:.3f} m; upstream effort and "
        "velocity limits are both zero and remain uncalibrated"
    )
    print(f"Zerith collision geometries: {collision_geometry_count}")
    print("Move the Meshcat sliders; press Escape or Ctrl+C to exit.")

    _run_slider_loop(meshcat, diagram, context, plant, zerith)


if __name__ == "__main__":
    main()
