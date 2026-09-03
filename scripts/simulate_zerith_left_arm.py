#!/usr/bin/env python3
"""Run contact-aware PD control of Zerith's left arm in a SceneSmith scene."""

import argparse
import time
import xml.etree.ElementTree as ET

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    LoadModelDirectives,
    Meshcat,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Parser,
    ProcessModelDirectives,
    RigidTransform,
    Role,
    RollPitchYaw,
    Simulator,
)

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
ZERITH_PACKAGE_NAME = "zerith_drake"
ZERITH_MODEL_RELATIVE_PATH = Path("models/zerith_drake")
ZERITH_URDF_RELATIVE_PATH = Path("urdf/zerith_drake.urdf")


@dataclass(frozen=True)
class JointControllerConfig:
    """Parameters for one independently actuated joint."""

    name: str
    kp: float
    kd: float
    effort_limit: float
    slider_step: float
    initial_position: float = 0.0


@dataclass(frozen=True)
class Penetration:
    """One active penetration pair involving a Zerith collision geometry."""

    depth: float
    frame_a: str
    frame_b: str


JOINT_CONFIGS = (
    JointControllerConfig("left_shoulder_pitch_joint", 80.0, 8.0, 36.0, 0.01),
    JointControllerConfig("left_shoulder_roll_joint", 80.0, 8.0, 36.0, 0.01),
    JointControllerConfig("left_shoulder_yaw_joint", 60.0, 6.0, 27.0, 0.01),
    JointControllerConfig("left_elbow_joint", 60.0, 6.0, 27.0, 0.01),
    JointControllerConfig("left_wrist_roll_joint", 20.0, 2.0, 9.0, 0.01),
    JointControllerConfig("left_wrist_yaw_joint", 20.0, 2.0, 9.0, 0.01),
    JointControllerConfig("left_wrist_pitch_joint", 20.0, 2.0, 9.0, 0.01),
    JointControllerConfig(
        "left_jaw_left_finger_joint",
        500.0,
        10.0,
        25.0,
        0.001,
        -0.04,
    ),
    JointControllerConfig(
        "left_jaw_right_finger_joint",
        500.0,
        10.0,
        25.0,
        0.001,
        0.04,
    ),
)


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Simulate Zerith's left arm with finite-torque PD control and "
            "Drake contact dynamics."
        )
    )
    parser.add_argument(
        "scene_dmd",
        type=Path,
        help="Path to house_furniture_welded.dmd.yaml.",
    )
    parser.add_argument(
        "--scene-package-xml",
        type=Path,
        help="Scene package.xml; inferred from scene_dmd when omitted.",
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
        default=(4.177360808362734, 0.60, 0.1815),
        metavar=("X", "Y", "Z"),
        help="World position of the welded dipan_link in meters.",
    )
    parser.add_argument(
        "--robot-yaw-deg",
        type=float,
        default=90.0,
        help="World yaw of dipan_link in degrees.",
    )
    parser.add_argument(
        "--time-step",
        type=float,
        default=0.001,
        help="MultibodyPlant discrete time step in seconds.",
    )
    parser.add_argument(
        "--control-period",
        type=float,
        default=0.005,
        help="PD command update period in seconds.",
    )
    parser.add_argument(
        "--realtime-rate",
        type=float,
        default=1.0,
        help="Target simulator realtime rate.",
    )
    parser.add_argument(
        "--meshcat-port",
        type=int,
        help="Meshcat port; Drake selects an available port when omitted.",
    )
    parser.add_argument(
        "--initial-penetration-limit",
        type=float,
        help=(
            "Fail before simulation if an active robot penetration exceeds "
            "this depth in meters."
        ),
    )
    return parser.parse_args()


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


def _add_sliders(meshcat: Meshcat, plant, zerith, plant_context) -> None:
    """Add desired-position sliders for each controlled joint."""
    positions = plant.GetPositions(plant_context)
    lower_limits = plant.GetPositionLowerLimits()
    upper_limits = plant.GetPositionUpperLimits()

    for config in JOINT_CONFIGS:
        joint = plant.GetJointByName(config.name, zerith)
        position_index = joint.position_start()
        meshcat.AddSlider(
            config.name,
            min=lower_limits[position_index],
            max=upper_limits[position_index],
            step=config.slider_step,
            value=positions[position_index],
        )
    meshcat.AddButton("Stop simulation", "Escape")


def _set_initial_positions(plant, zerith, plant_context) -> None:
    """Set the initial controlled-joint positions before simulation starts."""
    positions = plant.GetPositions(plant_context).copy()
    for config in JOINT_CONFIGS:
        joint = plant.GetJointByName(config.name, zerith)
        positions[joint.position_start()] = config.initial_position
    plant.SetPositions(plant_context, positions)


def _lock_uncontrolled_joints(plant, zerith, plant_context) -> list[str]:
    """Lock every movable Zerith joint not controlled by this experiment."""
    controlled_names = {config.name for config in JOINT_CONFIGS}
    locked_names = []
    for joint_index in plant.GetJointIndices(zerith):
        joint = plant.get_joint(joint_index)
        if joint.num_velocities() == 0 or joint.name() in controlled_names:
            continue
        joint.Lock(plant_context)
        locked_names.append(joint.name())
    return locked_names


def _calc_pd_actuation(meshcat: Meshcat, plant, zerith, plant_context) -> np.ndarray:
    """Calculate a saturated full-plant actuation vector from slider targets."""
    positions = plant.GetPositions(plant_context)
    velocities = plant.GetVelocities(plant_context)
    actuation = np.zeros(plant.num_actuated_dofs())

    for config in JOINT_CONFIGS:
        joint = plant.GetJointByName(config.name, zerith)
        actuator = plant.GetJointActuatorByName(f"{config.name}_actuator", zerith)
        desired_position = meshcat.GetSliderValue(config.name)
        position = positions[joint.position_start()]
        velocity = velocities[joint.velocity_start()]
        effort = config.kp * (desired_position - position) - config.kd * velocity
        actuation[actuator.input_start()] = np.clip(
            effort,
            -config.effort_limit,
            config.effort_limit,
        )

    return actuation


def _find_robot_penetrations(
    plant,
    scene_graph,
    root_context,
    zerith,
) -> list[Penetration]:
    """Return active penetrations involving Zerith after collision filtering."""
    robot_geometry_ids = set()
    for body_index in plant.GetBodyIndices(zerith):
        body = plant.get_body(body_index)
        robot_geometry_ids.update(plant.GetCollisionGeometriesForBody(body))

    scene_graph_context = scene_graph.GetMyContextFromRoot(root_context)
    query_object = scene_graph.get_query_output_port().Eval(scene_graph_context)
    inspector = query_object.inspector()
    penetrations = []
    for pair in query_object.ComputePointPairPenetration():
        if (
            pair.id_A not in robot_geometry_ids
            and pair.id_B not in robot_geometry_ids
        ):
            continue
        penetrations.append(
            Penetration(
                depth=pair.depth,
                frame_a=inspector.GetName(inspector.GetFrameId(pair.id_A)),
                frame_b=inspector.GetName(inspector.GetFrameId(pair.id_B)),
            )
        )
    return sorted(penetrations, key=lambda item: item.depth, reverse=True)


def _report_initial_penetrations(
    penetrations: list[Penetration],
    penetration_limit: float | None,
) -> None:
    """Print active initial penetrations and enforce an optional depth limit."""
    print(f"Initial active robot penetration pairs: {len(penetrations)}")
    for penetration in penetrations:
        print(
            f"  {penetration.depth:.6f} m: "
            f"{penetration.frame_a} <-> {penetration.frame_b}"
        )

    if (
        penetration_limit is not None
        and penetrations
        and penetrations[0].depth > penetration_limit
    ):
        raise ValueError(
            f"Maximum initial penetration {penetrations[0].depth:.6f} m "
            f"exceeds limit {penetration_limit:.6f} m"
        )


def main() -> None:
    """Build and run the contact-aware left-arm simulation."""
    args = _parse_args()
    if args.time_step <= 0.0:
        raise ValueError("--time-step must be positive")
    if args.control_period < args.time_step:
        raise ValueError("--control-period must be at least --time-step")

    scene_dmd = args.scene_dmd.resolve()
    package_xml = (
        args.scene_package_xml.resolve()
        if args.scene_package_xml is not None
        else _find_package_xml(scene_dmd)
    )
    robot_model_dir = args.robot_model_dir.resolve()
    robot_urdf = robot_model_dir / ZERITH_URDF_RELATIVE_PATH

    if not scene_dmd.is_file():
        raise FileNotFoundError(f"Scene DMD does not exist: {scene_dmd}")
    if not package_xml.is_file():
        raise FileNotFoundError(f"Scene package.xml does not exist: {package_xml}")
    if not robot_urdf.is_file():
        raise FileNotFoundError(
            f"Zerith URDF does not exist: {robot_urdf}\n"
            "Run `python scripts/convert_zerith_for_drake.py`."
        )

    meshcat = Meshcat(args.meshcat_port)
    meshcat.Delete()
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder,
        time_step=args.time_step,
    )
    parser = Parser(plant)
    parser.SetAutoRenaming(True)
    _register_package_xml(parser, package_xml)
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
    if collision_geometry_count != 37:
        raise ValueError(
            f"Expected 37 Zerith collision geometries, got {collision_geometry_count}"
        )

    base_frame = plant.GetFrameByName("dipan_link", zerith)
    plant.WeldFrames(
        plant.world_frame(),
        base_frame,
        RigidTransform(
            RollPitchYaw(0.0, 0.0, np.deg2rad(args.robot_yaw_deg)),
            args.robot_xyz,
        ),
    )

    for config in JOINT_CONFIGS:
        joint = plant.GetJointByName(config.name, zerith)
        plant.AddJointActuator(
            f"{config.name}_actuator",
            joint,
            effort_limit=config.effort_limit,
        )

    plant.Finalize()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    collision_params = MeshcatVisualizerParams()
    collision_params.prefix = "collision"
    collision_params.role = Role.kProximity
    collision_params.visible_by_default = False
    MeshcatVisualizer.AddToBuilder(
        builder,
        scene_graph,
        meshcat,
        collision_params,
    )

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    _set_initial_positions(plant, zerith, plant_context)
    locked_joints = _lock_uncontrolled_joints(plant, zerith, plant_context)
    _add_sliders(meshcat, plant, zerith, plant_context)
    initial_penetrations = _find_robot_penetrations(
        plant,
        scene_graph,
        context,
        zerith,
    )
    _report_initial_penetrations(
        initial_penetrations,
        args.initial_penetration_limit,
    )

    actuation_port = plant.get_actuation_input_port()
    actuation_port.FixValue(
        plant_context,
        _calc_pd_actuation(meshcat, plant, zerith, plant_context),
    )

    simulator = Simulator(diagram, context)
    simulator.set_target_realtime_rate(args.realtime_rate)
    simulator.Initialize()

    print(f"Meshcat URL: {meshcat.web_url()}")
    print(f"Scene: {scene_dmd}")
    print(f"Zerith URDF: {robot_urdf}")
    print(f"Zerith collision geometries: {collision_geometry_count}")
    print(f"Controlled joints: {len(JOINT_CONFIGS)}")
    print(f"Locked Zerith joints: {len(locked_joints)}")
    print("The sliders are desired positions, not direct joint positions.")
    print("Use the collision tree checkbox to inspect proximity geometry.")
    print("Press Escape or Ctrl+C to stop.")

    try:
        while meshcat.GetButtonClicks("Stop simulation") < 1:
            actuation = _calc_pd_actuation(
                meshcat,
                plant,
                zerith,
                plant_context,
            )
            actuation_port.FixValue(plant_context, actuation)
            simulator.AdvanceTo(context.get_time() + args.control_period)
    except KeyboardInterrupt:
        pass
    finally:
        for config in JOINT_CONFIGS:
            meshcat.DeleteSlider(config.name)
        meshcat.DeleteButton("Stop simulation")


if __name__ == "__main__":
    main()
