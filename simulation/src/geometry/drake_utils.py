"""Small shared Drake parsing utilities."""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


def register_package_xml(parser: Any, package_xml: Path) -> None:
    """Register one ROS-style package.xml with a Drake Parser."""
    path = Path(package_xml)
    root = ET.parse(path).getroot()
    name = root.findtext("name")
    if name is None:
        raise ValueError(f"Missing <name> in {path}")
    name = name.strip()
    packages = parser.package_map()
    if packages.Contains(name) and Path(packages.GetPath(name)).resolve() != path.parent.resolve():
        raise ValueError(f"Conflicting package {name}: {packages.GetPath(name)} and {path.parent}")
    packages.Add(name, str(path.parent.resolve()))


def set_free_body_world_pose(
    plant: Any,
    plant_context: Any,
    body: Any,
    world_from_body: Any,
) -> None:
    """Set a free body's world pose for either world or framed joints.

    Drake's ``SetFreeBodyPose`` accepts the pose between the floating joint's
    parent and child frames. A DMD model may attach that joint below a named
    scene frame, and its child frame need not coincide with the body frame.
    This helper performs both fixed-frame conversions explicitly.
    """
    floating_joints = [
        plant.get_joint(index)
        for index in plant.GetJointIndices(body.model_instance())
        if (
            plant.get_joint(index).child_body().index() == body.index()
            and plant.get_joint(index).num_positions() == 7
            and plant.get_joint(index).num_velocities() == 6
        )
    ]
    if len(floating_joints) != 1:
        model_name = plant.GetModelInstanceName(body.model_instance())
        raise ValueError(
            "Expected exactly one floating joint for "
            f"{model_name}::{body.name()}, found {len(floating_joints)}"
        )
    joint = floating_joints[0]
    world_from_parent_joint = joint.frame_on_parent().CalcPoseInWorld(
        plant_context
    )
    body_from_child_joint = joint.frame_on_child().CalcPoseInBodyFrame(
        plant_context
    )
    plant.SetFreeBodyPose(
        plant_context,
        body,
        world_from_parent_joint.inverse()
        @ world_from_body
        @ body_from_child_joint,
    )
