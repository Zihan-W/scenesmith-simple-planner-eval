"""Non-welded mobile Zerith model construction; source assets stay unchanged."""

import dataclasses
import math
import xml.etree.ElementTree as ET

from pydrake.all import FixedOffsetFrame, PlanarJoint, RigidTransform

from src.online_manipulation.adapters.description import (
    DescriptionRobotAdapter,
    drake_pose,
)
from src.online_manipulation.adapters.zerith_dual import ZerithDualRobotAdapter


@dataclasses.dataclass(frozen=True)
class DifferentialDriveSpec:
    """Measured robot geometry and model limits, distinct from servo settings."""

    radius_m: float
    track_m: float
    joint_axis_signs: tuple[float, float]
    velocity_limit_rad_s: float
    effort_limit_nm: float


class ZerithMobileRobotAdapter(ZerithDualRobotAdapter):
    """Two differential drive wheels and four explicitly simplified supports."""

    wheel_joint_names = ("left_middle_wheel_joint", "right_middle_wheel_joint")
    wheel_body_names = ("left_middle_wheel_link", "right_middle_wheel_link")
    drive_spec = DifferentialDriveSpec(0.0835, 0.379, (1.0, -1.0), 2.3, 60.0)
    navigation_frame_name = "navigation_frame"
    planar_joint_name = "prescribed_planar_base"
    support_body_names = tuple(
        f"{side}_{end}_wheel_link"
        for side in ("left", "right")
        for end in ("front", "rear")
    )

    def __init__(self, spec, base_config):
        self.base_config = base_config
        self.base_mode = base_config.mode
        if self.base_mode == "planar_kinematic":
            pose = drake_pose(spec.base_pose)
            rpy = pose.rotation().ToRollPitchYaw().vector()
            if (
                abs(
                    pose.translation()[2]
                    - base_config.ground_height_m
                    - base_config.base_height_m
                )
                > 1e-9
                or max(abs(rpy[0]), abs(rpy[1])) > 1e-9
            ):
                raise ValueError(
                    "Prescribed planar initial pose must match its configured plane; no projection is performed"
                )
        if self.base_mode == "wheel_dynamic":
            spec = dataclasses.replace(
                spec,
                locked_joint_positions={
                    name: value
                    for name, value in spec.locked_joint_positions.items()
                    if name not in self.wheel_joint_names
                },
            )
        super().__init__(spec)

    def add_model(self, parser):
        """Derive only wheel/support proximity for mobile dynamics, in memory.

        CAD wheel radii/axes are measured in P0. Equal tire support heights and
        frictionless auxiliary sliders are explicit simulation assumptions.
        No gripper, scene, ground or other body friction is changed.
        """
        parser.package_map().Add(
            self.spec.package_name, str(self.spec.model_path.parent.parent)
        )
        root = ET.parse(self.spec.model_path).getroot()
        if self.base_mode == "wheel_dynamic":
            drake = "http://drake.mit.edu"
            ET.register_namespace("drake", drake)
            for name in self.support_body_names:
                for collision in root.findall(f"./link[@name='{name}']/collision"):
                    origin = collision.find("origin")
                    # Bring all four support bottoms to z=-0.180 m. The drive
                    # cylinder bottoms are also -0.180 m in base coordinates.
                    z = 0.0015 if "front" in name else -0.00008539
                    origin.set("xyz", f"0 0 {z}")
                    props = ET.SubElement(collision, f"{{{drake}}}proximity_properties")
                    ET.SubElement(props, f"{{{drake}}}mu_static", value="0")
                    ET.SubElement(props, f"{{{drake}}}mu_dynamic", value="0")
            for name in self.wheel_joint_names:
                body = name.replace("_joint", "_link")
                for collision in root.findall(f"./link[@name='{body}']/collision"):
                    collision.find("origin").set("rpy", f"{math.pi / 2} 0 0")
                    geometry = collision.find("geometry")
                    geometry.clear()
                    ET.SubElement(
                        geometry,
                        "cylinder",
                        radius=str(self.drive_spec.radius_m),
                        length="0.0415",
                    )
        instances = parser.AddModelsFromString(
            ET.tostring(root, encoding="unicode"), "urdf"
        )
        if len(instances) != 1:
            raise ValueError("Expected exactly one mobile robot model")
        return instances[0]

    def configure_model(self, plant, instance):
        """Add TCP/navigation frames, never weld the base to world."""
        self.add_actuators(plant, instance)
        self.add_tcp_frames(plant, instance)
        # Differential drive reference lies on the axle, projected onto ground.
        frame = plant.AddFrame(
            FixedOffsetFrame(
                self.navigation_frame_name,
                plant.GetFrameByName(self.spec.base_link_name, instance),
                RigidTransform([0.0544, 0, -self.base_config.base_height_m]),
            )
        )
        if self.base_mode == "planar_kinematic":
            ground = plant.AddFrame(
                FixedOffsetFrame(
                    "navigation_plane",
                    plant.world_frame(),
                    RigidTransform([0, 0, self.base_config.ground_height_m]),
                )
            )
            plant.AddJoint(PlanarJoint(self.planar_joint_name, ground, frame))
        else:
            for name in self.wheel_joint_names:
                plant.AddJointActuator(
                    f"{name}_drive",
                    plant.GetJointByName(name, instance),
                    effort_limit=min(
                        self.base_config.maximum_wheel_torque_nm,
                        self.drive_spec.effort_limit_nm,
                    ),
                )
        # Dynamic mode retains the root's six floating DOFs added by Finalize.

    def initialize_state(self, plant, context, instance):
        """Apply reset-only pose; lock rail/passive joints, never the dynamic base."""
        DescriptionRobotAdapter.initialize_state(self, plant, context, instance)
        if self.base_mode == "planar_kinematic":
            transform = drake_pose(self.spec.base_pose) @ RigidTransform(
                [0.0544, 0, -self.base_config.base_height_m]
            )
            joint = plant.GetJointByName(self.planar_joint_name, instance)
            joint.set_translation(context, transform.translation()[:2])
            joint.set_rotation(
                context, transform.rotation().ToRollPitchYaw().yaw_angle()
            )
            joint.Lock(context)
        else:
            plant.SetFreeBodyPose(
                context,
                plant.GetBodyByName(self.spec.base_link_name, instance),
                drake_pose(self.spec.base_pose),
            )
