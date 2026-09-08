"""One model assembly path shared by planning and live simulation."""

from pydrake.all import LoadModelDirectives, ProcessModelDirectives

from src.online_manipulation.drake_utils import register_package_xml
from src.online_manipulation.scene_geometry import proximity_records, resolve_ground_geometries


def populate_model(parser, scenario, adapter):
    """Load the scene and let the adapter configure its robot before Finalize."""
    parser.SetAutoRenaming(True)
    for package_xml in scenario.package_xmls:
        register_package_xml(parser, package_xml)
    ProcessModelDirectives(LoadModelDirectives(str(scenario.dmd_path)), parser)
    if parser.plant().HasModelInstanceNamed(adapter.spec.model_instance_name):
        raise ValueError(
            f"Scene already contains robot instance {adapter.spec.model_instance_name}; "
            "select a robot-free scene or explicitly prepare a derived scene"
        )
    instance = adapter.add_model(parser)
    adapter.configure_model(parser.plant(), instance)
    validate_actuator_mapping(parser.plant(), instance, adapter.spec)
    return instance


def validate_actuator_mapping(plant, instance, spec):
    """Validate the public single-DOF servo actuator convention before Finalize."""
    # Drake 1.49 populates the instance-indexed list only at Finalize.
    actuators = [
        plant.get_joint_actuator(i)
        for i in plant.GetJointActuatorIndices()
        if plant.get_joint_actuator(i).model_instance() == instance
    ]
    by_name = {a.name(): a for a in actuators}
    for name in spec.controlled_joint_names:
        joint = plant.GetJointByName(name, instance)
        expected = f"{name}_actuator"
        if joint.num_positions() != 1 or joint.num_velocities() != 1:
            raise ValueError(f"RobotAdapter controlled joint {name} must be single-DOF")
        if expected not in by_name:
            raise ValueError(
                f"RobotAdapter requires actuator {expected} for joint {name}"
            )
        attached = [a for a in actuators if a.joint().index() == joint.index()]
        if len(attached) != 1 or by_name[expected].joint().index() != joint.index():
            raise ValueError(
                f"RobotAdapter actuator {expected} must exclusively drive joint {name}"
            )


def support_geometry_limits(adapter, scenario, plant, inspector):
    """Resolve exact wheel/support-to-floor pairs; walls never inherit them."""
    floors = resolve_ground_geometries(
        plant, inspector, scenario.ground_body_names, scenario.ground_geometries
    )
    if getattr(adapter, "base_mode", "fixed") != "wheel_dynamic":
        return {}
    records = proximity_records(plant, inspector)
    bodies = {
        f"{adapter.spec.model_instance_name}::{name}"
        for name in adapter.support_body_names + adapter.wheel_body_names
    }
    return {
        tuple(sorted(((rb, rg), (fb, fg)))): adapter.base_config.support_contact_allowance_m
        for _, rb, rg in records if rb in bodies
        for gid, fb, fg in records if gid in floors
    }
