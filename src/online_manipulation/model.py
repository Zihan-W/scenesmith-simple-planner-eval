"""One model assembly path shared by planning and live simulation."""

from pydrake.all import LoadModelDirectives, ProcessModelDirectives

from src.online_manipulation.drake_utils import register_package_xml


def populate_model(parser, scenario, adapter):
    """Load the scene and let the adapter configure its robot before Finalize."""
    parser.SetAutoRenaming(True)
    for package_xml in scenario.package_xmls:
        register_package_xml(parser, package_xml)
    ProcessModelDirectives(LoadModelDirectives(str(scenario.dmd_path)), parser)
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


def support_contact_limits(adapter, scenario):
    """Return only the declared dynamic wheels' compressible ground contacts."""
    if getattr(adapter, "base_mode", "fixed") != "wheel_dynamic":
        return {}
    return {
        tuple(
            sorted((f"{adapter.spec.model_instance_name}::{body}", ground))
        ): adapter.base_config.support_contact_allowance_m
        for body in adapter.support_body_names + adapter.wheel_body_names
        for ground in scenario.ground_body_names
    }
