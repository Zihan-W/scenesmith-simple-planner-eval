"""Tests for the robot-independent coupled inverse-dynamics servo."""

import unittest

import numpy as np
from pydrake.all import (
    MultibodyPlant,
    RevoluteJoint,
    SpatialInertia,
    UnitInertia,
)

from src.online_manipulation.controller import CoupledInverseDynamicsServo
from src.online_manipulation.specs import JointSpec


def _single_joint_servo() -> tuple[CoupledInverseDynamicsServo, object]:
    """Build a one-joint Drake plant with a deliberately low torque limit."""
    plant = MultibodyPlant(time_step=0.001)
    body = plant.AddRigidBody(
        "link",
        SpatialInertia(
            mass=1.0,
            p_PScm_E=np.zeros(3),
            G_SP_E=UnitInertia.SolidSphere(0.1),
        ),
    )
    joint = plant.AddJoint(
        RevoluteJoint(
            "joint",
            plant.world_frame(),
            body.body_frame(),
            [0.0, 1.0, 0.0],
        )
    )
    actuator = plant.AddJointActuator("joint_actuator", joint, 0.1)
    plant.Finalize()
    context = plant.CreateDefaultContext()
    spec = JointSpec(
        name="joint",
        joint_type="revolute",
        position_lower=-2.0,
        position_upper=2.0,
        velocity_limit=3.0,
        effort_limit=0.1,
        kp=100.0,
        kd=10.0,
    )
    servo = CoupledInverseDynamicsServo(
        plant=plant,
        joints=(joint,),
        actuators=(actuator,),
        joint_specs=(spec,),
    )
    return servo, context


class CoupledInverseDynamicsServoTest(unittest.TestCase):
    """Validate generic dimensions, ordering, and effort limiting."""

    def test_compute_reports_and_limits_torque(self) -> None:
        servo, context = _single_joint_servo()
        output = servo.compute(context, [1.0])
        self.assertEqual(servo.joint_names, ("joint",))
        self.assertGreater(abs(output.raw_torque[0]), 0.1)
        self.assertAlmostEqual(abs(output.applied_torque[0]), 0.1)
        self.assertTrue(output.saturated[0])
        self.assertAlmostEqual(output.actuation[0], output.applied_torque[0])

    def test_compute_rejects_wrong_command_dimension(self) -> None:
        servo, context = _single_joint_servo()
        with self.assertRaisesRegex(ValueError, "joint count"):
            servo.compute(context, [0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
