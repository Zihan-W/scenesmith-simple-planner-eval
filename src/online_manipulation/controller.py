"""Robot-independent coupled inverse-dynamics joint servo."""

import dataclasses
from collections.abc import Sequence
from typing import Any

import numpy as np
from pydrake.all import MultibodyForces

from src.online_manipulation.specs import JointSpec


def configure_joint_servos(joints, settings):
    """Apply run-time gains/effort caps without changing model joint limits.

    Settings are keyed by joint name; kp/kd are acceleration-domain gains.
    The adapter's declared effort limit remains an upper bound. This function
    never modifies URDF, TCP geometry, position limits, or velocity limits.
    """
    unknown = set(settings) - {j.name for j in joints}
    if unknown:
        raise ValueError(f"Unknown servo joints: {sorted(unknown)}")
    configured = []
    for joint in joints:
        values = settings.get(joint.name, {})
        if set(values) - {"kp", "kd", "effort_limit"}:
            raise ValueError(f"Only kp/kd/effort_limit are runtime servo settings: {joint.name}")
        if values.get("effort_limit", joint.effort_limit) > joint.effort_limit:
            raise ValueError(f"Runtime effort cap exceeds adapter limit: {joint.name}")
        configured.append(dataclasses.replace(joint, **values))
    return tuple(configured)


@dataclasses.dataclass(frozen=True)
class ServoOutput:
    """One controller update in configured joint order."""

    q: np.ndarray
    v: np.ndarray
    q_desired: np.ndarray
    gravity_torque: np.ndarray
    pd_torque: np.ndarray
    raw_torque: np.ndarray
    applied_torque: np.ndarray
    saturated: np.ndarray
    actuation: np.ndarray


class CoupledInverseDynamicsServo:
    """Compute acceleration-domain PD plus full inverse-dynamics torques."""

    def __init__(
        self,
        *,
        plant: Any,
        joints: Sequence[Any],
        actuators: Sequence[Any],
        joint_specs: Sequence[JointSpec],
    ):
        """Bind an ordered set of single-DOF joints and actuators."""
        self._plant = plant
        self._joints = tuple(joints)
        self._actuators = tuple(actuators)
        self._joint_specs = tuple(joint_specs)
        if not (
            len(self._joints)
            == len(self._actuators)
            == len(self._joint_specs)
        ):
            raise ValueError("joints, actuators, and joint_specs must align")
        if not self._joints:
            raise ValueError("At least one controlled joint is required")
        for joint, spec in zip(
            self._joints,
            self._joint_specs,
            strict=True,
        ):
            if joint.name() != spec.name:
                raise ValueError(
                    f"Joint order mismatch: {joint.name()} != {spec.name}"
                )
            if joint.num_positions() != 1 or joint.num_velocities() != 1:
                raise ValueError(
                    f"Controlled joint {joint.name()} must be single-DOF"
                )

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Return configured joint names in command order."""
        return tuple(spec.name for spec in self._joint_specs)

    def compute(
        self,
        plant_context: Any,
        desired_positions: Sequence[float],
    ) -> ServoOutput:
        """Compute limited actuation for one low-level control update."""
        q_desired = np.asarray(desired_positions, dtype=float)
        if q_desired.shape != (len(self._joints),):
            raise ValueError(
                "desired_positions must match the configured joint count"
            )
        if not np.all(np.isfinite(q_desired)):
            raise ValueError("desired_positions must contain finite values")

        positions = self._plant.GetPositions(plant_context)
        velocities = self._plant.GetVelocities(plant_context)
        q = np.asarray(
            [positions[joint.position_start()] for joint in self._joints]
        )
        v = np.asarray(
            [velocities[joint.velocity_start()] for joint in self._joints]
        )
        kp = np.asarray([spec.kp for spec in self._joint_specs])
        kd = np.asarray([spec.kd for spec in self._joint_specs])
        effort_limits = np.asarray(
            [spec.effort_limit for spec in self._joint_specs]
        )

        gravity_generalized = self._plant.CalcGravityGeneralizedForces(
            plant_context
        )
        gravity_torque = np.asarray(
            [
                -gravity_generalized[joint.velocity_start()]
                for joint in self._joints
            ]
        )
        desired_acceleration = np.zeros(self._plant.num_velocities())
        for index, joint in enumerate(self._joints):
            desired_acceleration[joint.velocity_start()] = (
                kp[index] * (q_desired[index] - q[index])
                - kd[index] * v[index]
            )
        force_elements = MultibodyForces(self._plant)
        self._plant.CalcForceElementsContribution(
            plant_context,
            force_elements,
        )
        generalized_force = self._plant.CalcInverseDynamics(
            plant_context,
            desired_acceleration,
            force_elements,
        )
        raw_torque = np.asarray(
            [
                generalized_force[joint.velocity_start()]
                for joint in self._joints
            ]
        )
        pd_torque = raw_torque - gravity_torque
        applied_torque = np.clip(
            raw_torque,
            -effort_limits,
            effort_limits,
        )
        saturated = ~np.isclose(raw_torque, applied_torque, atol=1e-12)
        actuation = np.zeros(self._plant.num_actuated_dofs())
        for actuator, torque in zip(
            self._actuators,
            applied_torque,
            strict=True,
        ):
            actuation[actuator.input_start()] = torque
        return ServoOutput(
            q=q,
            v=v,
            q_desired=q_desired.copy(),
            gravity_torque=gravity_torque,
            pd_torque=pd_torque,
            raw_torque=raw_torque,
            applied_torque=applied_torque,
            saturated=saturated,
            actuation=actuation,
        )
