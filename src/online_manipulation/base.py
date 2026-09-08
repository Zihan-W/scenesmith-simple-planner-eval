"""Explicit prescribed-planar and wheel-driven execution modes.

The prescribed mode integrates a locked PlanarJoint in physics-sized increments;
it is an ideal moving constraint, not free-body dynamics and not a goal teleport.
The dynamic mode only writes wheel torques. It never sets base pose or velocity.
"""

import dataclasses
import math
import numpy as np

from src.online_manipulation.actions import BaseVelocityAction
from src.online_manipulation.adapters.description import public_pose


@dataclasses.dataclass(frozen=True)
class BaseConfig:
    """Auditable simulation limits, independently configurable by mode."""

    mode: str
    maximum_velocity_m_s: float = 0.15
    maximum_yaw_rate_rad_s: float = 0.6
    acceleration_m_s2: float = 0.25
    yaw_acceleration_rad_s2: float = 0.8
    maximum_wheel_speed_rad_s: float = 2.3
    maximum_wheel_torque_nm: float = 60.0
    wheel_velocity_gain: float = 30.0
    ground_height_m: float = 0.0
    base_height_m: float = 0.1808
    support_contact_allowance_m: float = 0.002

    def __post_init__(self):
        if self.mode not in ("planar_kinematic", "wheel_dynamic"):
            raise ValueError("Select an explicit mobile base mode")
        if not math.isfinite(self.ground_height_m):
            raise ValueError("Ground height must be finite")
        for f in dataclasses.fields(self):
            if f.name not in ("mode", "ground_height_m"):
                value = getattr(self, f.name)
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f"Invalid base parameter {f.name}")


def integrate_planar(pose, velocity, yaw_rate, dt):
    """Exact constant-twist differential-drive integration (including reverse)."""
    x, y, yaw = pose
    if abs(yaw_rate) < 1e-12:
        return np.array(
            [x + dt * velocity * math.cos(yaw), y + dt * velocity * math.sin(yaw), yaw]
        )
    angle = yaw + yaw_rate * dt
    return np.array(
        [
            x + velocity / yaw_rate * (math.sin(angle) - math.sin(yaw)),
            y - velocity / yaw_rate * (math.cos(angle) - math.cos(yaw)),
            angle,
        ]
    )


class BaseBackend:
    """Per-environment base command state; the runtime remains the sole clock."""

    def __init__(self, runtime, config):
        self.runtime, self.config = runtime, config
        self.target = np.zeros(2)
        self.limited = np.zeros(2)
        self.blocked = False
        self.blocking_pairs = []

    def reset(self):
        self.target[:] = 0
        self.limited[:] = 0
        self.blocked = False
        self.blocking_pairs = []
        self.owner = None

    def validate_command(self, action):
        """Reject competing writers; ownership changes require measured parking."""
        if action is None:
            return None
        if self.owner is not None and action.control_owner != self.owner:
            return "base_control_owned_by_another_caller"
        if action.release_control or (
            self.owner is None and action.control_owner is not None
        ):
            observation = self.observe()
            if (
                np.linalg.norm(observation["linear_velocity_world_m_s"][:2]) > 0.01
                or abs(observation["yaw_rate_rad_s"]) > 0.02
            ):
                return "base_control_handoff_requires_stop"
        if action.release_control and self.owner != action.control_owner:
            return "base_control_not_owned"
        return None

    def command(self, action=None):
        """Omission explicitly expires the previous tick's velocity command."""
        action = action if action is not None else BaseVelocityAction(0, 0)
        if action.control_owner is not None:
            self.owner = None if action.release_control else action.control_owner
        self.target = np.clip(
            [action.velocity_m_s, action.yaw_rate_rad_s],
            [-self.config.maximum_velocity_m_s, -self.config.maximum_yaw_rate_rad_s],
            [self.config.maximum_velocity_m_s, self.config.maximum_yaw_rate_rad_s],
        )

    def update_limit(self, dt):
        bounds = (
            np.array(
                [self.config.acceleration_m_s2, self.config.yaw_acceleration_rad_s2]
            )
            * dt
        )
        self.limited += np.clip(self.target - self.limited, -bounds, bounds)

    def observe(self):
        """Report actual link pose and twist, plus explicitly identified odometry."""
        r = self.runtime
        frame = r.plant.GetFrameByName(r.adapter.navigation_frame_name, r.instance)
        pose = frame.CalcPoseInWorld(r.plant_context)
        base_link = r.plant.GetBodyByName(r.spec.base_link_name, r.instance)
        base_link_pose = public_pose(
            base_link.EvalPoseInWorld(r.plant_context)
        ).as_dict()
        twist = frame.CalcSpatialVelocityInWorld(r.plant_context)
        if self.config.mode == "planar_kinematic":
            velocity = self.limited if not self.blocked else np.zeros(2)
            yaw = pose.rotation().ToRollPitchYaw().yaw_angle()
            linear = [velocity[0] * math.cos(yaw), velocity[0] * math.sin(yaw), 0]
            angular = velocity[1]
        else:
            linear = twist.translational().tolist()
            angular = float(twist.rotational()[2])
        return {
            "mode": self.config.mode,
            "frame": r.adapter.navigation_frame_name,
            "base_link_frame": r.spec.base_link_name,
            "base_link_pose": base_link_pose,
            "pose": public_pose(pose).as_dict(),
            "linear_velocity_world_m_s": linear,
            "yaw_rate_rad_s": angular,
            "odom_source": (
                "prescribed_kinematics"
                if self.config.mode == "planar_kinematic"
                else "simulation_ground_truth"
            ),
            "world_from_odom": {
                "translation_m": [0, 0, 0],
                "quaternion_wxyz": [1, 0, 0, 0],
            },
            "odom_from_navigation": public_pose(pose).as_dict(),
            "odom_from_base_link": base_link_pose,
            "blocked": self.blocked,
            "blocking_pairs": self.blocking_pairs,
            "requested_velocity": self.target.tolist(),
            "control_owner": self.owner,
        }


class PlanarKinematicBase(BaseBackend):
    """Ideal planar constraint, swept collision checked every physics increment."""

    def before_physics(self, dt):
        r = self.runtime
        joint = r.plant.GetJointByName(r.adapter.planar_joint_name, r.instance)
        pose = np.r_[
            joint.get_translation(r.plant_context), joint.get_rotation(r.plant_context)
        ]
        self.update_limit(dt)
        candidate = integrate_planar(pose, *self.limited, dt)
        valid, pairs = r.check_base_edge(pose, candidate)
        self.blocked = not valid
        self.blocking_pairs = pairs
        if not valid:
            self.limited[:] = 0
            return
        # Explicit prescribed motion of a constrained planar joint, never a
        # hidden overwrite of a freely simulated six-DOF body.
        joint.set_translation(r.plant_context, candidate[:2])
        joint.set_rotation(r.plant_context, candidate[2])


class WheelDrivenDynamicBase(BaseBackend):
    """Wheel velocity servo; traction and actual base motion are solved by Drake."""

    def wheel_actuation(self, dt):
        self.update_limit(dt)
        r, c = self.runtime, self.config
        v, omega = self.limited
        drive = r.adapter.drive_spec
        desired = (
            np.array(
                [
                    (v - omega * drive.track_m / 2) / drive.radius_m,
                    (v + omega * drive.track_m / 2) / drive.radius_m,
                ]
            )
            * drive.joint_axis_signs
        )
        largest = np.max(np.abs(desired))
        speed_limit = min(c.maximum_wheel_speed_rad_s, drive.velocity_limit_rad_s)
        if largest > speed_limit:
            desired *= speed_limit / largest
        actuation = np.zeros(r.plant.num_actuated_dofs())
        self.wheel_telemetry = {}
        for name, goal in zip(r.adapter.wheel_joint_names, desired, strict=True):
            joint = r.plant.GetJointByName(name, r.instance)
            speed = joint.get_angular_rate(r.plant_context)
            effort_limit = min(c.maximum_wheel_torque_nm, drive.effort_limit_nm)
            torque = float(
                np.clip(
                    c.wheel_velocity_gain * (goal - speed), -effort_limit, effort_limit
                )
            )
            actuator = r.plant.GetJointActuatorByName(f"{name}_drive", r.instance)
            actuation[actuator.input_start()] = torque
            self.wheel_telemetry[name] = {
                "target_rad_s": float(goal),
                "actual_rad_s": float(speed),
                "torque_nm": torque,
            }
        return actuation

    def observe(self):
        result = super().observe()
        result["wheels"] = self.wheel_telemetry
        contacts = self.runtime.plant.get_contact_results_output_port().Eval(
            self.runtime.plant_context
        )
        loads = {}
        for index in range(contacts.num_point_pair_contacts()):
            contact = contacts.point_pair_contact_info(index)
            force = float(abs(contact.contact_force()[2]))
            for body_index in (contact.bodyA_index(), contact.bodyB_index()):
                body = self.runtime.plant.get_body(body_index)
                if body.model_instance() == self.runtime.instance:
                    loads[body.name()] = loads.get(body.name(), 0.0) + force
        result["vertical_contact_loads_n"] = loads
        return result
