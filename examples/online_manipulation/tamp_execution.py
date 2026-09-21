"""Execute a grounded PickLift joint program through the public robot API."""

from __future__ import annotations

import math
from collections.abc import Mapping

from src.online_manipulation import GripperAction, JointPositionAction, RobotCommand


class JointWaypointPickLiftSkill:
    """Follow TAMP-certified staging, grasp and lifted joint waypoints."""

    def __init__(self, config, plan: Mapping, *, joint_step_limits: Mapping | None = None):
        expected = {"arm_joint_names", "staging_joint_positions",
                    "grasp_joint_positions", "lift_waypoints"}
        if (not isinstance(plan, Mapping) or not expected.issubset(plan)
                or set(plan) - expected - {"approach_waypoints"}):
            raise ValueError("Invalid grounded PickLift joint program")
        names = tuple(plan["arm_joint_names"])
        if len(names) != 7 or len(set(names)) != 7 or not all(
            isinstance(name, str) and name for name in names
        ):
            raise ValueError("PickLift joint program requires seven named arm joints")

        def positions(values):
            if len(values) != len(names) or not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in values
            ):
                raise ValueError("PickLift joint positions must be finite")
            return tuple(float(value) for value in values)

        waypoints = plan["lift_waypoints"]
        if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 32:
            raise ValueError("PickLift requires bounded lift waypoints")
        self.config = config
        self.names = names
        if joint_step_limits is not None:
            if set(joint_step_limits) != set(names) or any(
                    not math.isfinite(value) or value <= 0 for value in joint_step_limits.values()):
                raise ValueError("Joint step limits must cover the arm with positive finite values")
            self.joint_step_limits = tuple(float(joint_step_limits[name]) for name in names)
        else:
            self.joint_step_limits = None
        self.staging = positions(plan["staging_joint_positions"])
        self.grasp = positions(plan["grasp_joint_positions"])
        approach = plan.get("approach_waypoints", [self.grasp])
        if not isinstance(approach, list) or not 1 <= len(approach) <= 16:
            raise ValueError("PickLift requires bounded approach waypoints")
        self.approach = tuple(positions(item) for item in approach)
        if self.approach[-1] != self.grasp:
            raise ValueError("Approach path must terminate at the planned grasp")
        self.waypoints = tuple(positions(item) for item in waypoints)
        self.reset(None, {})

    def reset(self, observation, info):
        del observation, info
        self.stage = "pregrasp"
        self.stop_reason = None
        self.ticks = 0
        self.stage_ticks = 0
        self.stable_ticks = 0
        self.lost_contact_ticks = 0
        self.waypoint_index = 0
        self.approach_index = 0
        self.commanded_arm = None
        self.last_joint_target = None

    def diagnostics(self):
        return {"stage": self.stage, "failure_reason": self.stop_reason,
                "tick_count": self.ticks, "stage_ticks": self.stage_ticks,
                "waypoint_index": self.waypoint_index,
                "waypoint_count": len(self.waypoints),
                "approach_index": self.approach_index,
                "approach_count": len(self.approach),
                "commanded_arm_goal": self.last_joint_target,
                "stable_ticks": self.stable_ticks,
                "lost_contact_ticks": self.lost_contact_ticks}

    def _set_stage(self, stage):
        self.stage = stage
        self.stage_ticks = 0
        self.stable_ticks = 0

    def _fail(self, reason):
        self.stop_reason = reason
        self.stage = "failed"
        return RobotCommand()

    def _arm(self, positions, width):
        if self.joint_step_limits is not None:
            # Runtime independently clips per-joint deltas. Uniformly bound
            # this skill's target first so that clipping cannot bend the
            # certified straight joint edge into a different Cartesian path.
            delta = tuple(goal - current for goal, current in zip(
                positions, self.commanded_arm, strict=True))
            scale = min((1.0, *(limit / abs(value) for limit, value in zip(
                self.joint_step_limits, delta, strict=True) if value)))
            positions = tuple(current + scale * value for current, value in zip(
                self.commanded_arm, delta, strict=True))
        self.last_joint_target = tuple(positions)
        return RobotCommand(
            arms={"left": JointPositionAction(self.names, positions)},
            grippers={"left": GripperAction(width)},
        )

    def _reached(self, observation, positions, *, contact_mode=False):
        actual = dict(zip(observation.robot.joint_names, observation.robot.q,
                          strict=True))
        error = max(abs(actual[name] - value)
                    for name, value in zip(self.names, positions, strict=True))
        if contact_mode:
            # Reuse the shared expert's loaded-contact tracking contract.
            # A stable grasp has a steady servo offset; speed must also settle.
            velocities = dict(zip(observation.robot.joint_names, observation.robot.v,
                                  strict=True))
            ready = (error <= self.config.contact_cartesian_tracking_tolerance
                     and max(abs(velocities[name]) for name in self.names)
                     <= self.config.cartesian_velocity_tolerance)
        else:
            ready = error <= 0.025
        self.stable_ticks = self.stable_ticks + 1 if ready else 0
        return self.stable_ticks >= 4

    def act(self, observation):
        self.ticks += 1
        self.stage_ticks += 1
        if self.joint_step_limits is not None:
            commanded = dict(zip(observation.robot.joint_names,
                                 observation.robot.q_commanded, strict=True))
            self.commanded_arm = tuple(commanded[name] for name in self.names)
        if self.stage == "failed":
            return RobotCommand()
        if self.stage == "pregrasp":
            if self._reached(observation, self.staging):
                self._set_stage("align")
            elif self.stage_ticks > 220:
                return self._fail("planned_staging_timeout")
            else:
                return self._arm(self.staging, self.config.open_width_m)
        if self.stage == "align":
            goal = self.approach[self.approach_index]
            if self._reached(observation, goal):
                self.approach_index += 1
                self.stable_ticks = 0
                self.stage_ticks = 0
                if self.approach_index == len(self.approach):
                    self._set_stage("close")
                else:
                    goal = self.approach[self.approach_index]
            elif self.stage_ticks > 220:
                return self._fail("planned_grasp_timeout")
            if self.stage == "align":
                return self._arm(goal, self.config.open_width_m)
        if self.stage == "close":
            if observation.task.get("bilateral_gripper_contact", False):
                self.stable_ticks += 1
            else:
                self.stable_ticks = 0
            if self.stable_ticks >= self.config.stable_contact_steps:
                self._set_stage("verify")
            elif self.stage_ticks > self.config.maximum_close_steps:
                return self._fail("planned_bilateral_contact_timeout")
            else:
                return self._arm(self.grasp, self.config.closed_width_m)
        if self.stage == "verify":
            if observation.task.get("bilateral_gripper_contact", False):
                self.lost_contact_ticks = 0
            else:
                self.lost_contact_ticks += 1
            if self.lost_contact_ticks > self.config.maximum_lost_contact_steps:
                return self._fail("planned_lost_contact")
            goal = self.waypoints[self.waypoint_index]
            if self._reached(observation, goal, contact_mode=True):
                self.waypoint_index += 1
                self.stable_ticks = 0
                self.stage_ticks = 0
                if self.waypoint_index == len(self.waypoints):
                    self._set_stage("hold")
                else:
                    goal = self.waypoints[self.waypoint_index]
            elif self.stage_ticks > 160:
                return self._fail("planned_lift_waypoint_timeout")
            if self.stage == "verify":
                return self._arm(goal, self.config.closed_width_m)
        if self.stage == "hold":
            if self.stage_ticks > 300:
                return self._fail("planned_hold_timeout")
            return self._arm(self.waypoints[-1], self.config.closed_width_m)
        return self._fail("invalid_planned_skill_stage")
