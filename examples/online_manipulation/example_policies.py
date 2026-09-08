"""External policy examples, independent of environment construction."""

from collections.abc import Mapping
from typing import Any

from src.online_manipulation import (
    EnvironmentConfig,
    HoldAction,
    HoldPolicy,
    JointStepPolicy,
    JointStepPolicyConfig,
    Observation,
    RobotAction,
)


class MyPolicy:
    """User-editable minimal policy; never creates or mutates an environment."""

    def reset(self, observation: Observation, info: Mapping[str, Any]) -> None:
        """Reset episode-local policy state here."""
        del observation, info

    def act(self, observation: Observation) -> RobotAction:
        """Replace this hold with an action computed from the observation."""
        del observation
        return HoldAction()


def make_hold_policy(config: EnvironmentConfig) -> HoldPolicy:
    """Build a policy that needs no robot or task-specific configuration."""
    del config
    return HoldPolicy()


def make_joint_step_policy(config: EnvironmentConfig) -> JointStepPolicy:
    """Send a 0.003 rad named Zerith joint increment once, then hold.

    This concrete policy selects a joint explicitly; the environment and
    generic runner do not select joints.
    """
    del config
    return JointStepPolicy(JointStepPolicyConfig(
        joint_name="left_shoulder_pitch_joint", delta=0.003
    ))


def make_my_policy(config: EnvironmentConfig) -> MyPolicy:
    """Construct a user's policy separately from the environment factory."""
    del config
    return MyPolicy()
