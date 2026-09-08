"""Copy this module outside the repository and select its trusted factories.

Only the public API is imported. This HOLD evaluator is an interface example,
not a grasp classifier or benchmark robustness claim.
"""

from src.online_manipulation import HoldAction, TaskEvaluation


class MyPolicy:
    """A caller-owned policy with no environment or robot internals."""

    def reset(self, observation, info):
        """No policy state is needed for HOLD."""

    def act(self, observation):
        """Keep robot targets unchanged; request braking for a mobile base."""
        return HoldAction()


class MyEvaluator:
    """Confirm three consecutive low-speed observations, starting anew at reset."""

    def __init__(self, maximum_joint_speed=0.05):
        self.maximum_joint_speed = maximum_joint_speed
        self.stable_samples = 0

    def reset(self, observation, info):
        self.stable_samples = 0

    def evaluate(self, observation, baseline):
        speed = max(abs(v) for v in observation.robot.v)
        self.stable_samples = self.stable_samples + 1 if speed <= self.maximum_joint_speed else 0
        success = self.stable_samples >= 3
        return TaskEvaluation(success=success, terminated=success,
                              reward=float(success),
                              reason="custom_hold_confirmed" if success else "custom_hold_running",
                              metrics={"stable_samples": self.stable_samples, "joint_speed": speed})


def make_policy(context):
    """Factory receives resolved public configuration, never a Drake context."""
    return MyPolicy()


def make_evaluator(options):
    """A fresh evaluator owns all per-episode classifier state."""
    return MyEvaluator(options.get("maximum_joint_speed", 0.05))
