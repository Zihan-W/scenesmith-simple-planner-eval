"""External absolute-pose policy; independent of scene/env construction."""

from src.online_manipulation import CartesianPoseAction, Observation, Pose


class CartesianGoalPolicy:
    """Keep sending one absolute goal; the caller checks arrival/timeout."""

    def __init__(self, end_effector_frame: str, goal: Pose):
        """Store an explicit world-frame goal, not a precomputed trajectory."""
        self.action = CartesianPoseAction(end_effector_frame, "world", goal)

    def reset(self, observation: Observation, info: dict) -> None:
        """This stateless policy retains its configured goal across resets."""
        del observation, info

    def act(self, observation: Observation) -> CartesianPoseAction:
        """Return the same goal; the adapter recomputes IK each step."""
        del observation
        return self.action
