"""Use a current planning snapshot before executing one named joint goal."""

from src.online_manipulation import execute_validated_joint_goal


def move_one_joint(env, joint_name, delta=0.001):
    """A planner supplies a goal; the public helper synchronizes and checks it."""
    obs = env.observation
    positions = dict(zip(obs.robot.joint_names, obs.robot.q, strict=True))
    goal = positions[joint_name] + delta
    return execute_validated_joint_goal(env=env, goal_positions={joint_name: goal})
