"""Integration tests for the Zerith RobotAdapter implementation."""

import unittest
from pathlib import Path

import numpy as np
from pydrake.all import AddMultibodyPlantSceneGraph, DiagramBuilder, Parser

from src.online_manipulation import RobotAdapter
from src.online_manipulation.adapters.zerith import (
    ZerithRobotAdapter,
    make_zerith_robot_spec,
)
from src.zerith_online_env import ALL_SERVO_CONFIGS
from src.zerith_robot_config import (
    PICK_RAIL_POSITION_METERS,
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)

_REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
_ROBOT_MODEL_DIR = _REPOSITORY_ROOT / "models" / "zerith_drake"


def _adapter() -> ZerithRobotAdapter:
    """Return the calibrated fixed-rail test Adapter."""
    spec = make_zerith_robot_spec(
        robot_model_dir=_ROBOT_MODEL_DIR,
        robot_xyz=ROBOT_BASE_XYZ_METERS,
        robot_yaw_deg=ROBOT_BASE_YAW_DEG,
        rail_position=PICK_RAIL_POSITION_METERS,
        q_home_left=np.zeros(7),
    )
    return ZerithRobotAdapter(spec)


class ZerithRobotAdapterTest(unittest.TestCase):
    """Validate Zerith metadata and real Drake model integration."""

    def test_spec_matches_validated_control_order(self) -> None:
        adapter = _adapter()
        self.assertIsInstance(adapter, RobotAdapter)
        self.assertEqual(
            adapter.spec.controlled_joint_names,
            tuple(config.name for config in ALL_SERVO_CONFIGS),
        )
        self.assertEqual(
            adapter.spec.locked_joint_positions["daogui_joint"],
            PICK_RAIL_POSITION_METERS,
        )
        self.assertEqual(adapter.spec.home_positions, (0.0,) * 9)

    def test_gripper_width_maps_to_symmetric_joint_targets(self) -> None:
        adapter = _adapter()
        self.assertEqual(
            adapter.gripper_position_targets(0.08),
            {
                "left_jaw_left_finger_joint": -0.0,
                "left_jaw_right_finger_joint": 0.0,
            },
        )
        self.assertEqual(
            adapter.gripper_position_targets(0.0),
            {
                "left_jaw_left_finger_joint": -0.04,
                "left_jaw_right_finger_joint": 0.04,
            },
        )

    def test_adapter_builds_and_initializes_real_model(self) -> None:
        adapter = _adapter()
        builder = DiagramBuilder()
        plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
        parser = Parser(plant)
        model_instance = adapter.add_model(parser)
        adapter.configure_model(plant, model_instance)
        plant.Finalize()
        context = plant.CreateDefaultContext()
        adapter.initialize_state(plant, context, model_instance)

        rail = plant.GetJointByName("daogui_joint", model_instance)
        positions = plant.GetPositions(context)
        self.assertAlmostEqual(
            positions[rail.position_start()],
            PICK_RAIL_POSITION_METERS,
        )
        self.assertEqual(
            plant.num_actuated_dofs(),
            len(ALL_SERVO_CONFIGS),
        )
        for name in adapter.spec.locked_joint_positions:
            joint = plant.GetJointByName(name, model_instance)
            self.assertTrue(joint.is_locked(context), name)

        zeros = (0.0,) * len(ALL_SERVO_CONFIGS)
        observation = adapter.make_robot_observation(
            plant,
            context,
            model_instance,
            {
                "q_commanded": zeros,
                "torque_commanded": zeros,
                "torque_applied": zeros,
                "torque_saturated": (False,) * len(ALL_SERVO_CONFIGS),
            },
        )
        self.assertEqual(
            observation.joint_names,
            adapter.spec.controlled_joint_names,
        )
        self.assertEqual(observation.q, zeros)
        self.assertAlmostEqual(observation.gripper_width_m, 0.08)
        self.assertAlmostEqual(
            sum(
                value * value
                for value in observation.end_effector_pose.quaternion_wxyz
            ),
            1.0,
        )


if __name__ == "__main__":
    unittest.main()
