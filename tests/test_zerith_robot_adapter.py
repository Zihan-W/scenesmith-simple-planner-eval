"""Integration tests for the Zerith RobotAdapter implementation."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from pydrake.all import AddMultibodyPlantSceneGraph, DiagramBuilder, Parser

from src.online_manipulation import (
    CartesianDeltaAction,
    CompositeAction,
    GripperAction,
    JointDeltaAction,
    JointPositionAction,
    NullTask,
    ObservedBodySpec,
    Pose,
    RobotAdapter,
    ScenarioSpec,
    TimingConfig,
)
from src.online_manipulation.adapters.zerith import (
    ZerithLegacyActionTranslator,
    ZerithRobotAdapter,
    make_legacy_zerith_online_environment,
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

    def test_null_task_scene_does_not_require_a_target_model(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            scene_root = Path(directory)
            dmd_path = scene_root / "empty.dmd.yaml"
            package_xml = scene_root / "package.xml"
            model_path = scene_root / "movable.sdf"
            model_path.write_text(
                """<sdf version="1.7">
<model name="movable">
  <link name="body">
    <inertial>
      <mass>1</mass>
      <inertia>
        <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
      </inertia>
    </inertial>
  </link>
</model>
</sdf>
""",
                encoding="utf-8",
            )
            dmd_path.write_text(
                """directives:
- add_model:
    name: movable
    file: package://empty_scene/movable.sdf
    default_free_body_pose:
        body:
            base_frame: world
            translation: [0, 0, 1]
            rotation: !Rpy { deg: [0, 0, 0] }
""",
                encoding="utf-8",
            )
            package_xml.write_text(
                "<package format=\"2\"><name>empty_scene</name></package>\n",
                encoding="utf-8",
            )
            initial_pose = Pose(
                (0.2, 0.3, 1.5),
                (1.0, 0.0, 0.0, 0.0),
            )
            scenario = ScenarioSpec(
                dmd_path=dmd_path,
                package_xmls=(package_xml,),
                initial_object_poses={"movable": initial_pose},
                observed_bodies=(
                    ObservedBodySpec("movable", "movable", "body"),
                ),
                contact_parameters={
                    "penetration_allowance_m": 0.001,
                    "stiction_tolerance_m_s": 0.01,
                },
            )
            env = make_legacy_zerith_online_environment(
                scenario=scenario,
                adapter=_adapter(),
                timing=TimingConfig(),
                episode_duration=0.2,
                task=NullTask(),
            )
            observation, _ = env.reset(seed=0)
            self.assertEqual(
                observation.objects["movable"].pose,
                initial_pose,
            )

    def test_legacy_translation_preserves_named_action_semantics(self) -> None:
        translator = ZerithLegacyActionTranslator(_adapter().spec)
        arm_names = tuple(
            config.name for config in ALL_SERVO_CONFIGS[:7]
        )
        action = CompositeAction(
            arm=JointDeltaAction((arm_names[2],), (0.025,)),
            gripper=GripperAction(width_m=0.04),
        )
        legacy = translator.translate(action)
        np.testing.assert_allclose(legacy[:7], [0, 0, 0.025, 0, 0, 0, 0])
        self.assertAlmostEqual(legacy[7], 0.0)

        translator.update_from_runtime_info(
            {
                "desired_q_left": np.full(7, 0.1),
                "desired_gripper_width": 0.04,
            }
        )
        position = translator.translate(
            JointPositionAction((arm_names[0],), (0.12,))
        )
        np.testing.assert_allclose(position[:7], [0.02, 0, 0, 0, 0, 0, 0])
        self.assertAlmostEqual(position[7], 0.0)

    def test_legacy_translation_rejects_unsupported_cartesian_action(self) -> None:
        adapter = _adapter()
        translator = ZerithLegacyActionTranslator(adapter.spec)
        with self.assertRaisesRegex(NotImplementedError, "PlanningQuery"):
            translator.translate(
                CartesianDeltaAction(
                    end_effector_frame=adapter.spec.end_effector_frame_name,
                    reference_frame="world",
                    translation_m=(0.0, 0.0, 0.01),
                    rotation_vector_rad=(0.0, 0.0, 0.0),
                )
            )

    def test_legacy_translation_solves_configured_cartesian_action(self) -> None:
        class FakePlanningQuery:
            """Record one Cartesian delta and return a named joint result."""

            def __init__(self) -> None:
                self.kwargs = None

            def differential_ik_step(self, **kwargs):
                self.kwargs = kwargs
                return SimpleNamespace(
                    success=True,
                    reason="success",
                    configuration=(0.01,) + (0.0,) * 8,
                    edge=SimpleNamespace(valid=True),
                )

        query = FakePlanningQuery()
        adapter = _adapter()
        translator = ZerithLegacyActionTranslator(
            adapter.spec,
            planning_query=query,
            maximum_joint_delta=0.1,
        )
        legacy = translator.translate(
            CartesianDeltaAction(
                end_effector_frame=adapter.spec.end_effector_frame_name,
                reference_frame="world",
                translation_m=(0.01, -0.02, 0.03),
                rotation_vector_rad=(0.0, 0.0, 0.0),
            )
        )
        np.testing.assert_allclose(legacy[:7], [0.01, 0, 0, 0, 0, 0, 0])
        self.assertEqual(query.kwargs["translation_m"], (0.01, -0.02, 0.03))
        self.assertEqual(query.kwargs["maximum_joint_delta"], 0.1)

    def test_cartesian_joint_delta_is_scaled_without_changing_direction(self):
        class FakePlanningQuery:
            """Return a direction-preserving bounded differential-IK step."""

            def differential_ik_step(self, **kwargs):
                self.maximum_joint_delta = kwargs["maximum_joint_delta"]
                return SimpleNamespace(
                    success=True,
                    reason="success",
                    configuration=(0.005, -0.0025) + (0.0,) * 7,
                    edge=SimpleNamespace(valid=True),
                )

        adapter = _adapter()
        query = FakePlanningQuery()
        translator = ZerithLegacyActionTranslator(
            adapter.spec,
            planning_query=query,
            maximum_joint_delta=0.005,
        )
        action = translator.translate(
            CartesianDeltaAction(
                end_effector_frame=adapter.spec.end_effector_frame_name,
                reference_frame="world",
                translation_m=(0.001, 0.0, 0.0),
                rotation_vector_rad=(0.0, 0.0, 0.0),
            )
        )
        np.testing.assert_allclose(action[:2], [0.005, -0.0025])
        self.assertEqual(query.maximum_joint_delta, 0.005)

    def test_joint_delta_adjustment_reports_reason(self) -> None:
        adapter = _adapter()
        translator = ZerithLegacyActionTranslator(
            adapter.spec,
            maximum_joint_delta=0.01,
        )
        action = translator.translate(
            JointDeltaAction(
                (adapter.spec.controlled_joint_names[0],),
                (0.2,),
            )
        )
        self.assertAlmostEqual(action[0], 0.01)
        self.assertEqual(translator.last_decision["status"], "adjusted")
        self.assertIn(
            "maximum_joint_delta",
            translator.last_decision["reasons"],
        )

    def test_colliding_cartesian_edge_is_rejected_as_hold(self) -> None:
        class FakePlanningQuery:
            """Return one expected online collision rejection."""

            def differential_ik_step(self, **kwargs):
                del kwargs
                return SimpleNamespace(
                    success=False,
                    reason="edge_collision_or_clearance",
                    joint_delta_scaled=False,
                    edge=SimpleNamespace(valid=False),
                )

        adapter = _adapter()
        translator = ZerithLegacyActionTranslator(
            adapter.spec,
            planning_query=FakePlanningQuery(),
            maximum_joint_delta=0.01,
        )
        action = translator.translate(
            CartesianDeltaAction(
                end_effector_frame=adapter.spec.end_effector_frame_name,
                reference_frame="world",
                translation_m=(0.01, 0.0, 0.0),
                rotation_vector_rad=(0.0, 0.0, 0.0),
            )
        )
        np.testing.assert_allclose(action[:7], np.zeros(7))
        self.assertEqual(action[7], 1.0)
        self.assertEqual(translator.last_decision["status"], "rejected")
        self.assertEqual(
            translator.last_decision["reasons"],
            ("cartesian_edge_collision_or_clearance",),
        )


if __name__ == "__main__":
    unittest.main()
