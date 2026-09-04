"""Robot-independent integration tests for PlanningQuery."""

import math
import unittest
from pathlib import Path

import numpy as np
from pydrake.all import RigidTransform
from pydrake.planning import RobotDiagramBuilder

from src.online_manipulation import (
    GripperSpec,
    JointSpec,
    ObservedBodySpec,
    PairContactPolicy,
    PlanningQuery,
    Pose,
    RobotSpec,
)

_ROBOT_URDF = """
<robot name="test_robot">
  <link name="base">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0.4 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <collision name="arm_collision">
      <origin xyz="0.4 0 0"/>
      <geometry><box size="0.8 0.1 0.1"/></geometry>
    </collision>
  </link>
  <joint name="joint" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2"/>
  </joint>
</robot>
"""

_OBSTACLE_SDF = """
<sdf version="1.7">
  <model name="obstacle">
    <static>true</static>
    <pose>0.75 0 0 0 0 0</pose>
    <link name="body">
      <collision name="obstacle_collision">
        <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

_MOVABLE_SDF = """
<sdf version="1.7">
  <model name="movable">
    <pose>2 2 0 0 0 0</pose>
    <link name="body">
      <inertial>
        <mass>1</mass>
        <inertia>
          <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="movable_collision">
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


class _TestRobotAdapter:
    """Minimal one-joint adapter used by generic planning tests."""

    def __init__(self) -> None:
        self._spec = RobotSpec(
            name="test",
            model_instance_name="test_robot",
            package_name="unused",
            model_path=Path("unused.urdf"),
            base_link_name="base",
            base_pose=Pose((0, 0, 0), (1, 0, 0, 0)),
            controlled_joints=(
                JointSpec(
                    "joint",
                    "revolute",
                    -3.14,
                    3.14,
                    2.0,
                    10.0,
                    20.0,
                    5.0,
                ),
            ),
            locked_joint_positions={},
            end_effector_frame_name="arm",
            home_positions=(math.pi / 2.0,),
            gripper=GripperSpec(("unused_finger",), 0.0, 0.1),
        )

    @property
    def spec(self):
        return self._spec

    def initialize_state(self, plant, plant_context, model_instance) -> None:
        joint = plant.GetJointByName("joint", model_instance)
        positions = plant.GetPositions(plant_context).copy()
        positions[joint.position_start()] = math.pi / 2.0
        plant.SetPositions(plant_context, positions)


def _planning_query() -> PlanningQuery:
    """Build a one-joint robot beside a fixed obstacle."""
    builder = RobotDiagramBuilder(time_step=0.001)
    parser = builder.parser()
    plant = builder.plant()
    robot = parser.AddModelsFromString(_ROBOT_URDF, "urdf")[0]
    parser.AddModelsFromString(_OBSTACLE_SDF, "sdf")
    parser.AddModelsFromString(_MOVABLE_SDF, "sdf")
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base", robot),
        RigidTransform(),
    )
    joint = plant.GetJointByName("joint", robot)
    plant.AddJointActuator("joint_actuator", joint, 10.0)
    plant.Finalize()
    diagram = builder.Build()
    return PlanningQuery(
        diagram=diagram,
        plant=plant,
        robot_model_instance=robot,
        robot_adapter=_TestRobotAdapter(),
        observed_bodies=(
            ObservedBodySpec("target", "movable", "body"),
            ObservedBodySpec("fixed", "obstacle", "body"),
        ),
    )


class PlanningQueryTest(unittest.TestCase):
    """Validate FK, IK, collision layers, and dense edge checking."""

    def test_configuration_and_edge_collision_checks(self) -> None:
        query = _planning_query()
        safe = query.check_configuration([math.pi / 2.0])
        colliding = query.check_configuration([0.0])
        self.assertTrue(safe.valid)
        self.assertFalse(colliding.valid)
        self.assertLess(
            colliding.clearance.minimum_nonpenetration_distance_m,
            0.0,
        )
        edge = query.check_edge(
            [math.pi / 2.0],
            [0.0],
            maximum_joint_step=0.05,
        )
        self.assertFalse(edge.valid)
        self.assertGreater(edge.sample_count, 2)

    def test_allowed_contact_only_relaxes_safety_layer(self) -> None:
        query = _planning_query()
        collision = query.collision_pairs([0.0])[0]
        policy = PairContactPolicy.from_pairs(
            "allow_test_contact",
            [(collision.body_a, collision.body_b)],
        )
        clearance = query.clearance([0.0], contact_policy=policy)
        self.assertLess(clearance.minimum_nonpenetration_distance_m, 0.0)
        self.assertEqual(clearance.minimum_safety_clearance_m, 0.05)

    def test_pose_ik_uses_independent_context(self) -> None:
        query = _planning_query()
        target = Pose(
            (0.0, 0.0, 0.0),
            (
                math.cos(math.pi / 4.0),
                0.0,
                0.0,
                math.sin(math.pi / 4.0),
            ),
        )
        result = query.solve_ik(
            target,
            seed=[math.pi / 2.0],
            position_tolerance_m=0.001,
            orientation_tolerance_rad=0.01,
        )
        self.assertTrue(result.success, result)
        self.assertAlmostEqual(result.configuration[0], math.pi / 2.0, places=2)
        self.assertLess(result.orientation_error_rad, 0.01)
        self.assertEqual(query.joint_limits()["joint"], (-3.14, 3.14))
        self.assertEqual(query.body_pose("obstacle", "body").translation_m[0], 0.75)

    def test_observed_free_body_pose_can_follow_runtime_state(self) -> None:
        query = _planning_query()
        updated = Pose((2.0, 1.0, 0.5), (1.0, 0.0, 0.0, 0.0))
        fixed = query.body_pose("obstacle", "body")
        query.set_observed_body_poses({"target": updated})
        self.assertEqual(query.body_pose("movable", "body"), updated)
        self.assertEqual(query.body_pose("obstacle", "body"), fixed)

        query.check_configuration([math.pi / 2.0])
        self.assertEqual(query.body_pose("movable", "body"), updated)

        with self.assertRaisesRegex(KeyError, "Unknown observed body"):
            query.set_observed_body_poses({"undeclared": updated})

    def test_differential_ik_returns_bounded_collision_checked_edge(self):
        query = _planning_query()
        result = query.differential_ik_step(
            translation_m=(0.0, 0.0, 0.0),
            rotation_vector_rad=(0.0, 0.0, 0.01),
            frame_name="arm",
            seed=(math.pi / 2.0,),
            maximum_joint_delta=0.02,
        )
        self.assertTrue(result.success, result)
        self.assertIsInstance(result.joint_delta_scaled, bool)
        self.assertLessEqual(
            abs(result.configuration[0] - math.pi / 2.0),
            0.02,
        )
        self.assertTrue(result.edge.valid)
        self.assertAlmostEqual(result.achieved_twist[2], 0.01, places=5)

    def test_differential_ik_rejects_a_colliding_edge(self) -> None:
        query = _planning_query()
        result = query.differential_ik_step(
            translation_m=(0.0, 0.0, 0.0),
            rotation_vector_rad=(0.0, 0.0, 0.01),
            frame_name="arm",
            seed=(0.0,),
            maximum_joint_delta=0.02,
        )
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "edge_collision_or_clearance")
        self.assertLess(
            result.edge.minimum_nonpenetration_distance_m,
            0.0,
        )


if __name__ == "__main__":
    unittest.main()
