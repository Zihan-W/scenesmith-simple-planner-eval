"""Real Drake geometry regression for turns missed by fixed-yaw translation."""

import math
from types import SimpleNamespace
import unittest

from pydrake.planning import RobotDiagramBuilder

from examples.online_manipulation.bt_core import Node
from examples.online_manipulation.bt_runtime import _certified_local_map


class NavigationTurnSweepTest(unittest.TestCase):
    def check_goal(self, x, y, yaw):
        builder = RobotDiagramBuilder(time_step=0.001)
        parser = builder.parser()
        robot = parser.AddModelsFromString('''<robot name="robot"><link name="base">
          <inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
          <collision name="offset_arm"><origin xyz="0 0.4 0.5"/>
          <geometry><sphere radius="0.04"/></geometry></collision></link></robot>''', 'urdf')[0]
        parser.AddModelsFromString('''<sdf version="1.7"><model name="obstacle">
          <static>true</static><pose>-0.28284 0.28284 0.5 0 0 0</pose><link name="body">
          <collision name="corner"><geometry><sphere><radius>0.04</radius></sphere></geometry></collision>
          </link></model></sdf>''', 'sdf')
        plant = builder.plant()
        plant.Finalize()
        diagram = builder.Build()
        root = diagram.CreateDefaultContext()
        query = SimpleNamespace(plant=plant, context=plant.GetMyMutableContextFromRoot(root),
                                robot_model_instance=robot)
        adapter = SimpleNamespace(spec=SimpleNamespace(base_link_name='base'), navigation_frame_name='base')
        scenario = SimpleNamespace(ground_body_names=(), ground_geometries=())
        return _certified_local_map(query, adapter, scenario,
                                    Node('action', 'NavigateTo', (str(x), str(y), str(yaw), 'world')))

    def test_rejects_collision_only_present_during_initial_turn(self):
        with self.assertRaisesRegex(ValueError, 'alpha=0.00, yaw='):
            self.check_goal(0.5, 0.5, 0)

    def test_rejects_collision_during_final_turn_without_translation(self):
        with self.assertRaisesRegex(ValueError, 'corridor collision'):
            self.check_goal(0, 0, math.pi / 2)

    def test_reverse_does_not_require_an_unnecessary_half_turn(self):
        self.assertIsNotNone(self.check_goal(-0.5, 0, 0))

    def test_clear_straight_translation_passes(self):
        self.assertIsNotNone(self.check_goal(0.5, 0, 0))


if __name__ == '__main__':
    unittest.main()
