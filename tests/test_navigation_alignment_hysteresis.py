"""Tight parking tolerances must retain margin for final-yaw alignment drift."""
import math
from types import SimpleNamespace
import unittest

from simulation.src import NavigationGoal, Navigator, Pose, StaticNavigationMap
from simulation.src.control.navigation import NavigationConfig


def observation(distance, yaw=0., time_s=0.):
    return SimpleNamespace(time_s=time_s, base={
        'pose': Pose((-distance, 0., 0.),
                     (math.cos(yaw/2), 0., 0., math.sin(yaw/2))).as_dict(),
        'linear_velocity_world_m_s': [0., 0., 0.],
        'yaw_rate_rad_s': 0., 'blocked': False})


class NavigationAlignmentTest(unittest.TestCase):
    def test_tight_tolerance_preserves_alignment_margin_and_arrival_gate(self):
        nav = Navigator(StaticNavigationMap((), 0.),
                        NavigationConfig(position_tolerance_m=.005))
        goal = NavigationGoal(Pose((0., 0., 0.),
            (math.cos(math.pi/4), 0., 0., math.sin(math.pi/4))))
        nav.set_goal(goal, observation(.1))
        # Approaching the 5-mm boundary must leave room for turn-induced drift.
        action = nav.act(observation(.0045, time_s=.1))
        self.assertGreater(action.velocity_m_s, 0.)
        self.assertFalse(nav.aligning_final_yaw)
        action = nav.act(observation(.003, time_s=.2))
        self.assertEqual(action.velocity_m_s, 0.)
        self.assertTrue(nav.aligning_final_yaw)
        self.assertGreater(action.yaw_rate_rad_s, 0.)
        # Drift inside the original arrival tolerance keeps the alignment phase.
        action = nav.act(observation(.0045, yaw=.3, time_s=.3))
        self.assertEqual(action.velocity_m_s, 0.)
        self.assertTrue(nav.aligning_final_yaw)
        # Outside 5 mm, translation must resume; success tolerance is unchanged.
        nav.act(observation(.0055, yaw=math.pi/2, time_s=.4))
        self.assertFalse(nav.aligning_final_yaw)
        self.assertEqual(nav.status, 'tracking')
        nav.act(observation(.0045, yaw=math.pi/2, time_s=.5))
        nav.act(observation(.0045, yaw=math.pi/2, time_s=1.1))
        self.assertEqual(nav.status, 'arrived')


if __name__ == '__main__':
    unittest.main()
