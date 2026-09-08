"""Real two-mode cancellation/frames plus conservative planner edge cases."""

import dataclasses
import math
import unittest
import numpy as np
from pydrake.all import RigidTransform, RollPitchYaw

from src.online_manipulation.recipes.mobile import make_config
from src.online_manipulation import (
    BaseVelocityAction,
    HoldAction,
    NavigationGoal,
    Navigator,
    Pose,
    StaticNavigationMap,
    make_env,
)
from src.online_manipulation.contact import (
    PairContactPolicy,
    SupportContactPolicy,
    penetration_limit,
)


class NavigationTest(unittest.TestCase):
    """Verify measured cancellation, ownership and full-frame goal resolution."""

    def test_two_modes_cancel_stop_and_local_pose_freeze(self):
        for mode in ("planar_kinematic", "wheel_dynamic"):
            with self.subTest(mode=mode):
                env = make_env(make_config(mode))
                obs, _ = env.reset(0)
                self.assertEqual(
                    obs.base["odom_from_base_link"],
                    obs.robot.frame_poses_world[obs.base["base_link_frame"]].as_dict(),
                )
                self.assertEqual(
                    obs.base["world_from_odom"]["translation_m"], [0, 0, 0]
                )
                for _ in range(15):
                    obs, *_ = env.step(HoldAction())
                nav = Navigator(StaticNavigationMap((), 0.8))
                reference = obs.robot.frame_poses_world[env.backend.spec.base_link_name]
                q = np.array(reference.quaternion_wxyz)
                from pydrake.all import Quaternion

                X_WB = RigidTransform(
                    Quaternion(q / np.linalg.norm(q)), reference.translation_m
                )
                X_WG = RigidTransform(RollPitchYaw(0, 0, 0.4), [1, 0.2, 0])
                X_BG = X_WB.inverse() @ X_WG
                local = NavigationGoal(
                    Pose(
                        tuple(X_BG.translation()),
                        tuple(X_BG.rotation().ToQuaternion().wxyz()),
                    ),
                    frame_id=env.backend.spec.base_link_name,
                )
                nav.set_goal(local, obs)
                saved = nav.world_goal
                np.testing.assert_allclose(saved.translation_m, (1, 0.2, 0), atol=1e-10)
                for _ in range(15):
                    obs, *_ = env.step(nav.act(obs))
                self.assertEqual(saved, nav.world_goal)
                obs, _, _, _, info = env.step(BaseVelocityAction(0.1, 0))
                self.assertFalse(info["action_decision"]["accepted"])
                self.assertEqual(obs.base["control_owner"], "navigation")
                nav.cancel()
                self.assertEqual(nav.status, "cancelling")
                for _ in range(30):
                    obs, *_ = env.step(nav.act(obs))
                    if nav.status == "cancelled":
                        break
                self.assertEqual(nav.status, "cancelled")
                self.assertLessEqual(
                    np.linalg.norm(obs.base["linear_velocity_world_m_s"][:2]), 0.01
                )
                self.assertLessEqual(abs(obs.base["yaw_rate_rad_s"]), 0.02)
                obs, _, _, _, info = env.step(
                    BaseVelocityAction(0, 0, "navigation", release_control=True)
                )
                self.assertTrue(info["action_decision"]["accepted"])
                self.assertIsNone(obs.base["control_owner"])
                nav.release()
                bad = NavigationGoal(Pose((0, 0, 0.1), (1, 0, 0, 0)))
                with self.assertRaisesRegex(ValueError, "not on"):
                    nav.set_goal(bad, obs)

    def test_obstacle_sweep_reverse_and_no_path(self):
        map_ = StaticNavigationMap(
            ((-0.1, -0.5, 0.1, 0.5),), 0.4, bounds=(-2, -2, 2, 2)
        )
        self.assertFalse(map_.edge_free((-1, 0), (1, 0)))
        path = map_.plan((-1, 0), (1, 0))
        self.assertIsNotNone(path)
        for start, end in zip(path, path[1:]):
            self.assertTrue(map_.edge_free(end, start))
        blocked = dataclasses.replace(map_, obstacles=((-0.1, -2, 0.1, 2),))
        self.assertIsNone(blocked.plan((-1, 0), (1, 0)))
        # Corridor width .6m cannot fit the .8m-diameter full-robot disk.
        narrow = dataclasses.replace(
            map_, obstacles=((-2, 0.3, 2, 2), (-2, -2, 2, -0.3))
        )
        self.assertIsNone(narrow.plan((-1, 0), (1, 0)))

    def test_support_bound_does_not_relax_grasp_contacts(self):
        task = PairContactPolicy.from_pairs(
            "grasp", [("finger", "target")], maximum_allowed_penetration_m=0.0001
        )
        combined = SupportContactPolicy(
            task, {tuple(sorted(("wheel", "floor"))): 0.002}
        )
        self.assertEqual(penetration_limit(combined, "wheel", "floor"), 0.002)
        self.assertEqual(penetration_limit(combined, "finger", "target"), 0.0001)
        self.assertEqual(penetration_limit(combined, "wheel", "obstacle"), 0)


if __name__ == "__main__":
    unittest.main()
