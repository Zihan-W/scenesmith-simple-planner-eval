"""Measured image identities, semantic attributes and parked-state verification."""

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from planner.src.tamp.hierarchy import PredicateGoal
from planner.src.tamp.scenesmith_online import SceneSmithWorldObserver
from simulation.src import Pose


class ObservationMetadataTest(unittest.TestCase):
    def test_exact_mask_identity_and_semantic_attributes(self):
        bodies = [SimpleNamespace(observation_name=name, model_instance_name=model,
                                  body_name="link")
                  for name, model in (("cube", "thing"), ("bin", "thing_extra"))]
        task = SimpleNamespace(config=SimpleNamespace(
            target_contact_body="thing::link", support_contact_bodies=("thing_extra::link",)))
        camera = SimpleNamespace(
            timestamp_s=0.5, rgb=np.zeros((12, 12, 3), dtype=np.uint8),
            label=np.zeros((12, 12), dtype=np.int16),
            label_names={1: "thing::link", 2: "thing_extra::link"},
        )
        camera.label[2:5, 2:5] = 1
        camera.label[8:11, 8:11] = 2
        executor = SimpleNamespace(
            experiment=SimpleNamespace(environment_config=SimpleNamespace(
                task_factory=lambda: task, scenario=SimpleNamespace(observed_bodies=bodies))),
            env=SimpleNamespace(capture_cameras=lambda: {"head_camera": camera}),
            last_navigation_goal=None, last_info={},
        )
        pose = Pose((0, 0, 0), (1, 0, 0, 0))
        observation = SimpleNamespace(
            time_s=0.5, objects={name: SimpleNamespace(pose=pose) for name in ("cube", "bin")},
            task={}, robot=SimpleNamespace(joint_names=(), q=(), gripper_widths_m={}),
            base={"pose": pose.as_dict(), "linear_velocity_world_m_s": [0.1, 0, 0],
                  "frame": "navigation_frame", "base_link_frame": "dipan_link",
                  "base_link_pose": Pose((0.05, 0, 0.18), (1, 0, 0, 0)).as_dict(),
                  "yaw_rate_rad_s": 0, "control_owner": None},
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            metadata = root / "metadata.json"
            metadata.write_text(json.dumps({"objects": [
                {"model_instance": "thing", "semantic_category": "manipuland",
                 "static_in_sdf": False, "fixed_by_directive": False, "joints": []},
                {"model_instance": "thing_extra", "semantic_category": "container",
                 "static_in_sdf": False, "fixed_by_directive": True, "joints": []},
            ]}))
            observer = SceneSmithWorldObserver(executor, "cube", root, scene_metadata=metadata)
            world = observer.observe(observation)
            self.assertTrue(world.objects["cube"]["movable"])
            self.assertFalse(world.objects["bin"]["movable"])
            self.assertTrue(world.objects["bin"]["surface"])
            self.assertEqual(world.robot["base_pose_frame"], "navigation_frame")
            self.assertEqual(world.robot["base_link_frame"], "dipan_link")
            self.assertNotEqual(world.robot["base_pose"], world.robot["base_link_pose"])
            images = observer.capture_images(observation)
            boxes = {item["object"]: item["bbox_xyxy"] for item in images[0]["annotations"]}
            self.assertEqual(boxes, {"cube": [2, 2, 4, 4], "bin": [8, 8, 10, 10]})
            self.assertEqual(images[0]["timestamp_s"], 0.5)
            executor.last_navigation_goal = (0, 0, 0)
            parked = PredicateGoal("at_pick_pose", ("cube",))
            self.assertNotIn(parked, observer.observe(observation).facts)
            observation.base["linear_velocity_world_m_s"] = [0, 0, 0]
            self.assertIn(parked, observer.observe(observation).facts)
            executor.experiment.resolved_config = {"user_config": {"policy_options": {
                "navigation_position_tolerance_m": 0.005}}}
            observation.base["pose"] = Pose((0.01, 0, 0), (1, 0, 0, 0)).as_dict()
            self.assertNotIn(parked, observer.observe(observation).facts)
            observation.base["pose"] = Pose((0.004, 0, 0), (1, 0, 0, 0)).as_dict()
            self.assertIn(parked, observer.observe(observation).facts)
            observation.base["control_owner"] = "navigation"
            self.assertNotIn(parked, observer.observe(observation).facts)


if __name__ == "__main__":
    unittest.main()
