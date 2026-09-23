"""Geometry containment and actual robot regressions for planning spheres."""
import itertools
import json
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial import ConvexHull
from scripts.refine_cutamp_inner_spheres import expand_inner_spheres, augment_inner_spheres

REPO = Path(__file__).resolve().parents[1]


class SphereRefinementTest(unittest.TestCase):
    def test_enlargement_contains_original_and_stays_in_convex_body(self):
        vertices = np.array(list(itertools.product((-1., 1.), repeat=3)))
        original = [{'center': [0., 0., 0.], 'radius': .1},
                    {'center': [.99, 0., 0.], 'radius': .02}]
        refined, audit = expand_inner_spheres(original, vertices)
        self.assertEqual(original[0]['radius'], .1)
        self.assertAlmostEqual(refined[0]['radius'], 1 - 1e-8)
        self.assertEqual(refined[1], original[1])  # Existing surface sphere retained.
        planes = ConvexHull(vertices).equations
        for sphere, item in zip(refined, audit):
            if item['expanded']:
                self.assertLessEqual(np.max(planes[:, :3] @ sphere['center'] + planes[:, 3]
                                           + sphere['radius']), 0)

    def test_added_spheres_are_contained_and_selection_is_rigid_covariant(self):
        vertices = np.array(list(itertools.product((-1., 1.), repeat=3)))
        points, normals = [], []
        for axis in range(3):
            for sign in (-1., 1.):
                for a, b in itertools.product((-.7, 0., .7), repeat=2):
                    p = np.zeros(3); p[axis] = sign
                    p[[j for j in range(3) if j != axis]] = [a, b]
                    n = np.zeros(3); n[axis] = sign
                    points.append(p); normals.append(n)
        points, normals = np.array(points), np.array(normals)
        spheres = [{'center': [0., 0., 0.], 'radius': .1}]
        result, report = augment_inner_spheres(spheres, vertices, points, normals, 4)
        self.assertEqual(len(result), 5)
        self.assertLess(report['mean_uncovered_depth_after_m'], report['mean_uncovered_depth_before_m'])
        self.assertLessEqual(report['max_added_sphere_outside_halfspace_m'], 1e-9)
        # Use a translated non-symmetric fixture to check geometry-frame handling.
        shift = np.array([2., 3., -1.])
        shifted = [{'center': shift.tolist(), 'radius': .1}]
        other, _ = augment_inner_spheres(shifted, vertices + shift, points + shift, normals, 4)
        for s in other[1:]:
            self.assertLessEqual(np.max(np.abs(np.array(s['center']) - shift) + s['radius']), 1+1e-9)

    def test_invalid_geometry_and_budget_rejected(self):
        vertices = np.array(list(itertools.product((-1., 1.), repeat=3)))
        with self.assertRaises(ValueError):
            expand_inner_spheres([{'center': [0., 0., 0.], 'radius': float('nan')}], vertices)
        with self.assertRaises(ValueError):
            augment_inner_spheres([], vertices, np.zeros((1, 3)), np.ones((1, 3)), 1)

    def test_actual_robot_bad_grasp_and_valid_parked_grasp(self):
        from pydrake.all import DiagramBuilder, AddMultibodyPlantSceneGraph, Parser
        config = json.loads((REPO / 'experiments/cutamp/robot.json').read_text())['kinematics']
        fixtures = json.loads((REPO / 'tests/fixtures/cutamp_sphere_regression.json').read_text())
        builder = DiagramBuilder()
        plant, scene = AddMultibodyPlantSceneGraph(builder, time_step=.001)
        parser = Parser(plant)
        parser.package_map().AddPackageXml(str(REPO / 'models/zerith_drake/package.xml'))
        instance = parser.AddModels(str(REPO / 'models/zerith_drake/urdf/zerith_drake.urdf'))[0]
        plant.Finalize(); diagram = builder.Build(); context = diagram.CreateDefaultContext()
        pc = plant.GetMyMutableContextFromRoot(context)
        pairs = [('body_yaw_link', 'left_shoulder_roll_link'), ('body_pitch_link', 'left_elbow_link')]
        for fixture in fixtures['configurations']:
            values = {**fixture['lock_joints'], **dict(zip(fixture['joint_names'], fixture['q']))}
            q = plant.GetPositions(pc)
            for name, value in values.items():
                q[plant.GetJointByName(name, instance).position_start()] = value
            plant.SetPositions(pc, q)
            query = scene.get_query_output_port().Eval(scene.GetMyContextFromRoot(context))
            for a, b in pairs:
                def spheres(name):
                    body = plant.GetBodyByName(name, instance)
                    pose = plant.EvalBodyPoseInWorld(pc, body)
                    rows = config['collision_spheres'][name]
                    return np.array([pose @ np.array(s['center']) for s in rows]), np.array([s['radius'] for s in rows])
                ca, ra = spheres(a); cb, rb = spheres(b)
                gap = float((np.linalg.norm(ca[:, None] - cb, axis=2) - ra[:, None] - rb).min())
                ga = plant.GetCollisionGeometriesForBody(plant.GetBodyByName(a, instance))
                gb = plant.GetCollisionGeometriesForBody(plant.GetBodyByName(b, instance))
                exact = min(query.ComputeSignedDistancePairClosestPoints(x, y).distance for x in ga for y in gb)
                with self.subTest(case=fixture['name'], pair=(a,b)):
                    self.assertEqual(exact < 0, fixture['penetrating'])
                    self.assertEqual(gap < 0, fixture['penetrating'])


if __name__ == '__main__':
    unittest.main()
