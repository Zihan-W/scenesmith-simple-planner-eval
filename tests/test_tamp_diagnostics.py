"""Read-only diagnostic contracts, including a real Drake contact response."""
import json
from types import MappingProxyType, SimpleNamespace
import unittest
import numpy as np
from pydrake.all import (AddMultibodyPlantSceneGraph, CoulombFriction, DiagramBuilder,
                        RigidTransform, Simulator, SpatialInertia, Sphere, UnitInertia)
from planner.src.tamp.diagnostics import measured_contacts, target_flags, json_value, WorkMeter
from simulation.src import RobotCommand, GripperAction

class DiagnosticsTest(unittest.TestCase):
    def test_pair_identity_does_not_infer_bilateral_from_other_object(self):
        result=target_flags([('target','left'),('other','right')],'target',('left','right'),('table',))
        self.assertEqual(result['finger_contacts'],[True,False])
        self.assertFalse(result['bilateral_gripper_contact'])

    def test_immutable_command_serialization(self):
        command=RobotCommand(grippers=MappingProxyType({'left':GripperAction(.02)}))
        encoded=json.loads(json.dumps(json_value(command)))
        self.assertEqual(encoded['grippers']['left']['width_m'],.02)

    def test_contact_results_are_read_only_and_geometry_has_separate_time(self):
        builder=DiagramBuilder();plant,_=AddMultibodyPlantSceneGraph(builder,time_step=.001)
        bodies=[]
        for name,x in [('target',0.),('left',.19),('right',-.5)]:
            model=plant.AddModelInstance(name)
            body=plant.AddRigidBody('body',model,SpatialInertia(1.,np.zeros(3),UnitInertia.SolidSphere(.1)))
            plant.RegisterCollisionGeometry(body,RigidTransform(),Sphere(.1),'sphere',CoulombFriction(.5,.4))
            if name!='target':plant.WeldFrames(plant.world_frame(),body.body_frame(),RigidTransform([x,0,0]))
            bodies.append(body)
        plant.mutable_gravity_field().set_gravity_vector([0,0,0]);plant.Finalize()
        diagram=builder.Build();sim=Simulator(diagram);sim.Initialize();sim.AdvanceTo(.001)
        context=plant.GetMyMutableContextFromRoot(sim.get_mutable_context())
        backend=SimpleNamespace(plant=plant,plant_context=context)
        config=SimpleNamespace(target_contact_body='target::body',gripper_contact_bodies=('left::body','right::body'),support_contact_bodies=())
        q=plant.GetPositions(context).copy();v=plant.GetVelocities(context).copy()
        data=measured_contacts(backend,config)
        np.testing.assert_array_equal(q,plant.GetPositions(context));np.testing.assert_array_equal(v,plant.GetVelocities(context))
        self.assertEqual(data['geometry_time_s'],.001)
        self.assertFalse(data['contact_timestamp_available'])
        self.assertEqual(data['contact_result_presence']['finger_contacts'],[True,False])
        self.assertGreater(np.linalg.norm(data['actual_contact_results'][0]['force_world_n']),0)
        self.assertFalse(data['geometry_presence']['bilateral_gripper_contact'])

    def test_instrumentation_installs_and_restores_runtime_methods(self):
        from simulation.src.runtime.runtime import DrakeRuntime
        from simulation.src.geometry.planning import PlanningQuery
        original_step, original_ik = DrakeRuntime.step, PlanningQuery.solve_ik
        with WorkMeter().instrument():
            self.assertIsNot(DrakeRuntime.step, original_step)
            self.assertIsNot(PlanningQuery.solve_ik, original_ik)
        self.assertIs(DrakeRuntime.step, original_step)
        self.assertIs(PlanningQuery.solve_ik, original_ik)

    def test_timing_nested_exclusive_is_additive(self):
        meter=WorkMeter()
        with meter.span('outer'):
            with meter.span('inner'):sum(range(1000))
        stats=meter.as_dict()['timings']
        self.assertAlmostEqual(stats['outer']['inclusive_s'],stats['outer']['exclusive_s']+stats['inner']['exclusive_s'])

if __name__=='__main__':unittest.main()
