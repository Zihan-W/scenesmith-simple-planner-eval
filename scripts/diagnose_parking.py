"""Planning-only observed-state sensitivity and bounded station coverage."""
import argparse,copy,dataclasses,json,random,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from pydrake.all import Quaternion,RollPitchYaw,RotationMatrix
from planner.src.tamp.diagnostics import WorkMeter,json_value
from planner.src.tamp.planner import Subgoal
from planner.src.tamp.scenesmith import _yaw_pose
from simulation.src import Pose
from simulation.src.robots.adapters.description import drake_pose,public_pose
from simulation.src.io.experiment import load_experiment
from scripts.diagnose_pick_contact import domain_for,saved_actions

def pose(p):return Pose(tuple(p['translation_m']),tuple(p['quaternion_wxyz']))
def observation(s):
    return SimpleNamespace(time_s=s['time_s'],base=s['base'],robot=SimpleNamespace(joint_names=s['robot']['joint_names'],q=s['robot']['q']),objects={name:SimpleNamespace(pose=pose(item['pose'])) for name,item in s['objects'].items()})
def rpy(s):return drake_pose(pose(s['base']['base_link_pose'])).rotation().ToRollPitchYaw().vector()
def changed_pose(s,xyz,angles):
    s['base']['base_link_pose']={'translation_m':list(xyz),'quaternion_wxyz':RollPitchYaw(angles).ToQuaternion().wxyz().tolist()}
    return s

def stability(states):
    results=[]
    for end in range(1,len(states)):
        window=[s for s in states[:end+1] if states[end]['time_s']-s['time_s']<=.5000001]
        if window[-1]['time_s']-window[0]['time_s']<.499:continue
        xyz=np.array([s['base']['base_link_pose']['translation_m'] for s in window])
        angles=np.unwrap(np.array([rpy(s) for s in window]),axis=0)
        q=np.array([s['robot']['q'] for s in window])
        obj=np.array([s['objects']['pick_target']['pose']['translation_m'] for s in window])
        objangles=np.unwrap(np.array([drake_pose(pose(s['objects']['pick_target']['pose'])).rotation().ToRollPitchYaw().vector() for s in window]),axis=0)
        changes={'base_xyz_range_m':np.ptp(xyz,axis=0).tolist(),'base_rpy_range_rad':np.ptp(angles,axis=0).tolist(),'max_joint_range':float(np.max(np.ptp(q,axis=0))),'object_xyz_range_m':np.ptp(obj,axis=0).tolist(),'object_rpy_range_rad':np.ptp(objangles,axis=0).tolist()}
        stable=max(changes['base_xyz_range_m']+changes['object_xyz_range_m'])<=.0001 and max(changes['base_rpy_range_rad']+changes['object_rpy_range_rad'])<=.001 and changes['max_joint_range']<=.0001
        results.append({'index':end,'time_s':states[end]['time_s'],'stable':stable,**changes})
    matches=[r for r in results if r['stable']]
    return {'windows':results,'first_stable_index':matches[0]['index'] if matches else None}

def evaluate(experiment,repo,s,parameters,meter):
    domain=domain_for(experiment,repo,observation(s));p=pose(s['base']['base_link_pose'])
    state={'base_pose':p,'base_height_m':experiment.environment_config.robot_adapter.base_config.base_height_m}
    return domain,state

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--mode',choices=['sensitivity','coverage'],default='sensitivity');ap.add_argument('--candidate-rule',choices=['existing_sampler','observed_error_offsets'],default='existing_sampler');args=ap.parse_args()
    repo=Path.cwd();out=args.output;out.mkdir(exist_ok=False,parents=True);meter=WorkMeter()
    exp=load_experiment(repo/'experiments/navigation_picklift_tamp.json',repository_root=repo,cache_root=Path('/dev/shm/contact-parking-diagnostic-20260920')/out.name,scene_root=Path('/root/scenesmith-simple-planner-eval/scene/scene_000'),trust_factories=True,meshcat=False)
    initial=json.loads((args.source/'repeat0/initial.json').read_text());arrival=json.loads((args.source/'repeat0/arrival.json').read_text());settling=json.loads((args.source/'repeat0/settling_states.json').read_text())
    stable=stability(settling);(out/'stability.json').write_text(json.dumps(stable,indent=2))
    stable_index=stable['first_stable_index'];settled=settling[stable_index] if stable_index is not None else settling[-1]
    nav=saved_actions(repo/'runs/seed500-501-fix-20260920/live_seed501_5mm/runtime/tamp_trace.jsonl')[0]
    nominal=copy.deepcopy(initial);xyz=[nav.geometric_parameters['base_x_m'],nav.geometric_parameters['base_y_m'],exp.environment_config.robot_adapter.base_config.base_height_m];angles=[0,0,nav.geometric_parameters['base_yaw_rad']];changed_pose(nominal,xyz,angles)
    witness=nav.geometric_parameters['checks']['pick_witness'];fixed=witness['parameters']
    base_domain,state=evaluate(exp,repo,nominal,fixed,meter);pick=Subgoal('PickLift',{'target':base_domain.target_name});rng=random.Random(91401)
    candidates=[base_domain.sample_candidate(pick,state,rng) for _ in range(16)]
    (out/'frozen_inputs.json').write_text(json.dumps({'nominal':nominal,'arrival':arrival,'settled':settled,'stable_criterion_met':stable_index is not None,'original_witness':witness,'paired_search_candidates':candidates},indent=2))
    factors={'nominal':nominal,'arrival':arrival,'stable' if stable_index is not None else 'bounded_wait_end_unstable':settled}
    actual_xyz=arrival['base']['base_link_pose']['translation_m'];actual_angles=rpy(arrival)
    for name,indices in [('xy',(0,1)),('z',(2,))]:
        vector=list(xyz)
        for i in indices:vector[i]=actual_xyz[i]
        factors['only_'+name]=changed_pose(copy.deepcopy(nominal),vector,angles)
    for name,indices in [('yaw',(2,)),('roll_pitch',(0,1))]:
        vector=list(angles)
        for i in indices:vector[i]=actual_angles[i]
        factors['only_'+name]=changed_pose(copy.deepcopy(nominal),xyz,vector)
    for name in ('robot','objects'):
        s=copy.deepcopy(nominal);s[name]=copy.deepcopy(arrival[name]);factors['only_'+name]=s
    factors['all_base_components']=changed_pose(copy.deepcopy(nominal),actual_xyz,actual_angles)
    (out/'factor_states.json').write_text(json.dumps(factors,indent=2))
    def write(row):
        with (out/'results.jsonl').open('a') as stream:stream.write(json.dumps(json_value(row))+'\n')
        (out/'costs.json').write_text(json.dumps(meter.as_dict(),indent=2))
        print(json.dumps({k:v for k,v in row.items() if k not in ('details','state')}),flush=True)
    with meter.instrument():
        if args.mode=='sensitivity':
            for name,s in factors.items():
                domain,state=evaluate(exp,repo,s,fixed,meter)
                for kind,params in [('A_fixed_grasp_parameters',fixed),('A_frozen_grasp_joints',{**fixed,'grasp_arm_joint_positions':witness['checks']['ik']['grasp_pose_in_target']['arm_joint_positions']})]:
                    started=time.perf_counter();ok,reason,details=domain.check(pick,params,state)
                    write({'state_name':name,'test':kind,'feasible':ok,'reason':reason,'details':details,'wall_s':time.perf_counter()-started})
                for i,candidate in enumerate(candidates):
                    started=time.perf_counter();ok,reason,details=domain.check(pick,candidate,state)
                    write({'state_name':name,'test':'B_resample_16','trial':i+1,'budget':16,'feasible':ok,'reason':reason,'details':details,'wall_s':time.perf_counter()-started})
                    if ok:break
        else:
            navgoal=Subgoal('NavigateToPick',{'target':base_domain.target_name});rng=random.Random(91401)
            stations=[{k:nav.geometric_parameters[k] for k in ('base_x_m','base_y_m','base_yaw_rad')}]+[base_domain.sample_candidate(navgoal,state,rng) for _ in range(2)]
            if args.candidate_rule=='observed_error_offsets':
                error=np.array(arrival['base']['base_link_pose']['translation_m'][:2])-np.array(xyz[:2])
                stations=[{'base_x_m':xyz[0]-scale*error[0],'base_y_m':xyz[1]-scale*error[1],'base_yaw_rad':angles[2]} for scale in (0,1,2)]
            # Physical observations at completion, +1 s, +2 s, and bounded settled/end.
            observed=[nominal,arrival,settling[10],settling[20],settled]
            (out/'coverage_protocol.json').write_text(json.dumps({'candidate_rule':args.candidate_rule,'stations':stations,'observed_state_times_s':[s['time_s'] for s in observed],'per_state_full_witness_budget':4,'paired_parameters':[fixed]+candidates[:3],'perturbation_source':'actual replay relative to frozen nominal; base transform and measured robot/object states','production_change':False,'continuous_guarantee':False},indent=2))
            for ci,c in enumerate(stations):
                for si,source in enumerate(observed):
                    s=copy.deepcopy(source);p=source['base']['base_link_pose'];t=np.array(p['translation_m'])+np.array([c['base_x_m']-xyz[0],c['base_y_m']-xyz[1],0]);a=rpy(source);a[2]+=c['base_yaw_rad']-angles[2];changed_pose(s,t,a)
                    domain,state=evaluate(exp,repo,s,fixed,meter);started=time.perf_counter();query,adapter=domain._candidate_query(state['base_pose']);measured=dict(zip(s['robot']['joint_names'],s['robot']['q']));configuration=[measured[n] for n in adapter.spec.controlled_joint_names]
                    cheap=query.check_configuration(configuration,contact_policy=domain._grasp_contact_policy())
                    write({'candidate':ci,'state_index':si,'test':'cheap_configuration','feasible':cheap.valid,'details':dataclasses.asdict(cheap),'wall_s':time.perf_counter()-started})
                    if not cheap.valid:continue
                    for i,params in enumerate([fixed]+candidates[:3]):
                        started=time.perf_counter();ok,reason,details=domain.check(pick,params,state)
                        write({'candidate':ci,'state_index':si,'test':'finite_state_witness','trial':i+1,'budget':4,'feasible':ok,'reason':reason,'details':details,'wall_s':time.perf_counter()-started})
                        if ok:break
    (out/'done.json').write_text(json.dumps({'complete':True,'mode':args.mode,'production_change':False}))

if __name__=='__main__':main()
