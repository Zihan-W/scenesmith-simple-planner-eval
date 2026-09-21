"""Replay saved, fully grounded actions in the unmodified simulation runtime."""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
from examples.online_manipulation.tamp_diagnostics import DiagnosticRecorder, WorkMeter, json_value, state_record
from examples.online_manipulation.tamp_hierarchy import ParameterizedSkillAction
from examples.online_manipulation.tamp_online import JsonlTrace
from examples.online_manipulation.tamp_scenesmith_online import SceneSmithSkillExecutor
from examples.online_manipulation.tamp_scenesmith import SceneSmithPickDomain
from src.online_manipulation import make_env, HoldAction
from src.online_manipulation.experiment import load_experiment

def saved_actions(path):
    actions=[];checks={}
    for line in path.read_text().splitlines():
        row=json.loads(line)
        if row['event']=='skill_skeleton':steps=row['steps'];checks={}
        elif row['event']=='ccsp_assignment':candidate=row['instantiated_steps'];checks={}
        elif row['event']=='ccsp_constraint_check' and row['feasible']:checks[row['step']]=row['details']
        elif row['event']=='ccsp_solved':
            for index,step in enumerate(steps):
                actions.append(ParameterizedSkillAction(step['skill'],step['arguments'],
                    {**candidate[index],**row['assignments'],'checks':checks[index]},(),True,step['continuous_variables']))
            if any(a.skill_name=='PickLift' for a in actions):break
    return actions

def domain_for(experiment,repo,observation):
    overrides=experiment.resolved_config['user_config']['policy_options'].get('expert_policy_overrides',{})
    return SceneSmithPickDomain(environment_config=experiment.environment_config,observation=observation,
        calibration_path=repo/'experiments/inputs/pick_lift/pick_lift_calibration.json',
        pick_home_path=repo/'experiments/inputs/pick_lift/pick_home.json',
        open_width_m=overrides.get('open_width_m'),lift_distance_m=overrides.get('lift_distance_m'))

def outcome_record(result):
    return {name:json_value(getattr(result,name)) for name in ('success','reason','duration_s','episode_finished','retryable','failure_details')}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seed',type=int,required=True);parser.add_argument('--repeat',type=int,default=1)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--grasp-offset',type=float);args=parser.parse_args()
    repo=Path.cwd();out=args.output;out.mkdir(parents=True,exist_ok=False)
    source=repo/f'runs/seed500-501-fix-20260920/live_seed{args.seed}_5mm/runtime'
    actions=saved_actions(source/'tamp_trace.jsonl'); nav=actions[0]
    (out/'frozen_actions.json').write_text(json.dumps(json_value(actions),indent=2))
    experiment=load_experiment(repo/'experiments/navigation_picklift_tamp.json',repository_root=repo,
        cache_root=Path('/dev/shm/contact-parking-diagnostic-20260920')/out.name,scene_root=Path('/root/scenesmith-simple-planner-eval/scene/scene_000'),trust_factories=True,meshcat=False)
    env=make_env(experiment.environment_config);meter=WorkMeter();summary=[]
    original=[json.loads(l) for l in (source/'skill_steps.jsonl').read_text().splitlines()]
    original_nav=[r for r in original if r['skill_index']==1]
    for repeat in range(args.repeat):
        folder=out/f'repeat{repeat}';folder.mkdir();observation,info=env.reset(seed=args.seed)
        (folder/'initial.json').write_text(json.dumps(state_record(env),indent=2))
        config=domain_for(experiment,repo,observation).task_config
        executor=SceneSmithSkillExecutor(env=env,experiment=experiment,repository_root=repo,output_root=folder,
                                        observation=observation,reset_info=info,model_called=False)
        recorder=DiagnosticRecorder(env,config,JsonlTrace(folder/'aligned_steps.jsonl'),meter)
        started=time.perf_counter()
        with meter.instrument(),recorder.install():
            with meter.span('navigation_execution'):navresult=executor.execute(nav)
            arrival=state_record(env);(folder/'arrival.json').write_text(json.dumps(arrival,indent=2))
            actual_nav=[json.loads(l) for l in (folder/'skill_steps.jsonl').read_text().splitlines()]
            comparison={'original_steps':len(original_nav),'replay_steps':len(actual_nav),'same_times':False}
            if len(actual_nav)==len(original_nav):
                comparison['same_times']=all(a['time_s']==b['time_s'] for a,b in zip(actual_nav,original_nav))
                for key in ('q','v','q_commanded'):
                    comparison[key+'_max_abs_difference']=max(float(np.max(np.abs(np.array(a['robot'][key])-b['robot'][key]))) for a,b in zip(actual_nav,original_nav))
            entry={'repeat':repeat,'navigation':outcome_record(navresult),'prefix_comparison':comparison}
            if args.seed==500 and navresult.success:
                pick_action=next(a for a in actions if a.skill_name=='PickLift')
                if args.grasp_offset is not None:
                    from examples.online_manipulation.tamp_planner import Subgoal
                    from src.online_manipulation import Pose
                    domain=domain_for(experiment,repo,env.observation)
                    p=env.observation.base['base_link_pose']
                    state={'base_pose':Pose(tuple(p['translation_m']),tuple(p['quaternion_wxyz']))}
                    params={k:pick_action.geometric_parameters[k] for k in ('target','arm','grasp_lateral_offset_m','approach_height_offset_m','approach_segments')}
                    params['grasp_lateral_offset_m']=args.grasp_offset
                    feasible,reason,checks=domain.check(Subgoal('PickLift',{'target':domain.target_name}),params,state)
                    (folder/'counterfactual_grasp_check.json').write_text(json.dumps({'parameters':params,'feasible':feasible,'reason':reason,'checks':checks},indent=2))
                    if not feasible:
                        entry['diagnostic_grasp_search']={'feasible':False,'reason':reason,'budget':1};summary.append(entry)
                        (out/'summary.json').write_text(json.dumps(summary,indent=2));(out/'costs.json').write_text(json.dumps(meter.as_dict(),indent=2));continue
                    geometry={**params,'g0':{'lateral_offset_m':args.grasp_offset,'ik':checks['ik']['grasp_pose_in_target']},'a0':checks['ik']['staging_pose_in_target'],'checks':checks}
                    pick_action=ParameterizedSkillAction('PickLift',pick_action.symbolic_args,geometry,(),True,pick_action.parameter_bindings)
                from src.online_manipulation import Pose
                calibration=json.loads((repo/'experiments/inputs/pick_lift/pick_lift_calibration.json').read_text())
                for label in ('staging_pose_in_target','grasp_pose_in_target'):
                    p=calibration[label];xyz=list(p['translation_m']);xyz[1]+=pick_action.geometric_parameters['grasp_lateral_offset_m']
                    if label=='staging_pose_in_target':xyz[2]+=pick_action.geometric_parameters['approach_height_offset_m']
                    recorder.relative_targets[label]=Pose(tuple(xyz),tuple(p['quaternion_wxyz']))
                with meter.span('fixed_grasp_execution'):pickresult=executor.execute(pick_action)
                entry['pick']=outcome_record(pickresult)
            elif args.seed==501 and navresult.success:
                states=[arrival]
                for i in range(30):
                    env.step(HoldAction());states.append(state_record(env))
                (folder/'settling_states.json').write_text(json.dumps(states))
            entry.update(wall_s=time.perf_counter()-started,final=state_record(env),mode='DIAGNOSTIC_FIXED_GRASP',online_model_success_trial=False)
            summary.append(entry)
        (out/'summary.json').write_text(json.dumps(summary,indent=2));(out/'costs.json').write_text(json.dumps(meter.as_dict(),indent=2))
        print(json.dumps({'repeat':repeat,'nav':navresult.reason,'pick':entry.get('pick',{}).get('reason'),'prefix':comparison,'wall_s':entry['wall_s']}),flush=True)

if __name__=='__main__':main()
