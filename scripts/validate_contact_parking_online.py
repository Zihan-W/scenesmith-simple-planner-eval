"""Real-model online validation with unchanged models and bounded CCSP budget."""
import argparse,dataclasses,json,os,random,time
from pathlib import Path
from scripts.validate_vlm_ccsp import RecordedClient,save,source_hashes
from examples.online_manipulation.bt_generation import OpenAICompatibleChatClient
from examples.online_manipulation.tamp_ccsp import Proc3sCCSPSolver
from examples.online_manipulation.tamp_diagnostics import WorkMeter
from examples.online_manipulation.tamp_hierarchy import PredicateGoal,picklift_registry
from examples.online_manipulation.tamp_online import IncrementalTampRunner,JsonlTrace,RecoveryLimits
from examples.online_manipulation.tamp_proc3s import PRoC3SProgramGenerator
from examples.online_manipulation.tamp_semantic import SemanticSubgoalPlanner,ModelSettings
from examples.online_manipulation.tamp_scenesmith_online import SceneSmithSkillExecutor,SceneSmithWorldObserver
from scripts.diagnose_pick_contact import domain_for
from src.online_manipulation import make_env
from src.online_manipulation.experiment import load_experiment

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seed',type=int,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--cache',type=Path,required=True);args=parser.parse_args()
    repo=Path.cwd();out=args.output;out.mkdir(parents=True,exist_ok=False);before=source_hashes(repo);save(out/'source_before.json',before)
    settings=json.loads((repo/'experiments/tamp_hierarchical_config.json').read_text());settings['proc3s_ccsp']={'max_samples':16}
    # Same bounded recovery configuration as both frozen reference runs.
    settings['recovery'].update(semantic_replans=0,skill_replans=1,geometry_retries=1,max_skill_executions=6)
    save(out/'config.json',{'settings':settings,'seed':args.seed,'mode':'ONLINE_REAL_MODEL','video_enabled':False,'cache':str(args.cache),'physics_controller_geometry_threshold_changes':False})
    exp=load_experiment(repo/'experiments/navigation_picklift_tamp.json',repository_root=repo,cache_root=args.cache,scene_root=Path('/root/scenesmith-simple-planner-eval/scene/scene_000'),trust_factories=True,meshcat=False)
    save(out/'resolved_experiment.json',exp.resolved_config);env=make_env(exp.environment_config);obs,info=env.reset(seed=args.seed);task=exp.environment_config.task_factory();target=getattr(task,'task',task).config.target_observation_name;registry=picklift_registry();runtime=out/'runtime';runtime.mkdir()
    executor=SceneSmithSkillExecutor(env=env,experiment=exp,repository_root=repo,output_root=runtime,observation=obs,reset_info=info,max_skill_steps=settings['max_skill_steps'],model_called=True,registry=registry)
    observer=SceneSmithWorldObserver(executor,target,runtime,scene_metadata=Path(exp.resolved_config['provenance']['scene_manifest']).with_name('scene_metadata.json'))
    client=RecordedClient(OpenAICompatibleChatClient(base_url=os.environ['OPENAI_BASE_URL'],api_key=os.environ['OPENAI_API_KEY']),JsonlTrace(out/'raw_model_io.jsonl'));client.case='live_end_to_end';trace=JsonlTrace(runtime/'tamp_trace.jsonl');rng=random.Random(args.seed);meter=WorkMeter()
    def solver_factory(world):return Proc3sCCSPSolver(registry,domain_for(exp,repo,executor.observation),trace=trace,rng=rng,max_samples=16)
    runner=IncrementalTampRunner(semantic=SemanticSubgoalPlanner(client,ModelSettings(**settings['subgoal_model']),trace=trace),registry=registry,solver_factory=solver_factory,executor=executor,observer=observer,trace=trace,limits=RecoveryLimits(**settings['recovery']),program_generator=PRoC3SProgramGenerator(client,ModelSettings(**settings['skill_model']),registry,trace=trace))
    start=time.perf_counter()
    try:
        with meter.instrument(),meter.span('online_total'):
            images=observer.capture_images(obs)
            result=runner.run(task='Pick up the red object and hold it.',task_goals=(PredicateGoal('holding',(target,)),),initial_observation=obs,initial_geometry_state=observer.geometry_state(obs,{'base_height_m':exp.environment_config.robot_adapter.base_config.base_height_m}),predicate_arity={'observed':1,'at_pick_pose':1,'holding':1,'gripper_empty':0},images=images)
        save(out/'live_result.json',{'success':result.success,'reason':result.reason,'metrics':dict(result.metrics),'final_task':dict(executor.observation.task),'wall_time_s':time.perf_counter()-start,'seed':args.seed,'mode':'ONLINE_REAL_MODEL','final_facts':[dataclasses.asdict(f) for f in sorted(result.world.facts)],'video_enabled':False})
        print(json.dumps({'seed':args.seed,'success':result.success,'reason':result.reason}),flush=True)
    finally:
        save(out/'costs.json',meter.as_dict());after=source_hashes(repo);save(out/'source_after.json',after);changed=sorted(k for k in before.keys()|after.keys() if before.get(k)!=after.get(k));save(out/'source_integrity.json',{'changed':changed,'unchanged':not changed})
        if changed:raise RuntimeError('Source changed during online validation')
if __name__=='__main__':main()
