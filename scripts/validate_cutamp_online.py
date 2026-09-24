"""Serial multi-seed recorded or strictly replayed simulation acceptance on one immutable source snapshot."""
import argparse
from collections import Counter
import dataclasses
import json
from pathlib import Path
import subprocess
import sys

from planner.src.tamp.provenance import record_validation_commit, source_manifest


def save(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def parse_args(argv=None):
    """Parse shared acceptance options and reject incompatible backend settings."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry-backend', choices=('proc3s', 'cutamp'), default='proc3s',
                        help='PRoC3S CCSP sampling or cuTAMP GPU solving; both use PRoC3S programs')
    parser.add_argument('--archive-dir', type=Path, help='Required for cuTAMP installation verification')
    parser.add_argument('--scene-root', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--experiment', type=Path, help='Experiment supported by the selected TAMP domain')
    parser.add_argument('--task', default='Pick up the red object and hold it.')
    parser.add_argument('--shift-world-x-m', type=float, default=.10)
    parser.add_argument('--cutamp-config', type=Path, help='Required only for the cuTAMP backend')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', required=True)
    parser.add_argument('--validation-ref', required=True)
    replay = parser.add_mutually_exclusive_group()
    replay.add_argument('--replay-from', type=Path, help='Prior acceptance directory with a transcript per seed')
    replay.add_argument('--model-replay', type=Path, help='One common fixed model transcript for every seed')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--max-wall-time-s', type=float, default=1200.)
    args = parser.parse_args(argv)
    if args.geometry_backend == 'cutamp':
        if args.archive_dir is None or args.cutamp_config is None:
            parser.error('cutamp requires --archive-dir and --cutamp-config')
    elif args.archive_dir is not None or args.cutamp_config is not None:
        parser.error('--archive-dir and --cutamp-config require --geometry-backend cutamp')
    if args.repeats < 1 or len(set(args.seeds)) != len(args.seeds):
        parser.error('Require positive repeats and unique seeds')
    if args.replay_from:
        for seed in args.seeds:
            if not (args.replay_from / f'seed_{seed}' / 'model_transcript.jsonl').is_file():
                parser.error(f'Missing replay transcript for seed {seed}')
    if args.model_replay and not args.model_replay.is_file():
        parser.error('Missing common model transcript')
    return args


def trial_command(args, repo, experiment, output, run, seed, cutamp_config=None):
    """Build the actual supervised CLI invocation for either geometry backend."""
    command = [sys.executable, '-m', 'planner.src.tamp', '--planner', 'tamp',
               '--tamp-mode', 'hierarchical', '--skill-planner', 'proc3s',
               '--geometry-backend', args.geometry_backend, '--repository-root', str(repo),
               '--scene-root', str(args.scene_root.resolve()),
               '--experiment', str(experiment),
               '--config', str(output / 'planner_config.json'),
               '--task', args.task, '--seed', str(seed),
               '--shift-world-x-m', str(args.shift_world_x_m), '--record-html',
               '--max-wall-time-s', str(args.max_wall_time_s), '--output-root', str(run)]
    if args.geometry_backend == 'cutamp':
        if cutamp_config is None:
            raise ValueError('cuTAMP trial requires its resolved seed configuration')
        command += ['--cutamp-config', str(cutamp_config)]
    elif cutamp_config is not None:
        raise ValueError('PRoC3S trial cannot use a cuTAMP configuration')
    if args.model_replay:
        command += ['--model-replay', str(args.model_replay.resolve())]
    elif args.replay_from:
        command += ['--model-replay', str((args.replay_from / f'seed_{seed}' / 'model_transcript.jsonl').resolve())]
    return command


def main():
    args = parse_args()
    replaying = bool(args.replay_from or args.model_replay)
    repo = Path(__file__).resolve().parents[1]
    experiment = (args.experiment or repo / 'experiments/navigation_picklift_tamp.json').resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    settings = None
    if args.geometry_backend == 'cutamp':
        from scripts.check_cutamp_environment import preflight
        from planner.src.tamp.cutamp import CuTAMPSettings

        settings = CuTAMPSettings.from_file(args.cutamp_config)
        save(output / 'preflight.json', preflight(args.archive_dir, settings, repo))
    evidence = record_validation_commit(repo, output / 'provenance', ref=args.validation_ref)
    config = json.loads(args.config.read_text())
    save(output / 'planner_config.json', config)
    trials = []
    for seed, repetition in ((seed, repetition) for seed in args.seeds for repetition in range(args.repeats)):
        if source_manifest(repo)['source_tree_sha256'] != evidence['source_tree_sha256']:
            raise RuntimeError('Source changed before trial; refusing mixed-snapshot statistics')
        trial_name = f'seed_{seed}' + (f'_repeat_{repetition + 1:03d}' if repetition else '')
        run = output / trial_name
        cutamp_config = None
        if settings is not None:
            cutamp_config = output / f'cutamp_seed_{seed}.json'
            save(cutamp_config, dataclasses.asdict(dataclasses.replace(settings, seed=seed)))
        command = trial_command(args, repo, experiment, output, run, seed, cutamp_config)
        save(output / f'command_{trial_name}.json', command)
        with (output / f'{trial_name}.log').open('w') as log:
            code = subprocess.run(command, cwd=repo, stdout=log, stderr=subprocess.STDOUT).returncode
        unchanged = source_manifest(repo)['source_tree_sha256'] == evidence['source_tree_sha256']
        if not unchanged:
            raise RuntimeError('Source changed during trial; refusing mixed-snapshot statistics')
        result = json.loads((run / 'result.json').read_text())
        # Supervision may terminate the worker before it records planner metadata.
        if (('geometry_backend' in result and result['geometry_backend'] != args.geometry_backend)
                or ('skill_planner' in result and result['skill_planner'] != 'proc3s')):
            raise RuntimeError('Trial used a different planner/backend than requested')
        selections = []
        for path in sorted(run.glob('cutamp/solve_*/candidate_selection.json')):
            selections.append({'solve': path.parent.name, **json.loads(path.read_text())})
        model_path = run / 'model_evidence.json'
        model = json.loads(model_path.read_text()) if model_path.exists() else None
        trials.append({'seed': seed, 'repetition': repetition, 'model': model,
                       'geometry_backend': args.geometry_backend, 'skill_planner': 'proc3s',
                       'worker_seed_first_solve': seed if settings is not None else None, 'returncode': code,
                       'success': bool(result['success']), 'reason': result['reason'],
                       'metrics': result.get('metrics', {}), 'selections': selections,
                       'cutamp_selections': result['cutamp_selections'],
                       'cutamp_settings': result['cutamp_settings'],
                       'grasp_compensation': result['grasp_compensation'],
                       'selection_config_recorded': result['selection_config_recorded'],
                       'source_unchanged': unchanged, 'result': str(run / 'result.json')})
        n = len(trials); k = sum(item['success'] for item in trials)
        z = 1.959963984540054
        center = (k/n + z*z/(2*n))/(1+z*z/n)
        margin = z*((k/n)*(1-k/n)/n+z*z/(4*n*n))**.5/(1+z*z/n)
        summary = {'source_tree_sha256': evidence['source_tree_sha256'],
             'geometry_backend': args.geometry_backend, 'skill_planner': 'proc3s',
             'validation_commit': evidence['validation_commit'], 'planned_seeds': args.seeds,
             'completed': n, 'successes': k, 'success_rate': k/n,
             'wilson_95_interval': [center-margin, center+margin] if args.repeats == 1 and not replaying else None,
             'reasons': dict(Counter(t['reason'] for t in trials)),
             'adaptive_postcheck': settings.adaptive_postcheck if settings is not None else None,
             'postcheck_quality_window': settings.postcheck_quality_window if settings is not None else None,
             'model_mode': 'strict_replay' if replaying else 'live_record',
             'repeats': args.repeats,
             'experiment': str(experiment), 'task': args.task, 'shift_world_x_m': args.shift_world_x_m,
             'replay_divergences': sum(t['reason'] == 'model_replay_divergence' for t in trials),
             'interval_assumption': 'descriptive_only; repeated seeds/transcripts are correlated, not independent Bernoulli samples',
             'scope': ('fixed_recorded_model_conditional_execution; report per seed and transcript; not live-model reliability'
                       if replaying else 'live_model_and_execution_joint_outcome; not physics-only reliability'),
             'trials': trials}
        save(output / 'summary.json', summary)
        # Keep the machine-readable acceptance index on the same per-trial contract.
        save(output / 'index.json', {'schema': 'cutamp.acceptance_index.v1', **summary})
        print(json.dumps({'seed': seed, 'success': result['success'], 'reason': result['reason']}), flush=True)
    print(f'Acceptance complete: {output / "summary.json"}', flush=True)


if __name__ == '__main__':
    main()
