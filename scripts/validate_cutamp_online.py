"""Serial multi-seed recorded or strictly replayed simulation acceptance on one immutable source snapshot."""
import argparse
from collections import Counter
import dataclasses
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from scripts.check_cutamp_environment import preflight
from planner.src.tamp.cutamp import CuTAMPSettings
from planner.src.tamp.provenance import record_validation_commit, source_manifest


def save(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive-dir', type=Path, required=True)
    parser.add_argument('--scene-root', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--experiment', type=Path, help='Experiment supported by the selected TAMP domain')
    parser.add_argument('--task', default='Pick up the red object and hold it.')
    parser.add_argument('--shift-world-x-m', type=float, default=.10)
    parser.add_argument('--cutamp-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', required=True)
    parser.add_argument('--validation-ref', required=True)
    replay = parser.add_mutually_exclusive_group()
    replay.add_argument('--replay-from', type=Path, help='Prior acceptance directory with a transcript per seed')
    replay.add_argument('--model-replay', type=Path, help='One common fixed model transcript for every seed')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--max-wall-time-s', type=float, default=1200.)
    args = parser.parse_args()
    if args.repeats < 1 or len(set(args.seeds)) != len(args.seeds):
        parser.error('Require positive repeats and unique seeds')
    if args.replay_from:
        for seed in args.seeds:
            if not (args.replay_from / f'seed_{seed}' / 'model_transcript.jsonl').is_file():
                parser.error(f'Missing replay transcript for seed {seed}')
    if args.model_replay and not args.model_replay.is_file():
        parser.error('Missing common model transcript')
    replaying = bool(args.replay_from or args.model_replay)
    repo = Path(__file__).resolve().parents[1]
    experiment = (args.experiment or repo / 'experiments/navigation_picklift_tamp.json').resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
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
        cutamp_config = output / f'cutamp_seed_{seed}.json'
        save(cutamp_config, dataclasses.asdict(dataclasses.replace(settings, seed=seed)))
        command = [sys.executable, '-m', 'planner.src.tamp.cli', '--planner', 'tamp',
                   '--tamp-mode', 'hierarchical', '--skill-planner', 'proc3s',
                   '--geometry-backend', 'cutamp', '--repository-root', str(repo),
                   '--scene-root', str(args.scene_root.resolve()),
                   '--experiment', str(experiment),
                   '--config', str(output / 'planner_config.json'), '--cutamp-config', str(cutamp_config),
                   '--task', args.task, '--seed', str(seed),
                   '--shift-world-x-m', str(args.shift_world_x_m), '--record-html', '--max-wall-time-s', str(args.max_wall_time_s),
                   '--output-root', str(run)]
        if args.model_replay:
            command += ['--model-replay', str(args.model_replay.resolve())]
        elif args.replay_from:
            command += ['--model-replay', str((args.replay_from / f'seed_{seed}' / 'model_transcript.jsonl').resolve())]
        save(output / f'command_{trial_name}.json', command)
        with (output / f'{trial_name}.log').open('w') as log:
            code = subprocess.run(command, cwd=repo, stdout=log, stderr=subprocess.STDOUT).returncode
        unchanged = source_manifest(repo)['source_tree_sha256'] == evidence['source_tree_sha256']
        if not unchanged:
            raise RuntimeError('Source changed during trial; refusing mixed-snapshot statistics')
        result = json.loads((run / 'result.json').read_text())
        selections = []
        for path in sorted(run.glob('cutamp/solve_*/candidate_selection.json')):
            selections.append({'solve': path.parent.name, **json.loads(path.read_text())})
        model_path = run / 'model_evidence.json'
        model = json.loads(model_path.read_text()) if model_path.exists() else None
        trials.append({'seed': seed, 'repetition': repetition, 'model': model, 'worker_seed_first_solve': seed, 'returncode': code,
                       'success': bool(result['success']), 'reason': result['reason'],
                       'metrics': result.get('metrics', {}), 'selections': selections,
                       'source_unchanged': unchanged, 'result': str(run / 'result.json')})
        n = len(trials); k = sum(item['success'] for item in trials)
        z = 1.959963984540054
        center = (k/n + z*z/(2*n))/(1+z*z/n)
        margin = z*((k/n)*(1-k/n)/n+z*z/(4*n*n))**.5/(1+z*z/n)
        save(output / 'summary.json', {'source_tree_sha256': evidence['source_tree_sha256'],
             'validation_commit': evidence['validation_commit'], 'planned_seeds': args.seeds,
             'completed': n, 'successes': k, 'success_rate': k/n,
             'wilson_95_interval': [center-margin, center+margin] if args.repeats == 1 and not replaying else None,
             'reasons': dict(Counter(t['reason'] for t in trials)),
             'adaptive_postcheck': settings.adaptive_postcheck,
             'postcheck_quality_window': settings.postcheck_quality_window,
             'model_mode': 'strict_replay' if replaying else 'live_record',
             'repeats': args.repeats,
             'experiment': str(experiment), 'task': args.task, 'shift_world_x_m': args.shift_world_x_m,
             'replay_divergences': sum(t['reason'] == 'model_replay_divergence' for t in trials),
             'interval_assumption': 'descriptive_only; repeated seeds/transcripts are correlated, not independent Bernoulli samples',
             'scope': ('fixed_recorded_model_conditional_execution; report per seed and transcript; not live-model reliability'
                       if replaying else 'live_model_and_execution_joint_outcome; not physics-only reliability'),
             'trials': trials})
        print(json.dumps({'seed': seed, 'success': result['success'], 'reason': result['reason']}), flush=True)
    print(f'Acceptance complete: {output / "summary.json"}', flush=True)


if __name__ == '__main__':
    main()
