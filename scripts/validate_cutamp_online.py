"""Serial multi-seed live simulation acceptance on one immutable source snapshot."""
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
    parser.add_argument('--cutamp-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', required=True)
    parser.add_argument('--validation-ref', required=True)
    parser.add_argument('--max-wall-time-s', type=float, default=1200.)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    settings = CuTAMPSettings.from_file(args.cutamp_config)
    save(output / 'preflight.json', preflight(args.archive_dir, settings, repo))
    evidence = record_validation_commit(repo, output / 'provenance', ref=args.validation_ref)
    config = json.loads(args.config.read_text())
    save(output / 'planner_config.json', config)
    trials = []
    for seed in args.seeds:
        if source_manifest(repo)['source_tree_sha256'] != evidence['source_tree_sha256']:
            raise RuntimeError('Source changed before trial; refusing mixed-snapshot statistics')
        run = output / f'seed_{seed}'
        cutamp_config = output / f'cutamp_seed_{seed}.json'
        save(cutamp_config, dataclasses.asdict(dataclasses.replace(settings, seed=seed)))
        command = [sys.executable, '-m', 'planner.src.tamp.cli', '--planner', 'tamp',
                   '--tamp-mode', 'hierarchical', '--skill-planner', 'proc3s',
                   '--geometry-backend', 'cutamp', '--repository-root', str(repo),
                   '--scene-root', str(args.scene_root.resolve()),
                   '--experiment', str(repo / 'experiments/navigation_picklift_tamp.json'),
                   '--config', str(output / 'planner_config.json'), '--cutamp-config', str(cutamp_config),
                   '--task', 'Pick up the red object and hold it.', '--seed', str(seed),
                   '--shift-world-x-m', '0.10', '--record-html', '--max-wall-time-s', str(args.max_wall_time_s),
                   '--output-root', str(run)]
        save(output / f'command_{seed}.json', command)
        with (output / f'seed_{seed}.log').open('w') as log:
            code = subprocess.run(command, cwd=repo, stdout=log, stderr=subprocess.STDOUT).returncode
        unchanged = source_manifest(repo)['source_tree_sha256'] == evidence['source_tree_sha256']
        if not unchanged:
            raise RuntimeError('Source changed during trial; refusing mixed-snapshot statistics')
        result = json.loads((run / 'result.json').read_text())
        selections = []
        for path in sorted(run.glob('cutamp/solve_*/candidate_selection.json')):
            selections.append({'solve': path.parent.name, **json.loads(path.read_text())})
        trials.append({'seed': seed, 'worker_seed_first_solve': seed, 'returncode': code,
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
             'wilson_95_interval': [center-margin, center+margin],
             'reasons': dict(Counter(t['reason'] for t in trials)),
             'adaptive_postcheck': settings.adaptive_postcheck,
             'scope': 'live_VLM_GPU_exact_postcheck_simulation_small_sample_not_reliability_certification',
             'trials': trials})
        print(json.dumps({'seed': seed, 'success': result['success'], 'reason': result['reason']}), flush=True)
    print(f'Acceptance complete: {output / "summary.json"}', flush=True)


if __name__ == '__main__':
    main()
