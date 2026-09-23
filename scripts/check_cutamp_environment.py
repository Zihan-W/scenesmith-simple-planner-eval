"""Replay pinned upstream patches before accepting an installed GPU environment."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile


def replay_sources(archive_dir, destination, repo):
    """Verify the two original archives and replay the versioned patch."""
    lock = json.loads((repo / 'experiments/cutamp/upstream-archives.json').read_text())
    roots = {}
    for name, expected in lock.items():
        if not isinstance(expected, dict):
            continue
        archive = archive_dir / name
        if hashlib.sha256(archive.read_bytes()).hexdigest() != expected['sha256']:
            raise ValueError(f'Upstream archive hash mismatch: {name}')
        target = destination / name.removesuffix('.tar.gz')
        target.mkdir()
        with tarfile.open(archive) as stream:
            stream.extractall(target, filter='data')
        children = list(target.iterdir())
        if len(children) != 1 or not children[0].is_dir():
            raise ValueError(f'Expected one upstream archive root: {name}')
        roots[name] = children[0]
    subprocess.run(['patch', '--batch', '--forward', '-p1', '-i',
                    str(repo / 'experiments/cutamp/upstream-local.patch')],
                   cwd=roots['cutamp-main.tar.gz'], check=True, stdout=subprocess.PIPE)
    return roots


def preflight(archive_dir, settings, repo):
    """Fail on archive, replay, installed source, artifact, or dependency drift."""
    from planner.src.tamp.cutamp_backend.integrity import verify_installation
    with tempfile.TemporaryDirectory() as folder:
        roots = replay_sources(Path(archive_dir), Path(folder), repo)
        replay = roots['cutamp-main.tar.gz']
        actual = {str(p.relative_to(replay)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in (replay / 'cutamp').rglob('*.py')}
        checked = verify_installation(settings)
        if actual != checked['optimizer_sources']:
            raise ValueError('Replayed source does not match installed/integrity source')
        if (replay / 'pyproject.toml').read_bytes() != (Path(settings.cutamp_root) / 'pyproject.toml').read_bytes():
            raise ValueError('Installed cuTAMP packaging differs from replay')
        return {**checked, 'patch_replay_verified': True,
                'patch_sha256': hashlib.sha256((repo / 'experiments/cutamp/upstream-local.patch').read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive-dir', type=Path, required=True)
    parser.add_argument('--cutamp-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from planner.src.tamp.cutamp import CuTAMPSettings
    repo = Path(__file__).resolve().parents[1]
    result = preflight(args.archive_dir, CuTAMPSettings.from_file(args.cutamp_config), repo)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print('Patch replay and verify_installation passed')


if __name__ == '__main__':
    main()
