"""Build a fresh GPU venv from verified upstream archives and version locks."""
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess
from scripts.check_cutamp_environment import replay_sources


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive-dir', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    lock = repo / 'experiments/cutamp/gpu-requirements.lock.txt'
    integrity = json.loads((repo / 'experiments/cutamp/integrity.json').read_text())
    if platform.python_version() != integrity['gpu_environment']['python']:
        raise ValueError('Run with the exact Python version in integrity.json')
    destination = args.destination.resolve()
    destination.mkdir(parents=True, exist_ok=False)
    roots = replay_sources(args.archive_dir.resolve(), destination, repo)
    import venv
    venv.EnvBuilder(with_pip=True).create(destination / 'venv')
    python = destination / 'venv/bin/python'
    requirements = destination / 'binary-requirements.txt'
    requirements.write_text('\n'.join(line for line in lock.read_text().splitlines()
                                      if not line.startswith(('cutamp==', 'nvidia-curobo=='))) + '\n')
    def pip(*arguments, env=None):
        subprocess.run([str(python), '-m', 'pip', *arguments], check=True, env=env)
    pip('install', '--extra-index-url', 'https://download.pytorch.org/whl/cu121',
        '-r', str(requirements), 'setuptools', 'setuptools_scm', 'wheel', 'ninja')
    for name in ('curobo-v0.7.8.tar.gz', 'cutamp-main.tar.gz'):
        build_env = dict(os.environ)
        if name == 'curobo-v0.7.8.tar.gz':
            # Verified archives have no .git metadata for setuptools_scm.
            build_env['SETUPTOOLS_SCM_PRETEND_VERSION_FOR_NVIDIA_CUROBO'] = integrity['gpu_environment']['packages']['nvidia-curobo']
        pip('install', '--no-build-isolation', '-c', str(lock), '-e', str(roots[name]), env=build_env)
    pip('install', '--no-deps', '-e', str(repo))
    # Use the CPU environment's preflight: it imports Drake via CuTAMPSettings.
    environment = dict(os.environ, CUTAMP_PYTHON=str(python),
                       CUTAMP_ROOT=str(roots['cutamp-main.tar.gz']))
    env_file = destination / 'environment.json'
    env_file.write_text(json.dumps({key: environment[key] for key in ('CUTAMP_PYTHON', 'CUTAMP_ROOT')}, indent=2)+'\n')
    print(f'Built GPU environment; machine paths: {env_file}')
    print('Run check_cutamp_environment with the CPU project Python before acceptance.')


if __name__ == '__main__':
    main()
