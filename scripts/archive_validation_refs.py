"""Archive old validation refs into a verified, self-contained local Git bundle."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


def archive_refs(repo, output, *, keep=2, prune=False):
    """Keep current plus recent refs; delete old refs only after a fresh-repo restore."""
    if keep < 1:
        raise ValueError('Retain at least one dated validation ref')
    repo, output = Path(repo).resolve(), Path(output).resolve()
    def git(*args, **kwargs):
        return subprocess.run(['git', '-C', str(repo), *args], check=True,
                              capture_output=True, text=True, **kwargs).stdout.strip()
    rows = git('for-each-ref', '--sort=-creatordate', '--format=%(refname) %(objectname)',
               'refs/validation/').splitlines()
    refs = dict(row.split() for row in rows)
    dated = [ref for ref in refs if ref != 'refs/validation/current']
    selected = {ref: refs[ref] for ref in dated[keep:]}
    output.mkdir(parents=True, exist_ok=False)
    if not selected:
        raise ValueError('No refs exceed retention limit')
    bundle = output / 'validation.bundle'
    git('bundle', 'create', str(bundle), *selected)
    git('bundle', 'verify', str(bundle))
    with tempfile.TemporaryDirectory() as directory:
        subprocess.run(['git', 'init', '--bare', directory], check=True, capture_output=True)
        subprocess.run(['git', '-C', directory, 'fetch', str(bundle),
                        *(f'{ref}:{ref}' for ref in selected)], check=True, capture_output=True)
        for ref, expected in selected.items():
            actual = subprocess.run(['git', '-C', directory, 'rev-parse', ref],
                                    check=True, capture_output=True, text=True).stdout.strip()
            if actual != expected:
                raise RuntimeError('Restored validation ref differs from original')
    manifest = {'schema': 'validation.ref_archive.v1', 'bundle': bundle.name,
                'bundle_sha256': hashlib.sha256(bundle.read_bytes()).hexdigest(),
                'refs': selected, 'retained': {ref:oid for ref,oid in refs.items() if ref not in selected},
                'fresh_repository_restore_verified': True, 'pruned': False}
    path = output / 'manifest.json'
    path.write_text(json.dumps(manifest, indent=2) + '\n')
    if prune:
        # Compare-and-delete as one transaction; concurrent ref changes abort it.
        git('update-ref', '--stdin', input='start\n' + ''.join(
            f'delete {ref} {oid}\n' for ref, oid in selected.items()) + 'prepare\ncommit\n')
        manifest['pruned'] = True
        path.write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--keep', type=int, default=2)
    parser.add_argument('--prune', action='store_true', help='Delete archived refs after verified restore')
    args = parser.parse_args()
    print(json.dumps(archive_refs(args.repository, args.output, keep=args.keep, prune=args.prune), indent=2))


if __name__ == '__main__':
    main()
