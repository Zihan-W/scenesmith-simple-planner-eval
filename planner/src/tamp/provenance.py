"""Content-addressed source evidence for experiments, including dirty trees."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tarfile

SOURCE_DIRS = ("docs", "planner", "simulation", "scripts", "tests", "tools", "experiments")


def source_manifest(repo):
    repo = Path(repo)
    files = {}
    for folder in SOURCE_DIRS:
        for path in sorted((repo / folder).rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts and path.suffix in {
                    ".py", ".json", ".yaml", ".yml", ".txt", ".toml", ".html", ".md", ".patch"}:
                files[str(path.relative_to(repo))] = hashlib.sha256(path.read_bytes()).hexdigest()
    for name in ("pyproject.toml", ".gitignore", "README.md"):
        path = repo / name
        if path.is_file():
            files[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    digest = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()
    return {"source_tree_sha256": digest, "files": files}


def record_source_evidence(repo, output):
    repo, output = Path(repo), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    manifest = source_manifest(repo)
    for key, command in (("git_head", ["rev-parse", "HEAD"]),
                         ("git_status", ["status", "--porcelain=v1"])):
        manifest[key] = subprocess.run(["git", "-C", str(repo), *command],
            check=True, capture_output=True, text=True,
            env={**os.environ, "GIT_OPTIONAL_LOCKS": "0"}).stdout.strip()
    archive = output / "validated_sources.tar.gz"
    with tarfile.open(archive, "w:gz") as stream:
        for name in manifest["files"]:
            stream.add(repo / name, arcname=name, recursive=False)
    manifest["archive_sha256"] = hashlib.sha256(archive.read_bytes()).hexdigest()
    if source_manifest(repo)["source_tree_sha256"] != manifest["source_tree_sha256"]:
        raise RuntimeError("Source changed while recording validation evidence")
    (output / "source_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def record_validation_commit(repo, output, *, ref):
    """Commit the evidence archive with a temporary index, without touching HEAD.

    The validation ref stores only manifest-listed source/config/test files.
    Large scene assets and execution artifacts remain separately referenced.
    """
    import os
    import tempfile
    repo, output = Path(repo).resolve(), Path(output).resolve()
    evidence = record_source_evidence(repo, output)

    def git(*args, **kwargs):
        return subprocess.run(["git", "-C", str(repo), *args], check=True,
                              capture_output=True, **kwargs).stdout

    git("check-ref-format", ref)
    index_path = Path(git("rev-parse", "--path-format=absolute", "--git-path", "index").decode().strip())
    original_index = index_path.read_bytes() if index_path.exists() else None
    head = git("rev-parse", "HEAD").decode().strip()
    with tempfile.TemporaryDirectory() as directory:
        index = str(Path(directory) / "index")
        env = {**os.environ, "GIT_INDEX_FILE": index}
        git("read-tree", "--empty", env=env)
        # Hash and stage the archived bytes, not a potentially changing worktree.
        with tarfile.open(output / "validated_sources.tar.gz") as archive:
            entries = []
            for name in evidence["files"]:
                member = archive.getmember(name)
                blob = git("hash-object", "-w", "--stdin", input=archive.extractfile(member).read()).decode().strip()
                mode = "100755" if member.mode & 0o111 else "100644"
                entries.append(f"{mode} {blob}\t{name}\0".encode())
            git("update-index", "-z", "--index-info", input=b"".join(entries), env=env)
        tree = git("write-tree", env=env).decode().strip()
        commit = git("commit-tree", tree, "-p", head,
                     input=("Local validation source snapshot\n\n" + evidence["source_tree_sha256"] + "\n").encode()).decode().strip()
        # Zero expected old value forbids overwriting an existing validation ref.
        git("update-ref", ref, commit, "0" * 40)
    if (index_path.read_bytes() if index_path.exists() else None) != original_index or git("rev-parse", "HEAD").decode().strip() != head:
        raise RuntimeError("Original index or HEAD changed during evidence creation")
    if source_manifest(repo)["source_tree_sha256"] != evidence["source_tree_sha256"]:
        raise RuntimeError("Source changed while creating validation commit")
    evidence.update(validation_commit=commit, validation_ref=ref,
                    original_index_unchanged=True, original_head_unchanged=True,
                    commit_scope="manifest_sources_only_not_external_assets")
    (output / "source_manifest.json").write_text(json.dumps(evidence, indent=2) + "\n")
    return evidence
