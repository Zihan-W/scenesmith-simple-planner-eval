"""Reject runtime artifact or optimizer drift before invoking CUDA."""
import hashlib
import json
import subprocess
from pathlib import Path

ARTIFACT_FIELDS = ("kinematics_template", "robot_template", "multipliers",
                   "tolerances", "grasp_calibration")


def gpu_environment(python):
    """Read interpreter/package versions without importing GPU kernels."""
    names = ("torch", "triton", "nvidia-curobo", "cutamp", "numpy", "scipy",
             "roma", "trimesh", "warp-lang", "pyyaml", "networkx", "omegaconf",
             "jaxtyping", "einops", "tqdm", "yourdfpy", "rtree", "embreex")
    probe = ("import importlib.metadata as m,json,platform; names=" + repr(names) + "; "
             "print(json.dumps({'python':platform.python_version(),'packages':"
             "{d.metadata['Name'].lower():d.version for d in m.distributions() "
             "if d.metadata['Name'].lower() in names or d.metadata['Name'].lower().startswith('nvidia-')}}))")
    return json.loads(subprocess.run([python, "-c", probe], check=True,
                                      capture_output=True, text=True).stdout)


def verify_installation(settings):
    if settings.integrity_manifest is None:
        raise ValueError("cuTAMP execution requires an integrity_manifest; use the versioned configuration")
    manifest = json.loads(Path(settings.integrity_manifest).read_text())
    if manifest.get("schema") != "scenesmith.cutamp.integrity.v1":
        raise ValueError("Unsupported cuTAMP integrity manifest")
    artifacts = manifest["artifacts"]
    if set(artifacts) != set(ARTIFACT_FIELDS):
        raise ValueError("Incomplete cuTAMP artifact hashes")
    checked = {}
    for name in ARTIFACT_FIELDS:
        digest = hashlib.sha256(Path(getattr(settings, name)).read_bytes()).hexdigest()
        if digest != artifacts[name]:
            raise ValueError(f"cuTAMP artifact hash mismatch: {name}")
        checked[name] = digest
    root = Path(settings.cutamp_root)
    expected = manifest["optimizer_sources"]
    actual = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (root / "cutamp").rglob("*.py")}
    if actual != expected:
        changed = sorted(k for k in set(actual) | set(expected) if actual.get(k) != expected.get(k))
        raise ValueError(f"cuTAMP optimizer source drift: {changed}")
    environment = gpu_environment(settings.gpu_python)
    if environment != manifest["gpu_environment"]:
        raise ValueError("cuTAMP GPU Python/package versions differ from the integrity lock")
    return {"gpu_environment": environment, "artifacts": checked, "optimizer_sources": actual,
            "manifest_sha256": hashlib.sha256(Path(settings.integrity_manifest).read_bytes()).hexdigest()}
