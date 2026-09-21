"""Export one retained particle for inspection, never as a feasible selection."""

import argparse
import hashlib
import json
from pathlib import Path

import torch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe-root", type=Path, required=True)
    parser.add_argument("--particle-index", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    root = args.probe_root.resolve()
    result = json.loads((root / "result.json").read_text())
    if not 0 <= args.particle_index < result["num_particles"]:
        raise ValueError("Particle index is outside the retained batch")
    path = root / "particles.pt"
    particles = torch.load(path, map_location="cpu", weights_only=True)
    q = particles["q_pick"][args.particle_index].tolist()
    mobile = result["base_pose_optimized"]
    base = result["fixed_base_world_pose"]
    report = {"scope": "unselected_particle_diagnostic_not_execution_authorization",
              "probe_root": str(root), "particle_index": args.particle_index,
              "optimizer_selected_particle": result["selected_particle"],
              "source_particle_file_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "arm_joint_names": result["selected_arm_joint_names"],
              "arm_joint_positions": q[2:] if mobile else q,
              "base_world_pose": [*q[:2], *base[2:]] if mobile else base,
              "grasp_lateral_offset_m": result["initialization"]["grasp_lateral_offsets_m"][args.particle_index]}
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
