"""Conservatively cover selected original Drake box links with cuRobo spheres.

This changes only the requested links in a retained robot configuration. It
does not certify other links, remove self-collision pairs or change thresholds.
Each source box is tiled by closed rectangular cells, with one circumsphere per
cell: every point in the original box is therefore inside at least one sphere.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.robot import RobotConfig


def box_cover(record, max_cell_edge_m):
    """Return a full-volume sphere cover in the original robot body frame."""
    if record["shape"]["kind"] != "box":
        raise ValueError(f"Only exact box geometry is supported: {record['name']}")
    size = np.asarray(record["shape"]["size_m"], dtype=float)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError("Box dimensions must be positive finite metric lengths")
    divisions = np.ceil(size / max_cell_edge_m).astype(int)
    cell = size / divisions
    radius = float(np.linalg.norm(cell) / 2 + 1e-7)
    centers = np.stack(np.meshgrid(*[
        -length / 2 + (np.arange(count) + 0.5) * step
        for length, count, step in zip(size, divisions, cell, strict=True)
    ], indexing="ij"), axis=-1).reshape(-1, 3)
    transform = np.asarray(record["body_from_geometry"], dtype=float)
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0):
        raise ValueError("Geometry transform is not rigid")
    centers = centers @ rotation.T + transform[:3, 3]
    return [{"center": center.tolist(), "radius": radius} for center in centers], {
        "geometry": record["name"], "divisions": divisions.tolist(),
        "cell_size_m": cell.tolist(), "sphere_radius_m": radius,
        "full_box_volume_covered_by_construction": True,
        "sphere_count": len(centers),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--robot-config", type=Path, required=True)
    parser.add_argument("--link", action="append", required=True)
    parser.add_argument("--max-cell-edge-m", type=float, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    if not np.isfinite(args.max_cell_edge_m) or args.max_cell_edge_m <= 0:
        parser.error("max-cell-edge-m must be positive and finite")
    output = args.output_root.resolve()
    output.mkdir(parents=True, exist_ok=False)
    geometry = json.loads(args.geometry.read_text())
    config = json.loads(args.robot_config.read_text())
    spheres = config["kinematics"]["collision_spheres"]
    covers = []
    for link in args.link:
        if link not in spheres:
            raise ValueError(f"Requested link is absent from the supplied model: {link}")
        records = [r for r in geometry["geometries"] if r["robot"] and r["body"] == link]
        if not records:
            raise ValueError(f"No original proximity geometries for {link}")
        replacement = []
        for record in records:
            values, report = box_cover(record, args.max_cell_edge_m)
            replacement.extend(values)
            covers.append(report)
        spheres[link] = replacement
    total = sum(len(values) for values in spheres.values())
    if total >= 1024:
        raise ValueError(f"{total} spheres exceed the actual cuRobo kernel's <1024 limit")
    # Exercise the actual CUDA robot loader before retaining a usable config.
    robot = CudaRobotModel(RobotConfig.from_dict(copy.deepcopy(config)).kinematics)
    q = torch.zeros((1, len(robot.joint_names)), device="cuda")
    actual = robot.get_state(q).get_link_spheres().detach().cpu().numpy()[0]
    expected = []
    for link, values in spheres.items():
        transform = np.asarray(geometry["base_from_robot_body"][link])
        for value in values:
            center = transform[:3, :3] @ value["center"] + transform[:3, 3]
            expected.append([*center, value["radius"]])
    # Exported zero/home arm state must match this q; fail rather than silently
    # use the transform comparison for another initial arm configuration.
    error = float(np.max(np.abs(actual - np.asarray(expected))))
    if error >= 1e-5:
        raise ValueError(f"Original export and zero-arm sphere transforms differ: {error}")
    report = {"scope": "selected_box_links_conservative_cover_not_whole_robot_certification",
              "modified_links": args.link, "covers": covers, "num_spheres": total,
              "max_home_sphere_transform_error_m": error,
              "collision_model_validated": False,
              "source_geometry_sha256": hashlib.sha256(args.geometry.read_bytes()).hexdigest(),
              "source_robot_config_sha256": hashlib.sha256(args.robot_config.read_bytes()).hexdigest()}
    (output / "robot_config.json").write_text(json.dumps(config, indent=2) + "\n")
    (output / "box_cover_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
