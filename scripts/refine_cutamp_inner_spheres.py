"""Enlarge selected planning spheres only inside their convex proximity mesh.

The existing fitter leaves interior voxel spheres smaller than their available
clearance. Existing centers and exterior spheres are retained. Optional added spheres are
contained in the same convex body; filters and safety thresholds are unchanged. It improves coverage; it is NOT a complete mesh cover.
Exact Drake checks remain required.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull


def expand_inner_spheres(spheres, vertices):
    """Return a monotone sphere refinement and its per-sphere containment audit."""
    points = np.asarray(vertices, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("Expected finite 3D proximity vertices")
    planes = ConvexHull(points).equations
    normals = np.linalg.norm(planes[:, :3], axis=1)
    result, audit = copy.deepcopy(spheres), []
    for index, sphere in enumerate(result):
        center = np.asarray(sphere['center'], dtype=float)
        old = float(sphere['radius'])
        if center.shape != (3,) or not np.isfinite(center).all() or not np.isfinite(old) or old <= 0:
            raise ValueError("Expected finite centers and positive radii")
        clearance = float(np.min(-(planes[:, :3] @ center + planes[:, 3]) / normals))
        # Numerical inward construction padding; no collision tolerance changes.
        sphere['radius'] = max(old, clearance - 1e-8)
        audit.append({'index': index, 'old_radius_m': old,
                      'new_radius_m': sphere['radius'],
                      'convex_boundary_clearance_m': clearance,
                      'expanded': sphere['radius'] > old})
    return result, audit



def augment_inner_spheres(spheres, vertices, surface_points, surface_normals, count):
    """Add mesh-contained tangent spheres to reduce deterministic fitting gaps.

    Selection uses only geometry, never task candidates or joint configurations.
    The probe depths are approximation-fitting samples, not safety tolerances.
    """
    if not isinstance(count, int) or count < 0:
        raise ValueError("Additional sphere count must be a nonnegative integer")
    points, normals = np.asarray(surface_points), np.asarray(surface_normals)
    if (points.ndim != 2 or points.shape[1] != 3 or normals.shape != points.shape
            or not np.isfinite(points).all() or not np.isfinite(normals).all()
            or not np.allclose(np.linalg.norm(normals, axis=1), 1, atol=1e-8)):
        raise ValueError("Expected finite surface points and unit outward normals")
    planes = ConvexHull(vertices).equations
    numer = -(points @ planes[:, :3].T + planes[:, 3])
    denom = 1 - normals @ planes[:, :3].T
    ratios = np.where(denom > 1e-10, numer / np.maximum(denom, 1e-10), np.inf)
    radii = np.maximum(ratios.min(axis=1) - 1e-8, 0)
    centers = points - radii[:, None] * normals
    valid = radii > 1e-5
    centers, radii = centers[valid], radii[valid]
    if len(radii) < count:
        raise ValueError("Insufficient valid mesh-contained candidate spheres")
    probes = np.concatenate([points[::3] - normals[::3] * depth
                             for depth in (.001, .003, .006, .01)])
    probes = probes[np.max(probes @ planes[:, :3].T + planes[:, 3], axis=1) <= 1e-9]
    if not len(probes):
        raise ValueError("No interior fitting probes")
    result = copy.deepcopy(spheres)
    old_centers = np.array([s['center'] for s in spheres])
    old_radii = np.array([s['radius'] for s in spheres])
    deficit = np.maximum((np.linalg.norm(probes[:, None] - old_centers, axis=2)
                          - old_radii).min(axis=1), 0)
    distances = np.maximum(np.linalg.norm(probes[:, None] - centers, axis=2)
                           - radii, 0).astype(np.float32)
    original = deficit.copy()
    chosen = []
    for _ in range(count):
        gain = (deficit[:, None] - np.minimum(deficit[:, None], distances)).sum(axis=0)
        gain[chosen] = -1
        index = int(np.argmax(gain))
        chosen.append(index)
        deficit = np.minimum(deficit, distances[:, index])
        result.append({'center': centers[index].tolist(), 'radius': float(radii[index])})
    outside = (float(np.max(centers[chosen] @ planes[:, :3].T + planes[:, 3]
                            + radii[chosen, None])) if chosen else 0.)
    if outside > 1e-9:
        raise ValueError("Added sphere escapes the original convex geometry")
    return result, {'added': count, 'probe_count': len(probes),
                    'mean_uncovered_depth_before_m': float(original.mean()),
                    'mean_uncovered_depth_after_m': float(deficit.mean()),
                    'max_added_sphere_outside_halfspace_m': outside}


def main():
    """Write an isolated refined config; never overwrite a source artifact."""
    import trimesh
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--robot-config', type=Path, required=True)
    parser.add_argument('--link', action='append', required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--additional-per-link', type=int, default=0)
    parser.add_argument('--construction-seed', type=int, default=20260923)
    args = parser.parse_args()
    if args.additional_per_link < 0:
        parser.error("additional-per-link must be nonnegative")
    args.output_root.mkdir(parents=True, exist_ok=False)
    geometry = json.loads(args.geometry.read_text())
    config = json.loads(args.robot_config.read_text())
    reports = []
    for link in args.link:
        if link not in config['kinematics']['collision_spheres']:
            raise ValueError(f'Unknown planning link: {link}')
        records = [r for r in geometry['geometries'] if r['robot'] and r['body'] == link]
        if len(records) != 1 or records[0]['shape']['kind'] not in ('mesh', 'convex'):
            raise ValueError('Expected one exported convex distance mesh per link')
        record = records[0]
        shape = record['shape']
        path = Path(shape['path'])
        if hashlib.sha256(path.read_bytes()).hexdigest() != shape['sha256']:
            raise ValueError(f'Source proximity mesh changed: {path}')
        transform = np.asarray(record['body_from_geometry'], dtype=float)
        if (transform.shape != (4, 4) or not np.isfinite(transform).all()
                or not np.allclose(transform[3], [0, 0, 0, 1])
                or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-9, rtol=0)
                or not np.isclose(np.linalg.det(transform[:3, :3]), 1)):
            raise ValueError('Expected a rigid geometry-to-body transform')
        mesh = trimesh.load(path, force='mesh')
        mesh.apply_scale(shape['scale'])
        mesh.apply_transform(transform)
        spheres, audit = expand_inner_spheres(config['kinematics']['collision_spheres'][link], mesh.vertices)
        augmentation = {}
        if args.additional_per_link:
            mesh = mesh.convex_hull
            points, faces = trimesh.sample.sample_surface(mesh, 3000, seed=args.construction_seed)
            spheres, augmentation = augment_inner_spheres(
                spheres, mesh.vertices, points, mesh.face_normals[faces], args.additional_per_link)
        config['kinematics']['collision_spheres'][link] = spheres
        reports.append({'link': link, 'mesh_sha256': shape['sha256'],
                        'audit': audit, 'augmentation': augmentation})
    if sum(map(len, config['kinematics']['collision_spheres'].values())) >= 1024:
        raise ValueError('Refined model exceeds the current CUDA sphere limit')
    report = {'construction_seed': args.construction_seed, 'scope': 'Selected interior spheres only; not full coverage or dynamic certification',
              'source_config_sha256': hashlib.sha256(args.robot_config.read_bytes()).hexdigest(),
              'source_geometry_sha256': hashlib.sha256(args.geometry.read_bytes()).hexdigest(),
              'construction_inward_padding_m': 1e-8, 'links': reports,
              'num_spheres': sum(map(len, config['kinematics']['collision_spheres'].values()))}
    (args.output_root / 'robot_config.json').write_text(json.dumps(config, indent=2)+'\n')
    (args.output_root / 'refinement_report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({r['link']:sum(x['expanded'] for x in r['audit']) for r in reports}))


if __name__ == '__main__':
    main()
