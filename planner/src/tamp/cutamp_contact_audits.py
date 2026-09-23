"""Exact checks for contacts invariant under the optimizer's selected variables.

Only locked relative self geometry and constant-height support on a verified
horizontal floor are substituted. All other collision costs and Drake checks
remain required. No penetration tolerance or collision permission is invented.
"""

from collections import defaultdict
import hashlib
import itertools
import json
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from .cutamp_backend.geometry_export import stationary_support_check


def _rigid_components(robot):
    """Group links connected entirely through fixed or explicitly locked joints."""
    kin = robot["kinematics"]
    root = ET.parse(kin["urdf_path"]).getroot()
    parent = {link.attrib["name"]: link.attrib["name"] for link in root.findall("link")}

    def find(name):
        while parent[name] != name:
            name = parent[name]
        return name

    for joint in root.findall("joint"):
        if joint.attrib["type"] == "fixed" or joint.attrib["name"] in kin["lock_joints"]:
            a, b = joint.find("parent").attrib["link"], joint.find("child").attrib["link"]
            parent[find(a)] = find(b)
    return {name: find(name) for name in parent}


def audit_locked_self_pairs(query, geometry, robot):
    """Replace false positive spheres only after proving exact rigid-pair safety."""
    components = _rigid_components(robot)
    kin = robot["kinematics"]
    proximity = query.plant.get_geometry_query_input_port().Eval(query.context)
    inspector = proximity.inspector()
    ids = {int(item.get_value()): item for item in inspector.GetAllGeometryIds()}
    records = defaultdict(list)
    for item in geometry["geometries"]:
        if item["robot"]:
            records[item["body"]].append(item)
    spheres = {}
    for name, values in kin["collision_spheres"].items():
        matrix = np.asarray(geometry["base_from_robot_body"][name])
        centers = np.asarray([value["center"] for value in values])
        spheres[name] = (centers @ matrix[:3, :3].T + matrix[:3, 3],
                         np.asarray([value["radius"] for value in values]))
    audits = []
    ignored = kin["self_collision_ignore"]
    for a, b in itertools.combinations(spheres, 2):
        if components[a] != components[b] or b in ignored.get(a, ()) or a in ignored.get(b, ()):
            continue
        ca, ra = spheres[a]
        cb, rb = spheres[b]
        overlap = float(np.max(ra[:, None] + rb - np.linalg.norm(ca[:, None] - cb, axis=-1)))
        if overlap <= 0:
            continue
        distances = []
        for ga, gb in itertools.product(records[a], records[b]):
            first, second = ids[ga["id"]], ids[gb["id"]]
            if inspector.CollisionFiltered(first, second):
                continue
            distance = float(proximity.ComputeSignedDistancePairClosestPoints(first, second).distance)
            distances.append({"geometry_a": ga["id"], "geometry_b": gb["id"], "distance_m": distance})
        valid = bool(distances) and all(pair["distance_m"] >= 0 for pair in distances)
        audits.append({"body_a": a, "body_b": b, "rigid_component": components[a],
                       "approximate_overlap_m": overlap, "exact_pairs": distances,
                       "substituted": valid})
        if valid:
            ignored.setdefault(a, []).append(b)
            ignored.setdefault(b, []).append(a)
    return {"scope": "exact_nonpenetration_for_locked_relative_self_pairs_only",
            "locked_joints": dict(kin["lock_joints"]), "pairs": audits}


def _shape_corners(shape):
    """Conservative bounds of the original proximity geometry, not fitted spheres."""
    kind = shape["kind"]
    if kind in {"mesh", "convex"}:
        from pathlib import Path
        path = Path(shape["path"])
        if hashlib.sha256(path.read_bytes()).hexdigest() != shape["sha256"]:
            raise ValueError("Support proximity mesh changed")
        mesh = trimesh.load(path, force="mesh")
        mesh.apply_scale(shape["scale"])
        bounds = mesh.bounds
    elif kind == "cylinder":
        extent = np.array([shape["radius_m"], shape["radius_m"], shape["length_m"] / 2])
        bounds = np.stack((-extent, extent))
    elif kind == "box":
        extent = np.asarray(shape["size_m"]) / 2
        bounds = np.stack((-extent, extent))
    else:
        raise ValueError(f"Unsupported exact planar support shape: {kind}")
    return np.array(list(itertools.product(*bounds.T)))


def audit_planar_support(query, geometry, robot):
    """Prove existing wheel/floor checks invariant over the entire base domain."""
    audit = stationary_support_check(query, geometry)
    if not audit["valid"]:
        raise ValueError("Initial support does not satisfy existing exact contact rules")
    components = _rigid_components(robot)
    kin = robot["kinematics"]
    records = {item["id"]: item for item in geometry["geometries"]}
    base = np.asarray(geometry["observed_robot_base_world_matrix"])
    offsets = np.asarray(geometry["base_candidates_xyz_yaw"])[:, :2] - base[:2, 3]
    coverage = []
    for pair in audit["pairs"]:
        support = records[pair["robot_geometry_id"]]
        floor = records[pair["ground_geometry_id"]]
        if components[support["body"]] != components[kin["base_link"]]:
            raise ValueError("Support geometry moves relative to the optimization base")
        transform = np.asarray(floor["world_from_geometry"])
        if floor["shape"]["kind"] != "box" or not np.allclose(
                transform[:3, 2], [0, 0, 1], atol=1e-9, rtol=0):
            raise ValueError("Planar support substitution needs a horizontal box floor")
        x_support = np.asarray(support["world_from_geometry"])
        vertices = _shape_corners(support["shape"]) @ x_support[:3, :3].T + x_support[:3, 3]
        # Cover every original geometry point at every hull vertex; convexity
        # then covers the entire allowed XY domain at the fixed height/heading.
        vertices = vertices[None] + np.column_stack((offsets, np.zeros(len(offsets))))[:, None]
        local = (vertices - transform[:3, 3]) @ transform[:3, :3]
        margin = np.asarray(floor["shape"]["size_m"])[:2] / 2 - np.abs(local[..., :2])
        if float(margin.min()) <= 0:
            raise ValueError("Parking domain crosses a support floor boundary")
        floor_top = float(transform[2, 3] + floor["shape"]["size_m"][2] / 2)
        if float(vertices[..., 2].max()) < floor_top:
            raise ValueError("Support geometry is below the floor top")
        coverage.append({"robot_geometry_id": support["id"], "ground_geometry_id": floor["id"],
                         "minimum_horizontal_boundary_margin_m": float(margin.min())})
    return {"scope": "fixed_height_heading_rigid_support_over_verified_horizontal_floor",
            "stationary_audit": audit, "coverage": coverage,
            "base_candidates_xyz_yaw": geometry["base_candidates_xyz_yaw"],
            "observed_robot_base_world_matrix": geometry["observed_robot_base_world_matrix"]}
