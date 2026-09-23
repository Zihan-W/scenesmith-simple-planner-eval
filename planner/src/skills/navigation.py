"""Navigation corridor precheck for standalone skill execution.

Preserves the BT baseline's sampled corridor and turning envelope. The BT
module stays unchanged; both routes use simulation.control.navigation.Navigator.
"""

import math

import numpy as np
from pydrake.all import RigidTransform, RollPitchYaw

from simulation.src import StaticNavigationMap
from simulation.src.geometry.scene_geometry import resolve_ground_geometries


def certified_local_map(query, adapter, scenario, navigate):
    """Check translation and the differential-drive turning envelope.

    Sample all headings between the initial, forward/reverse travel, and final
    headings along the corridor. This conservatively covers the controller's
    blended turning; it remains a geometric precheck, not a dynamics proof.
    """
    query = query.fork()
    plant, context = query.plant, query.context
    instance = query.robot_model_instance
    body = plant.GetBodyByName(adapter.spec.base_link_name, instance)
    base_frame = plant.GetFrameByName(adapter.spec.base_link_name, instance)
    navigation_frame = plant.GetFrameByName(adapter.navigation_frame_name, instance)
    X_BN = base_frame.CalcPoseInWorld(context).inverse() @ navigation_frame.CalcPoseInWorld(context)
    x, y, yaw = map(float, navigate.args[:3])
    if navigate.args[3] != "world" or not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError("NavigateTo must have a finite world-frame goal")
    X_WB_start = base_frame.CalcPoseInWorld(context)
    X_WN_start = navigation_frame.CalcPoseInWorld(context)
    start_xy = X_WN_start.translation()[:2]
    start_yaw = X_WN_start.rotation().ToRollPitchYaw().yaw_angle()
    delta = np.array([x, y]) - start_xy
    wrap = lambda angle: math.atan2(math.sin(angle), math.cos(angle))
    heading = math.atan2(delta[1], delta[0]) if np.linalg.norm(delta) > 1e-9 else start_yaw
    if abs(wrap(heading - start_yaw)) > math.pi / 2:
        heading += math.pi  # Match Navigator's reverse-drive choice.
    travel_yaw = start_yaw + wrap(heading - start_yaw)
    end_yaw = travel_yaw + wrap(yaw - travel_yaw)
    low, high = min(start_yaw, travel_yaw, end_yaw), max(start_yaw, travel_yaw, end_yaw)
    headings = np.linspace(low, high, max(2, math.ceil((high - low) / math.radians(2)) + 1))
    count = max(31, math.ceil(np.linalg.norm(delta) / 0.01) + 1)
    poses = [(0.0, start_yaw, X_WB_start)]
    for alpha in np.linspace(0, 1, count):
        xy = start_xy + alpha * delta
        for sample_yaw in headings:
            X_WN = RigidTransform(RollPitchYaw(0, 0, sample_yaw), [*xy, 0])
            poses.append((alpha, sample_yaw, X_WN @ X_BN.inverse()))
    floor_ids = resolve_ground_geometries(
        plant, plant.get_geometry_query_input_port().Eval(context).inspector(),
        scenario.ground_body_names, scenario.ground_geometries,
    )
    for alpha, sample_yaw, pose in poses:
        plant.SetFreeBodyPose(context, body, pose)
        geometry_query = plant.get_geometry_query_input_port().Eval(context)
        inspector = geometry_query.inspector()
        for pair in geometry_query.ComputeSignedDistancePairwiseClosestPoints(0.01):
            if pair.id_A in floor_ids or pair.id_B in floor_ids:
                continue
            a = plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_A))
            b = plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_B))
            a_robot, b_robot = a.model_instance() == instance, b.model_instance() == instance
            if not (a_robot or b_robot):
                continue
            if a_robot and b_robot and {a.name(), b.name()} == {"neck_yaw_link", "neck_camera_link"}:
                continue  # fixed, measured CAD overlap; present before motion
            required = 0.005 if a_robot != b_robot else 0.0
            if pair.distance < required:
                raise ValueError(
                    f"Navigation corridor collision at alpha={alpha:.2f}, yaw={sample_yaw:.4f}: "
                    f"{a.name()} / {b.name()} = {pair.distance:.4f} m")
    bounds = (-4.5, -4.5, 4.5, 4.5)
    return StaticNavigationMap(obstacles=(), robot_radius_m=0.0, bounds=bounds,
                               resolution_m=0.05)
