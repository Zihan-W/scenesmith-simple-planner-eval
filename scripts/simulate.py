#!/usr/bin/env python3
import argparse
import json
from pathlib import Path
from typing import List, Tuple

import numpy as np

from pydrake.geometry import StartMeshcat
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import TrajectorySource, Demultiplexer, Multiplexer, Adder, Gain
from pydrake.trajectories import (
    CompositeTrajectory,
    PathParameterizedTrajectory,
    PiecewisePolynomial,
)
from pydrake.multibody.optimization import Toppra, CalcGridPointsOptions

# Robotic Manipulation course "manipulation" python package.
from manipulation.station import LoadScenario, MakeHardwareStation, MakeMultibodyPlant


def _ensure_model_drivers(yaml_text: str) -> str:
    if "model_drivers:" in yaml_text:
        return yaml_text.rstrip() + "\n"
    drivers_block = """
model_drivers:
  mobile_iiwa: !InverseDynamicsDriver {}
  wsg_50: !SchunkWsgDriver {}
"""
    return yaml_text.rstrip() + "\n" + drivers_block.lstrip()


def _load_plan(plan_path: Path) -> Tuple[np.ndarray, int, List[str]]:
    data = json.loads(plan_path.read_text())
    q_list = data["q"]
    nq = int(data["nq"])
    segments = list(data.get("segments", []))
    q = np.array(q_list, dtype=float)
    if q.ndim != 2 or q.shape[1] != nq:
        raise ValueError(f"Plan q has shape {q.shape}, expected (N,{nq}).")
    return q, nq, segments


def _pl_path_to_traj(path: np.ndarray) -> CompositeTrajectory:
    """Turn waypoint list into a CompositeTrajectory with unit-time segments.

    Uses cubic segments with C2 continuity imposed locally via endpoint second-derivs = 0.
    """
    nq = path.shape[1]
    segments = []
    for i in range(1, path.shape[0]):
        # Each segment is [i-1, i] in time (unit duration).
        seg = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            np.array([float(i - 1), float(i)]),
            np.array([path[i - 1], path[i]]).T,
            np.zeros(nq),
            np.zeros(nq),
        )
        segments.append(seg)
    return CompositeTrajectory(segments)


def _retime_toppra(
    plant,
    rough_traj,
    n_gridpoints=200,
    ee_vel_limit=None,
    ee_accel_limit=None,
):
    t0 = rough_traj.start_time()
    t1 = rough_traj.end_time()
    gridpoints = Toppra.CalcGridPoints(rough_traj, CalcGridPointsOptions(min_points=n_gridpoints))

    vel_multiplier = 1.0
    accel_multiplier = 1.0
    effort_multiplier = 1.0

    toppra = Toppra(rough_traj, plant, gridpoints)

    toppra.AddJointVelocityLimit(
        vel_multiplier * plant.GetVelocityLowerLimits(),
        vel_multiplier * plant.GetVelocityUpperLimits(),
    )
    toppra.AddJointAccelerationLimit(
        accel_multiplier * plant.GetAccelerationLowerLimits(),
        accel_multiplier * plant.GetAccelerationUpperLimits(),
    )
    toppra.AddJointTorqueLimit(
        effort_multiplier * plant.GetEffortLowerLimits(),
        effort_multiplier * plant.GetEffortUpperLimits(),
    )

    # ----------------------------
    # End-effector (frame) limits (optional)
    # Spatial velocity / acceleration measured AND expressed in world frame.
    # ----------------------------
    if ee_vel_limit is not None or ee_accel_limit is not None:
        wsg_instance = plant.GetModelInstanceByName("wsg_50")
        ee_frame = plant.GetFrameByName("body", wsg_instance)  # wsg_50::body

        if ee_vel_limit is not None:
            s = float(ee_vel_limit)
            lower_v = -s * np.ones(6)
            upper_v =  s * np.ones(6)
            toppra.AddFrameVelocityLimit(ee_frame, lower_v, upper_v)

        if ee_accel_limit is not None:
            a = float(ee_accel_limit)
            lower_a = -a * np.ones(6)
            upper_a =  a * np.ones(6)
            toppra.AddFrameAccelerationLimit(ee_frame, lower_a, upper_a)

    time_traj = toppra.SolvePathParameterization()
    if time_traj is None:
        raise RuntimeError("TOPPRA failed to find a time parameterization.")
    return PathParameterizedTrajectory(rough_traj, time_traj)


def _make_piecewise_hold(times: np.ndarray, samples: np.ndarray) -> PiecewisePolynomial:
    """Piecewise-linear hold trajectory through samples at given times.
    samples: shape (N, d)
    returns: PiecewisePolynomial.FirstOrderHold
    """
    return PiecewisePolynomial.FirstOrderHold(times, samples.T)


def main():
    parser = argparse.ArgumentParser(
        description="Build a manipulation.station HardwareStation and play a retimed plan"
    )
    parser.add_argument("scenario_yaml", type=str, help="Path to directives/scenario .yaml")
    parser.add_argument("robot_plan_json", type=str, help="Path to robot_plan.json (waypoints)")
    parser.add_argument(
        "--package-xml",
        type=str,
        action="append",
        default=[],
        help="Path to a package.xml file (may be repeated).",
    )
    parser.add_argument(
        "--record-html",
        type=str,
        default="simulation.html",
        help="Write a static Meshcat recording to this HTML file (default: simulation.html).",
    )
    parser.add_argument(
        "--ee-vel",
        type=float,
        default=None,
        help="End-effector spatial velocity component limit (applied to all 6 components) in world frame. "
             "Units: rad/s for rotational, m/s for translational. Default: None (no limit).",
    )
    parser.add_argument(
        "--ee-accel",
        type=float,
        default=None,
        help="End-effector spatial acceleration component limit (applied to all 6 components) in world frame. "
             "Units: rad/s^2 for rotational, m/s^2 for translational. Default: None (no limit).",
    )

    args = parser.parse_args()

    scenario_path = Path(args.scenario_yaml)
    plan_path = Path(args.robot_plan_json)
    if not scenario_path.exists():
        raise FileNotFoundError(scenario_path)
    if not plan_path.exists():
        raise FileNotFoundError(plan_path)

    # Load + patch scenario YAML.
    yaml_text = _ensure_model_drivers(scenario_path.read_text())

    # Meshcat
    meshcat = StartMeshcat()

    # Parse scenario
    scenario = LoadScenario(data=yaml_text)

    # ----------------------------
    # Build a robot-only plant for TOPPRA
    # ----------------------------
    # We only include the robot model instance in this plant so TOPPRA sees
    # exactly the robot DOFs (no scene objects).
    toppra_plant = MakeMultibodyPlant(
        scenario,
        model_instance_names=["mobile_iiwa", "wsg_50"],
        add_frozen_child_instances=True,  # keep anything welded to the robot as frozen if needed
        package_xmls=[str(Path(p)) for p in args.package_xml],
    )

    # ----------------------------
    # Load plan and split robot vs gripper
    # ----------------------------
    q_all, nq, segments = _load_plan(plan_path)

    # Your plan is 13D: [base/arm ... , left_finger, right_finger]
    # We'll assume the last 2 entries are WSG joints; retime the rest.
    if nq < 3:
        raise ValueError("Expected nq >= 3.")
    nq_gripper = 2
    nq_robot = nq - nq_gripper
    q_robot = q_all[:, :nq_robot]
    q_wsg = q_all[:, nq_robot:]  # shape (N,2)

    # Safety check: robot-only plant dimension must match nq_robot.
    # Use num_positions across *all* instances in that plant (it should basically be just the robot).
    if toppra_plant.num_positions() != nq:
        raise ValueError(
            f"Plan nq={nq}, but TOPPRA plant has num_positions={toppra_plant.num_positions()}."
        )

    # Rough path traj (unit-time per segment)
    rough_traj = _pl_path_to_traj(q_all)

    # TOPPRA retime robot traj
    timed_robot_traj = _retime_toppra(
        toppra_plant,
        rough_traj,
        ee_vel_limit=args.ee_vel,
        ee_accel_limit=args.ee_accel,
    )

    # ----------------------------
    # Build station and wire trajectory sources
    # ----------------------------
    station = MakeHardwareStation(
        scenario,
        meshcat=meshcat,
        package_xmls=[str(Path(p)) for p in args.package_xml],
        hardware=False,
    )

    builder = DiagramBuilder()
    station_sys = builder.AddSystem(station)

    # Suppose timed_q_traj is your TOPPRA-retimed POSITION trajectory of size nq_total (=13).
    # If you currently have timed_robot_traj as 13D, rename it for clarity:
    timed_q_traj = timed_robot_traj
    timed_v_traj = timed_q_traj.MakeDerivative(1)   # size 13

    nq_total = timed_q_traj.rows()   # should be 13

    q_source = builder.AddSystem(TrajectorySource(timed_q_traj))
    v_source = builder.AddSystem(TrajectorySource(timed_q_traj.MakeDerivative(1)))

    # Demux full q and v into scalars
    q_demux_all = builder.AddSystem(Demultiplexer([1] * nq_total))
    v_demux_all = builder.AddSystem(Demultiplexer([1] * nq_total))
    builder.Connect(q_source.get_output_port(), q_demux_all.get_input_port())
    builder.Connect(v_source.get_output_port(), v_demux_all.get_input_port())

    # Build robot q,v by muxing outputs 0..(nq_robot-1)
    nq_wsg = 2
    nq_robot = nq_total - nq_wsg

    q_robot_mux = builder.AddSystem(Multiplexer([1] * nq_robot))
    v_robot_mux = builder.AddSystem(Multiplexer([1] * nq_robot))
    for i in range(nq_robot):
        builder.Connect(q_demux_all.get_output_port(i), q_robot_mux.get_input_port(i))
        builder.Connect(v_demux_all.get_output_port(i), v_robot_mux.get_input_port(i))

    # desired_state = [q_robot; v_robot]
    state_mux = builder.AddSystem(Multiplexer([nq_robot, nq_robot]))
    builder.Connect(q_robot_mux.get_output_port(), state_mux.get_input_port(0))
    builder.Connect(v_robot_mux.get_output_port(), state_mux.get_input_port(1))
    builder.Connect(state_mux.get_output_port(),
                    station_sys.GetInputPort("mobile_iiwa.desired_state"))

    neg = builder.AddSystem(Gain(-1.0, 1))     # multiply by -1
    wsg_diff = builder.AddSystem(Adder(2, 1))

    builder.Connect(q_demux_all.get_output_port(12),
                    wsg_diff.get_input_port(0))   # +q12
    builder.Connect(q_demux_all.get_output_port(11),
                    neg.get_input_port())          # q11
    builder.Connect(neg.get_output_port(),
                    wsg_diff.get_input_port(1))   # -q11

    builder.Connect(wsg_diff.get_output_port(),
                    station_sys.GetInputPort("wsg_50.position"))

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    diagram.ForcedPublish(context)

    sim = Simulator(diagram)
    sim.Initialize()

    print(f"Loaded scenario from: {scenario_path}")
    print(f"Loaded plan from: {plan_path}")
    if segments:
        print("Plan segments:")
        for s in segments:
            print(f"  - {s}")

    t0 = timed_q_traj.start_time()
    t1 = timed_q_traj.end_time()
    print(f"Timed duration: {t1 - t0:.3f} s")
    print("Meshcat server running. Ctrl+C to exit.")

    try:
        meshcat.StartRecording()
        sim.AdvanceTo(t1)
        meshcat.StopRecording()
        meshcat.PublishRecording()
        html = meshcat.StaticHtml()
        Path(args.record_html).write_text(html)
        print(f"Wrote Meshcat recording to: {args.record_html}")
        while True:
            pass
    except KeyboardInterrupt:
        print("\nExiting.")


if __name__ == "__main__":
    main()
