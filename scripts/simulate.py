#!/usr/bin/env python3
import argparse
from pathlib import Path

from pydrake.geometry import StartMeshcat
from pydrake.systems.analysis import Simulator

# Robotic Manipulation course "manipulation" python package.
# Docs: https://manipulation.csail.mit.edu/python/station.html
from manipulation.station import LoadScenario, MakeHardwareStation


def _ensure_model_drivers(yaml_text: str) -> str:
    """Append model_drivers if missing.

    Assumes your YAML is a Scenario-like root (e.g., has 'directives:' at top level).
    """
    if "model_drivers:" in yaml_text:
        return yaml_text.rstrip() + "\n"

    # Append at root indentation.
    # InverseDynamicsDriver is supported as a model driver type. :contentReference[oaicite:2]{index=2}
    drivers_block = """
model_drivers:
  mobile_iiwa: !InverseDynamicsDriver {}
  wsg_50: !SchunkWsgDriver {}
"""
    return yaml_text.rstrip() + "\n" + drivers_block.lstrip()


def main():
    parser = argparse.ArgumentParser(
        description="Build a manipulation.station HardwareStation from directives YAML"
    )
    parser.add_argument("scenario_yaml", type=str, help="Path to directives/scenario .yaml")
    parser.add_argument(
        "--package-xml",
        type=str,
        action="append",
        default=[],
        help="Path to a package.xml file (may be repeated). Passed to MakeHardwareStation(package_xmls=...).",
    )
    args = parser.parse_args()

    scenario_path = Path(args.scenario_yaml)
    if not scenario_path.exists():
        raise FileNotFoundError(scenario_path)

    # Read + patch YAML to ensure model_drivers exist.
    yaml_text = scenario_path.read_text()
    yaml_text = _ensure_model_drivers(yaml_text)

    # Build station.
    meshcat = StartMeshcat()
    meshcat.Delete()

    scenario = LoadScenario(data=yaml_text)
    station = MakeHardwareStation(
        scenario,
        meshcat=meshcat,
        package_xmls=[str(Path(p)) for p in args.package_xml],
        hardware=False,
    )

    # Publish once so you see something immediately.
    context = station.CreateDefaultContext()
    station.ForcedPublish(context)

    # Optional: run a tiny bit of simulation so time-based publishers kick once.
    sim = Simulator(station)
    sim.Initialize()
    sim.AdvanceTo(0.01)

    print(f"Loaded scenario from: {scenario_path}")
    print("Meshcat server running. Press Ctrl+C to exit.")
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("\nExiting.")


if __name__ == "__main__":
    main()
