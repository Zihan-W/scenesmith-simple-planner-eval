"""Optional original IIWA pick-place baseline, isolated from online Runtime.

This baseline uses offline IK/RRT/TOPPRA and the original sticky gripper
research model. It is not evidence of Zerith's physical PickLift behavior.
Every stage runs in an isolated process and failures stop the pipeline.
"""

import argparse
from pathlib import Path
import subprocess
import sys


def baseline_commands(scene_root, repository_root, output_root):
    """Return the four original stage invocations using absolute asset paths."""
    scene, repo, out = map(Path, (scene_root, repository_root, output_root))
    commands = scene / "combined_house/robot_commands.json"
    dmd = scene / "combined_house/house_furniture_welded.dmd.yaml"
    packages = ["--package-xml", str(scene / "package.xml"),
                "--package-xml", str(repo / "models/iiwa/package.xml")]
    robot = str(out / "robot_task.dmd.yaml")
    waypoints = str(out / "robot_waypoints.json")
    plan = str(out / "robot_plan.json")
    stages = [
        ("add_robot_to_directives", [str(dmd), str(commands), robot]),
        ("compute_grasp_config_noninteractive", [str(commands), robot, *packages,
                                                 "--out-waypoints", waypoints]),
        ("plan_robot_waypoints_rrt_noninteractive", [str(commands), robot, waypoints,
             *packages, "--out-traj", plan, "--shortcut-tries", "100", "--gripper-clearance"]),
        ("simulate_noninteractive", [robot, plan, *packages, "--friction-mult", "10",
             "--ee-vel", "1", "--ee-accel", "1", "--write-updated-scenario",
             str(out / "final.dmd.yaml"), "--record-html", str(out / "simulation.html")]),
    ]
    return [[sys.executable, "-B", "-m", "tools.iiwa." + name, *args]
            for name, args in stages]


def main():
    """Run once into a fresh directory; never delete or overwrite prior runs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-root", required=True, type=Path)
    parser.add_argument("--repository-root", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    args = parser.parse_args()
    scene, repo, out = (p.resolve() for p in
                        (args.scene_root, args.repository_root, args.output_root))
    for path in (scene / "combined_house/robot_commands.json",
                 scene / "combined_house/house_furniture_welded.dmd.yaml",
                 scene / "package.xml", repo / "models/iiwa/package.xml"):
        if not path.is_file():
            raise FileNotFoundError(f"IIWA baseline requires {path}")
    out.mkdir(parents=True, exist_ok=False)
    for index, command in enumerate(baseline_commands(scene, repo, out)):
        print(f"IIWA stage {index + 1}/4: {command[3]}", flush=True)
        with (out / f"stage_{index + 1}.log").open("w") as log:
            subprocess.run(command, cwd=out, stdout=log, stderr=subprocess.STDOUT,
                           check=True)
    print(f"IIWA pipeline completed; artifacts: {out}")


if __name__ == "__main__":
    main()
