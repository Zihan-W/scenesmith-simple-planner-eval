#!/usr/bin/env python3
import argparse
import json

# --- Task loading ---
def load_task(task_path):
    with open(task_path, "r") as f:
        task = json.load(f)
    return task.get("robot_start_xy", [0.0, 0.0])

# --- Generate robot + gripper YAML as raw text ---
def make_robot_and_gripper_yaml(start_xy):
    x, y = start_xy
    return f"""- add_model:
    name: mobile_iiwa
    file: package://mobile_iiwa/mobile_iiwa.urdf
    default_joint_positions:
      world_x_joint: [{x}]
      world_y_joint: [{y}]
- add_weld:
    parent: world
    child: mobile_iiwa::base
    X_PC:
      translation: [0.0, 0.0, 0.0]
      rotation: !AngleAxis
        angle_deg: 0.0
        axis: [0.0, 0.0, 1.0]

- add_model:
    name: wsg_50
    file: package://drake_models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf
    default_joint_positions:
      left_finger_sliding_joint: [-0.054]
      right_finger_sliding_joint: [0.054]
- add_weld:
    parent: mobile_iiwa::iiwa_link_7
    child: wsg_50::body
    X_PC:
      translation: [0, 0, 0.09]
      rotation: !Rpy {{ deg: [90, 0, 68] }}
"""

# --- Main ---
def main():
    parser = argparse.ArgumentParser(
        description="Prepend mobile_iiwa + wsg_50 to a Drake directives YAML file."
    )
    parser.add_argument("directives", help="Original directives YAML file")
    parser.add_argument("task_json", help="Task JSON file for start XY")
    parser.add_argument("output", help="Output directives YAML file")
    args = parser.parse_args()

    # Load task start position
    start_xy = load_task(args.task_json)

    # Generate robot + gripper YAML block
    robot_yaml = make_robot_and_gripper_yaml(start_xy)

    # Read existing directives file
    with open(args.directives, "r") as f:
        existing_text = f.read()

    # Prepend robot directive under 'directives:' key
    if existing_text.lstrip().startswith("directives:"):
        # Keep the 'directives:' line, insert robot after it
        lines = existing_text.splitlines()
        output_text = lines[0] + "\n" + robot_yaml + "\n".join(lines[1:]) + "\n"
    else:
        # No top-level 'directives:', just prepend everything
        output_text = "directives:\n" + robot_yaml + existing_text

    # Save output
    with open(args.output, "w") as f:
        f.write(output_text)

    print(f"Saved new directives file with mobile_iiwa + wsg_50 at '{args.output}'")

if __name__ == "__main__":
    main()
