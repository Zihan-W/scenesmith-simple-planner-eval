#!/usr/bin/env python3
import argparse
import json

# --- Task loading ---
def load_task(task_path):
    with open(task_path, "r") as f:
        task = json.load(f)
    start_xy = task.get("robot_start_xy", [0.0, 0.0])
    return start_xy

# --- Generate robot directive as YAML string ---
def make_robot_directive_yaml(name, urdf_path, start_xy):
    x, y = start_xy
    yaml_block = f"""- add_model:
    name: {name}
    file: {urdf_path}
    default_joint_positions:
      world_x_joint: [{x}]
      world_y_joint: [{y}]
"""
    return yaml_block

# --- Main ---
def main():
    parser = argparse.ArgumentParser(
        description="Prepend a robot directive as raw YAML text."
    )
    parser.add_argument("directives", help="Original directives YAML file")
    parser.add_argument("robot_urdf", help="Robot URDF (package://...)")
    parser.add_argument("robot_name", help="Name of the robot model")
    parser.add_argument("task_json", help="Task JSON file")
    parser.add_argument("output", help="Output directives YAML file")
    args = parser.parse_args()

    # Load task start position
    start_xy = load_task(args.task_json)

    # Generate robot YAML block
    robot_yaml = make_robot_directive_yaml(args.robot_name, args.robot_urdf, start_xy)

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

    print(f"Saved new directives file with robot at '{args.output}'")

if __name__ == "__main__":
    main()
