#!/usr/bin/env python3
"""Generate reviewable BT artifacts from a SceneSmith environment and task plan."""

import argparse
import json
from pathlib import Path

from examples.online_manipulation.generated_bt_navigation import (
    compile_model_response,
    generate_tree,
    load_generation_inputs,
    planning_prompt,
    to_dict,
    to_mdsl,
    to_mermaid,
)
from examples.online_manipulation.bt_visualization import write_viewer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--environment", required=True, type=Path)
    parser.add_argument("--task-plan", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--model-response", type=Path)
    parser.add_argument("--model-metadata", type=Path)
    args = parser.parse_args()
    environment, task_plan = load_generation_inputs(args.environment, args.task_plan)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "planning_prompt.txt").write_text(
        planning_prompt(environment, task_plan), encoding="utf-8"
    )
    if args.model_response:
        raw_response = args.model_response.read_text(encoding="utf-8")
        response, root = compile_model_response(environment, task_plan, raw_response)
        generation = (
            json.loads(args.model_metadata.read_text(encoding="utf-8"))
            if args.model_metadata
            else {"mode": "recorded_vlm_response"}
        )
        mdsl = to_mdsl(root)
        artifact = {
            "schema": "scenesmith.vlm_bt.generated.v1",
            "raw_response": raw_response,
            "parsed_response": response,
            "generation": generation,
            "mdsl": mdsl,
            "mdsl_sha256": __import__("hashlib").sha256(mdsl.encode()).hexdigest(),
            "tree": to_dict(root),
        }
        (args.output_dir / "generated_plan.json").write_text(
            json.dumps(artifact, indent=2) + "\n", encoding="utf-8"
        )
    else:
        root = generate_tree(environment, task_plan)
    (args.output_dir / "generated_bt.mdsl").write_text(to_mdsl(root), encoding="utf-8")
    (args.output_dir / "generated_bt.json").write_text(
        json.dumps(to_dict(root), indent=2) + "\n", encoding="utf-8"
    )
    (args.output_dir / "generated_bt.mmd").write_text(to_mermaid(root), encoding="utf-8")
    write_viewer(to_dict(root), args.output_dir, plan=artifact if args.model_response else None)
    print(args.output_dir.resolve())


if __name__ == "__main__":
    main()
