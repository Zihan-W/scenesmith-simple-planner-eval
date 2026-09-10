#!/usr/bin/env python3
"""Compile a recorded Microsoft-style PickLift BT response."""

import argparse
import hashlib
import json
from pathlib import Path

from examples.online_manipulation.generated_bt_picklift import (
    canonical_tree, compile_response, load_inputs, to_dict, to_mdsl, to_mermaid)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--environment", required=True, type=Path)
    parser.add_argument("--task-plan", required=True, type=Path)
    parser.add_argument("--model-response", required=True, type=Path)
    parser.add_argument("--model-metadata", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    environment, task_plan = load_inputs(args.environment, args.task_plan)
    raw = args.model_response.read_text()
    response, root = compile_response(environment, task_plan, raw)
    mdsl = to_mdsl(root)
    artifact = {"schema": "scenesmith.microsoft_picklift.generated.v1",
                "raw_response": raw, "parsed_response": response,
                "generation": json.loads(args.model_metadata.read_text()),
                "mdsl": mdsl, "mdsl_sha256": hashlib.sha256(mdsl.encode()).hexdigest(),
                "tree": to_dict(root)}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "generated_plan.json").write_text(json.dumps(artifact, indent=2) + "\n")
    (args.output_dir / "generated_bt.mdsl").write_text(mdsl)
    (args.output_dir / "generated_bt.json").write_text(json.dumps(to_dict(root), indent=2) + "\n")
    (args.output_dir / "generated_bt.mmd").write_text(to_mermaid(root))
    print(args.output_dir.resolve())


if __name__ == "__main__":
    main()
