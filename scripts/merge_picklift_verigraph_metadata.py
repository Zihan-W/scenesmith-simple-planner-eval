#!/usr/bin/env python3
"""Bind VeriGraph perception output to a SceneSmith PickLift snapshot."""

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-snapshot", required=True, type=Path)
    parser.add_argument("--catalog", required=True, type=Path)
    parser.add_argument("--scene-graph", required=True, type=Path)
    parser.add_argument("--verigraph-metadata", required=True, type=Path)
    parser.add_argument("--perception-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    runtime = json.loads(args.runtime_snapshot.read_text())
    catalog = json.loads(args.catalog.read_text())
    graph = json.loads(args.scene_graph.read_text())
    metadata = json.loads(args.verigraph_metadata.read_text())
    relation = runtime["task"]["initial_relation_from_task_binding"]
    if graph.get("relations") != [relation]:
        raise ValueError(f"VeriGraph relation does not match SceneSmith binding: {graph.get('relations')}")
    if set(graph.get("nodes", ())) != set(catalog["entities"]):
        raise ValueError("VeriGraph did not identify the complete configured PickLift scene")
    result = {
        "schema": "scenesmith.verigraph_pick.environment.v1",
        "runtime_snapshot": runtime,
        "verigraph": {
            "parser": "verigraph.core.parse.parse_llm_response_to_graph",
            "checker": "verigraph.core.simple_plan_checker.SimplePlanChecker",
            "upstream_commit": "5c07fb8f228049e228b53637e3867650f6854fa5",
            "catalog": catalog,
            "scene_graph": graph,
            "metadata": metadata,
            "perception_manifest": json.loads(args.perception_manifest.read_text()),
            "input_sha256": {
                "catalog": sha256(args.catalog),
                "scene_graph": sha256(args.scene_graph),
                "metadata": sha256(args.verigraph_metadata),
                "perception_manifest": sha256(args.perception_manifest),
            },
        },
        "available_skills": runtime["available_bt_skills"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(args.output.resolve())


if __name__ == "__main__":
    main()
