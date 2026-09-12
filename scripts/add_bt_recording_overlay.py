#!/usr/bin/env python3
"""Add a synchronized behavior-tree panel to an existing Meshcat recording."""

import argparse
import csv
import json
from pathlib import Path

from src.online_manipulation.recording_overlay import (
    build_behavior_tree_timeline,
    inject_behavior_tree_overlay,
    write_behavior_tree_timeline,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", required=True, type=Path)
    parser.add_argument("--trace", required=True, type=Path)
    parser.add_argument("--tree", required=True, type=Path)
    parser.add_argument("--title", default="Behavior Tree")
    parser.add_argument("--recording-start-time-s", type=float, default=0.0)
    parser.add_argument("--timeline-output", type=Path)
    args = parser.parse_args()
    with args.trace.open(encoding="utf-8") as stream:
        trace = list(csv.DictReader(stream))
    tree_document = json.loads(args.tree.read_text(encoding="utf-8"))
    tree = tree_document.get("tree", tree_document)
    payload = build_behavior_tree_timeline(
        definition={"title": args.title, "tree": tree},
        trace=trace,
        recording_start_time_s=args.recording_start_time_s,
    )
    if payload is None:
        raise ValueError("Trace contains no behavior_tree diagnostics")
    timeline = args.timeline_output or args.recording.with_name("bt_timeline.json")
    write_behavior_tree_timeline(timeline, payload)
    changed = inject_behavior_tree_overlay(args.recording, payload)
    print(json.dumps({"recording": str(args.recording.resolve()), "timeline": str(timeline.resolve()), "injected": changed}))


if __name__ == "__main__":
    main()
