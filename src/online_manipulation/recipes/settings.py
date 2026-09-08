"""Resolve finite data profiles without inheritance or implicit overrides."""

import json
from pathlib import Path


def read_pick_settings(path):
    """Resolve legacy CLI settings from profiles; accept explicit old user files."""
    path = Path(path)
    settings = json.loads(path.read_text())
    if "profiles" not in settings:
        return settings  # Explicit --environment-json backward compatibility.
    profiles = json.loads((path.parent / settings["profiles"]).read_text())
    task = profiles["task"][settings["task"]]
    return {"initial_state": profiles["initial_state"][settings["initial_state"]],
            "control": profiles["control"][settings["control"]],
            "scene_bindings": task["bindings"],
            "task": {k: v for k, v in task.items() if k not in ("kind", "bindings")}}
