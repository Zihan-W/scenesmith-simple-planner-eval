"""Dependency-free, offline visualization for generated SceneSmith BT trees."""

from __future__ import annotations

import json
from pathlib import Path


_TEMPLATE = Path(__file__).with_name("bt_viewer.html")


def render_viewer(tree: dict, *, plan: dict | None = None) -> str:
    """Render one generated tree without implying that it has been executed."""

    if not isinstance(tree, dict) or tree.get("kind") != "root":
        raise ValueError("BT visualization requires a root node")
    payload = json.dumps(
        {"tree": tree, "plan": plan}, ensure_ascii=False, separators=(",", ":")
    ).replace("<", "\\u003c")
    return _TEMPLATE.read_text(encoding="utf-8").replace("__BT_DATA__", payload)


def write_viewer(tree: dict, output_dir: Path, *, plan: dict | None = None) -> Path:
    """Write a self-contained HTML viewer beside generated_bt.json."""

    destination = Path(output_dir) / "generated_bt.html"
    page = render_viewer(tree, plan=plan)
    destination.write_text(page, encoding="utf-8")
    return destination
