#!/usr/bin/env python3
"""Create compact result data and plots for the generated BT experiment."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle


def _load_episodes(run_root):
    episodes = []
    for summary_path in sorted(run_root.glob("episode_*/summary.json")):
        summary = json.loads(summary_path.read_text())
        with (summary_path.parent / "trace.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        diagnostics = json.loads(rows[-1]["policy_diagnostics_json"])
        xy = [json.loads(row["base_json"])["pose"]["translation_m"][:2] for row in rows]
        episodes.append((summary_path.parent.name, summary, diagnostics, xy))
    if not episodes:
        raise ValueError(f"No episode summaries under {run_root}")
    return episodes


def _plot_trajectories(episodes, output_path):
    fig, ax = plt.subplots(figsize=(8.8, 5.2), constrained_layout=True)
    for name, summary, diagnostics, xy in episodes:
        ax.plot([p[0] for p in xy], [p[1] for p in xy], label=f"seed {summary['seed']}")
    ax.add_patch(Rectangle((1.25, -0.3), 0.5, 0.6, color="#2456d6", alpha=0.8, label="obstacle"))
    ax.scatter([0], [0], marker="o", s=70, color="#16803c", zorder=5, label="start")
    ax.scatter([2.8], [0], marker="*", s=180, color="#d9363e", zorder=5, label="goal")
    ax.set(title="Generated BT: three successful navigation trajectories", xlabel="world x (m)", ylabel="world y (m)")
    ax.axis("equal")
    ax.grid(alpha=0.25)
    ax.legend(loc="best", ncol=2)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def _plot_tree(tree, output_path):
    nodes = []
    edges = []

    def visit(node, depth=0, parent=None):
        index = len(nodes)
        nodes.append((node, depth))
        if parent is not None:
            edges.append((parent, index))
        for child in node["children"]:
            visit(child, depth + 1, index)

    visit(tree)
    depths = {}
    for index, (_, depth) in enumerate(nodes):
        depths.setdefault(depth, []).append(index)
    positions = {}
    for depth, indexes in depths.items():
        span = len(indexes)
        for column, index in enumerate(indexes):
            positions[index] = ((column + 1) / (span + 1), 1 - (depth + 0.7) / (len(depths) + 0.5))

    colors = {"root": "#e2e8f0", "selector": "#f3e8ff", "sequence": "#dbeafe", "condition": "#ccfbf1", "action": "#e0f2fe"}
    fig, ax = plt.subplots(figsize=(12, 6.4), constrained_layout=True)
    for parent, child in edges:
        x1, y1 = positions[parent]
        x2, y2 = positions[child]
        ax.annotate("", xy=(x2, y2 + 0.035), xytext=(x1, y1 - 0.035), arrowprops={"arrowstyle": "->", "color": "#64748b", "lw": 1.5})
    for index, (node, depth) in enumerate(nodes):
        x, y = positions[index]
        label = node["name"] or node["kind"]
        if node["args"]:
            label += "\n" + ", ".join(node["args"])
        width = 0.24 if node["kind"] in ("action", "condition") else 0.15
        box = FancyBboxPatch((x - width / 2, y - 0.045), width, 0.09, boxstyle="round,pad=0.012", facecolor=colors[node["kind"]], edgecolor="#334155", linewidth=1.3)
        ax.add_patch(box)
        ax.text(x, y, label, ha="center", va="center", fontsize=8.5)
    ax.set(xlim=(0, 1), ylim=(0, 1), title="Generated SceneSmith Behavior Tree")
    ax.axis("off")
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", required=True, type=Path)
    parser.add_argument("--generated-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    episodes = _load_episodes(args.run_root)
    tree = json.loads((args.generated_dir / "generated_bt.json").read_text())
    _plot_trajectories(episodes, args.output_dir / "trajectory_comparison.png")
    _plot_tree(tree, args.output_dir / "generated_bt.png")
    results = {
        "consecutive_successes": sum(int(item[1]["success"]) for item in episodes),
        "all_successful": all(item[1]["success"] for item in episodes),
        "episodes": [
            {
                "episode": name,
                "seed": summary["seed"],
                "success": summary["success"],
                "termination_reason": summary["termination_reason"],
                "episode_time_s": summary["episode_time_s"],
                "policy_steps": summary["policy_steps"],
                "metrics": summary["task"]["metrics"],
                "final_bt_status": diagnostics["tree_status"],
                "tamp_used": diagnostics["tamp_used"],
                "mdsl_sha256": diagnostics["mdsl_sha256"],
            }
            for name, summary, diagnostics, _ in episodes
        ],
    }
    (args.output_dir / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
