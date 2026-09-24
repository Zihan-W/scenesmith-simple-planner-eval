"""Portable cuTAMP selection evidence; static rank is not dynamic success."""

import json
import math
from pathlib import Path


SCORE_FIELDS = ["gap_imbalance_m", "negative_min_lift_joint_margin_rad",
                "abs_grasp_lateral_offset_m"]


def selection_evidence(selection):
    """Name the actual stopping policy and the lexicographically minimized rank.

    The second raw score component is negative; named quality exposes its
    positive physical margin. No feasible selection is represented by null.
    """
    window = selection["quality_window_after_first_pass"]
    base_policy = ("first_pass" if window == 0 else
                   "full_budget" if window is None else "quality_window")
    adaptive = selection["adaptive_budget"]
    score = selection["selected_score"]
    if score is not None:
        if len(score) != 3 or not all(math.isfinite(value) for value in score):
            raise ValueError("Selected cuTAMP score must contain three finite values")
        quality = dict(zip(SCORE_FIELDS, score))
        quality["min_lift_joint_margin_rad"] = -quality.pop("negative_min_lift_joint_margin_rad")
    else:
        quality = None
    return {
        "selection_policy": "adaptive_" + base_policy if adaptive is not None else base_policy,
        "quality_window_after_first_pass": window,
        "adaptive_final_mode": adaptive["mode"] if adaptive is not None else None,
        "candidate_order": selection["candidate_order"],
        "stop_reason": selection["stop_reason"],
        "selected_particle": selection["selected_particle"],
        "selected_score": list(score) if score is not None else None,
        "selected_score_fields": list(SCORE_FIELDS),
        "score_order": "lexicographic_minimize",
        "selected_quality": quality,
    }


def collect_selection_evidence(run):
    """Collect every completed solve selection, including failed selections.

    A killed solve may have no finalized selection file; it is not invented.
    Corrupt finalized evidence fails loudly.
    """
    return [{"solve": path.parent.name, **selection_evidence(json.loads(path.read_text()))}
            for path in sorted(Path(run).glob("cutamp/solve_*/candidate_selection.json"))]


def collect_run_selection_evidence(run):
    """Expose resolved per-run settings at the same location as selections.

    A worker killed before configuration was written has unknown settings,
    explicitly null; this is not interpreted as a disabled policy.
    """
    path = Path(run) / "planner_config.json"
    config = json.loads(path.read_text()) if path.exists() else None
    return {
        "cutamp_settings": config["cutamp"] if config is not None else None,
        "grasp_compensation": (config["settings"].get("grasp_compensation")
                               if config is not None else None),
        "selection_config_recorded": config is not None,
        "cutamp_selections": collect_selection_evidence(run),
    }
