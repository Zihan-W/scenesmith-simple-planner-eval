"""Evidence-based cuTAMP failures; optimizer search is not an UNSAT proof."""

from collections import Counter

from .failures import ProgramFailure


COST_CATEGORIES = {
    ("Collision", "robot_to_world"): "collision",
    ("Collision", "robot_to_movables"): "collision",
    ("Collision", "movable_to_world"): "collision",
    ("Motion", "joint_limit"): "joint_limits",
    ("Motion", "self_collision"): "collision",
    ("KinematicConstraint", "pos_err"): "ik",
    ("KinematicConstraint", "rot_err"): "ik",
    ("BaseDomain", "outside_m"): "reachability",
}


def optimizer_failure_feedback(candidates, metrics, *, max_steps, target):
    """Keep approximate violations global, preserving their measured categories."""
    counts = Counter()
    for particle in candidates:
        # Inspect actual per-particle conjunction failures. Minima from different
        # particles cannot be assembled into an imaginary satisfying particle.
        categories = {COST_CATEGORIES.get((group, name), "other")
                      for group, terms in particle["constraint_diagnostics"].items()
                      for name, item in terms.items() if not item["satisfied"]}
        counts.update(categories)
    timed_out = bool(metrics["timed_out"])
    return ProgramFailure(
        "", tuple(name for name, _ in counts.most_common()) or ("no_candidates",), (target,),
        program_unsat=False,
        search_budget_exhausted=timed_out or metrics["num_opt_steps"] >= max_steps,
        failure_source="optimizer_timeout" if timed_out else "approximate_tolerance_unmet",
        attribution_scope="global",
        budget_scope=("optimizer_wall_time" if timed_out else "optimizer_steps"
                      if metrics["num_opt_steps"] >= max_steps else "none"),
    ).as_feedback()


def exact_failure_feedback(program, failures, *, checks, candidate_count, max_postchecks,
                           target, stop_reason=None, count_exhausted=None):
    """Attribute only explicit exact-check evidence, never a missing index to zero."""
    if count_exhausted is None:
        count_exhausted = len(checks) >= max_postchecks and len(checks) < candidate_count
    elif type(count_exhausted) is not bool:
        raise ValueError("count_exhausted must be a boolean")
    steps = {item.details.get("program_step") for item in failures}
    index = next(iter(steps)) if len(steps) == 1 else None
    attributed = type(index) is int and 0 <= index < len(program.steps)
    excluded_only = bool(failures) and all(item.constraint == "excluded_assignment" for item in failures)
    return ProgramFailure.from_constraints(
        failures, skill=program.steps[index].skill if attributed else "", target=target,
        program_unsat=False,
        search_budget_exhausted=count_exhausted,
        failure_source="candidate_exclusion" if excluded_only else "exact_postcheck_rejected",
        attribution_scope="step" if attributed else "global",
        program_step=index if attributed else None,
        budget_scope="exact_postchecks" if count_exhausted else "none",
    ).as_feedback()


def postcheck_budget_feedback(stop_reason, target):
    """Classify soft admission stops as budget evidence, never geometric UNSAT."""
    if stop_reason not in {"postcheck_wall_time", "adaptive_native_guard"}:
        return None
    return ProgramFailure(
        "", ("no_candidates",), (target,), search_budget_exhausted=True,
        failure_source="geometry_wall_time", attribution_scope="global",
        budget_scope="exact_postcheck_wall_time").as_feedback()
