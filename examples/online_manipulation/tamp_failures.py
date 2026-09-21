"""Explicit information boundaries between geometry, programs and semantics.

These are repository integration contracts, not additional PRoC3S/cuTAMP
algorithms. Detailed numeric diagnostics stay in LowLevelFailure and traces.
"""

from __future__ import annotations

import dataclasses
from collections import Counter
from collections.abc import Mapping
from typing import Any


CONSTRAINT_CATEGORIES = frozenset({
    "ik", "collision", "corridor", "reachability", "joint_limits", "grasp_validity",
    "approach", "shared_variable", "excluded_assignment", "execution",
    "verification", "no_candidates", "other",
})


def constraint_category(reason: str, details: Mapping[str, Any] | None = None) -> str:
    """Map known runtime/check identifiers to categories; never relay raw text."""
    if reason == "navigation_pick_infeasible":
        failures = (details or {}).get("pick_failures", ())
        categories = Counter(constraint_category(item["reason"], item["details"])
                             for item in failures)
        return categories.most_common(1)[0][0] if categories else "reachability"
    if reason in CONSTRAINT_CATEGORIES:
        return reason
    if reason.startswith("pick_ik_"):
        # Keep the existing reason identifiers and feedback schema. The solver
        # result alone does not establish endpoint collision safety.
        checks = details or {}
        label = reason.removeprefix("pick_ik_")
        if label == "lift_waypoint":
            label = "lift_waypoints_world"
        if checks.get(label, {}).get("reason") == "endpoint_collision_or_clearance":
            return "collision"
        return "ik"
    if reason.startswith("pick_joint_edge_") or reason in {
            "joint_edge_rejected", "skill_runtime_failed:joint_edge_rejected"}:
        return "collision"
    if reason in {"navigation_geometry", "skill_runtime_failed:navigation_blocked"}:
        return "corridor"
    if reason == "base_not_parked":
        return "reachability"
    if reason in {"invalid_grasp_lateral_offset", "pick_target_outside_gripper"}:
        return "grasp_validity"
    if reason.startswith("pick_lift_") or reason in {
            "invalid_approach_height", "invalid_approach_segments"}:
        return "approach"
    if reason == "shared_variable_conflict":
        return "shared_variable"
    if reason == "expected_effect_not_observed":
        return "verification"
    return "other"


@dataclasses.dataclass(frozen=True)
class LowLevelFailure:
    """Full solver/runtime evidence, available locally but not in model prompts."""

    skill: str
    reason: str
    involved_objects: tuple[str, ...] = ()
    details: Mapping[str, Any] = dataclasses.field(default_factory=dict)

    def abstract(self) -> ProgramFailure:
        """Drop numeric details and arbitrary failure text."""
        return ProgramFailure(self.skill, (constraint_category(self.reason, self.details),),
                              tuple(sorted(set(self.involved_objects))))


@dataclasses.dataclass(frozen=True)
class ProgramFailure:
    """Only skill, constraint categories, objects and bounded-search status."""

    skill: str
    failed_constraints: tuple[str, ...]
    involved_objects: tuple[str, ...]
    program_unsat: bool = False
    search_budget_exhausted: bool = False

    def as_feedback(self) -> dict[str, Any]:
        return {
            "skill": self.skill,
            "failed_constraints": list(self.failed_constraints),
            "involved_objects": list(self.involved_objects),
            "program_unsat": self.program_unsat,
            "search_budget_exhausted": self.search_budget_exhausted,
        }

    @classmethod
    def from_feedback(cls, item: Mapping[str, Any]) -> ProgramFailure:
        """Revalidate existing dictionary feedback at the receiving boundary."""
        reasons = item.get("failed_constraints", tuple(item.get("constraint_counts", {})))
        return cls(
            str(item.get("skill", "")),
            tuple(dict.fromkeys(constraint_category(str(reason)) for reason in reasons)),
            tuple(sorted(set(item.get("involved_objects", ())))),
            bool(item.get("program_unsat", item.get("type") == "GEOMETRIC_INFEASIBLE")),
            bool(item.get("search_budget_exhausted", False)),
        )

    @classmethod
    def from_constraints(cls, constraints, *, skill: str, target: str | None = None,
                         search_budget_exhausted: bool = True) -> ProgramFailure:
        counts = Counter(constraint_category(item.constraint, item.details) for item in constraints)
        objects = {name for item in constraints for name in item.involved_objects}
        if target:
            objects.add(target)
        return cls(skill, tuple(name for name, _ in counts.most_common(2)) or ("no_candidates",),
                   tuple(sorted(objects)), True, search_budget_exhausted)

    def abstract(self) -> dict[str, Any]:
        """Do not expose skill names or constraint categories to semantic repair."""
        return {"reason": "physical_infeasibility_or_unverified_effect",
                "involved_objects": list(self.involved_objects)}


@dataclasses.dataclass(frozen=True)
class SemanticFailure:
    """Subgoal-level failure after bounded program/geometry recovery."""

    subgoal: Any
    blocking_failures: tuple[ProgramFailure, ...]

    def as_feedback(self) -> dict[str, Any]:
        return {"subgoal": dataclasses.asdict(self.subgoal),
                "reason": "cannot_refine_or_verify",
                "blocking_failures": [failure.abstract() for failure in self.blocking_failures]}
