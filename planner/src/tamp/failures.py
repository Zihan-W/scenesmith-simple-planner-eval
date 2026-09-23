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
    "verification", "no_candidates", "other", "domain_restriction",
})


FAILURE_SOURCES = frozenset({
    "unspecified", "sampling_budget_exhausted", "approximate_tolerance_unmet",
    "optimizer_timeout", "exact_postcheck_rejected", "candidate_exclusion", "geometry_wall_time",
    "execution_failure", "geometric_search_failed", "backend_precondition",
})
ATTRIBUTION_SCOPES = frozenset({"global", "skill", "step"})


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
        checks = details or {}
        if reason.startswith("pick_joint_edge_"):
            label = reason.removeprefix("pick_joint_edge_")
            if label == "lift_waypoint":
                label = "lift_waypoints_world"
            edge = checks.get(label, {}).get("joint_edge", {})
        else:
            edge = checks.get("last_action_rejection", checks).get("edge", {})
        if edge.get("joint_limits_valid") is False:
            return "joint_limits"
        return "collision"
    runtime_reason = reason.removeprefix("skill_runtime_failed:")
    if runtime_reason == "planned_hold_timeout":
        return "verification"
    if runtime_reason in {"planned_lost_contact", "planned_bilateral_contact_timeout"}:
        return "grasp_validity"
    if runtime_reason in {
            "planned_staging_timeout", "planned_grasp_timeout",
            "planned_lift_waypoint_timeout"}:
        return "execution"
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
                              tuple(sorted(set(self.involved_objects))), failure_source="execution_failure")


@dataclasses.dataclass(frozen=True)
class ProgramFailure:
    """Only skill, constraint categories, objects and bounded-search status."""

    skill: str
    failed_constraints: tuple[str, ...]
    involved_objects: tuple[str, ...]
    program_unsat: bool = False
    search_budget_exhausted: bool = False
    failure_source: str = "unspecified"
    attribution_scope: str = "skill"
    program_step: int | None = None
    budget_scope: str = "none"

    def __post_init__(self):
        if self.failure_source not in FAILURE_SOURCES:
            raise ValueError("Unknown failure source")
        if self.attribution_scope not in ATTRIBUTION_SCOPES:
            raise ValueError("Unknown failure attribution scope")
        if self.attribution_scope == "global" and (self.skill or self.program_step is not None):
            raise ValueError("A global failure cannot name a skill or program step")
        if self.attribution_scope == "skill" and not self.skill:
            raise ValueError("Skill attribution requires a nonempty skill")
        if self.attribution_scope == "step" and (
                not self.skill or type(self.program_step) is not int or self.program_step < 0):
            raise ValueError("Step attribution requires a known skill and nonnegative index")
        if self.attribution_scope != "step" and self.program_step is not None:
            raise ValueError("Only step attribution may include a program index")
        if self.budget_scope not in {"none", "outer_assignments", "optimizer_steps",
                                     "optimizer_wall_time", "exact_postchecks",
                                     "geometry_wall_time", "exact_postcheck_wall_time"}:
            raise ValueError("Unknown exhausted budget scope")

    def as_feedback(self) -> dict[str, Any]:
        return {
            "skill": self.skill,
            "failed_constraints": list(self.failed_constraints),
            "involved_objects": list(self.involved_objects),
            "program_unsat": self.program_unsat,
            "search_budget_exhausted": self.search_budget_exhausted,
            "failure_source": self.failure_source,
            "attribution_scope": self.attribution_scope,
            "program_step": self.program_step,
            "budget_scope": self.budget_scope,
        }

    @classmethod
    def from_feedback(cls, item: Mapping[str, Any]) -> ProgramFailure:
        """Revalidate existing dictionary feedback at the receiving boundary."""
        reasons = item.get("failed_constraints", tuple(item.get("constraint_counts", {})))
        return cls(
            str(item.get("skill", "")),
            tuple(dict.fromkeys(constraint_category(str(reason)) for reason in reasons)),
            tuple(sorted(set(item.get("involved_objects", ())))),
            bool(item.get("program_unsat", False)),
            bool(item.get("search_budget_exhausted", False)),
            item.get("failure_source", "unspecified"),
            item.get("attribution_scope", "skill" if item.get("skill") else "global"),
            item.get("program_step"), item.get("budget_scope", "none"),
        )

    @classmethod
    def from_constraints(cls, constraints, *, skill: str, target: str | None = None,
                         search_budget_exhausted: bool = False, program_unsat: bool = False,
                         failure_source: str = "unspecified", attribution_scope: str = "skill",
                         program_step: int | None = None,
                         budget_scope: str = "none") -> ProgramFailure:
        counts = Counter(constraint_category(item.constraint, item.details) for item in constraints)
        objects = {name for item in constraints for name in item.involved_objects}
        if target:
            objects.add(target)
        return cls(skill, tuple(name for name, _ in counts.most_common()) or ("no_candidates",),
                   tuple(sorted(objects)), program_unsat, search_budget_exhausted,
                   failure_source, attribution_scope, program_step, budget_scope)

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
