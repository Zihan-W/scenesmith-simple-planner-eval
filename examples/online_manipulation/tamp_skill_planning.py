"""Skill program generation interface and repository STRIPS baseline.

The deterministic planner is not PRoC3S. Geometry backends receive its fixed
skeleton; they cannot generate or revise that skeleton.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Protocol

from examples.online_manipulation.tamp_hierarchy import (
    PredicateGoal, SkillProgram, SkillRegistry, WorldState, refine_goals,
)


class SkillProgramGenerator(Protocol):
    """Generate an open-parameter program, without solving its geometry."""

    calls: int

    def generate(self, world: WorldState, goals: tuple[PredicateGoal, ...], *,
                 feedback: tuple[Mapping[str, Any], ...] = (),
                 excluded_programs: frozenset[tuple] = frozenset()
                 ) -> SkillProgram: ...


class StripsProgramGenerator:
    """Existing finite STRIPS/BFS search; never calls or impersonates an LLM."""

    calls = 0
    name = "strips"

    def __init__(self, registry: SkillRegistry):
        self.registry = registry

    def generate(self, world: WorldState, goals: tuple[PredicateGoal, ...], *,
                 feedback: tuple[Mapping[str, Any], ...] = (),
                 excluded_programs: frozenset[tuple] = frozenset()
                 ) -> SkillProgram:
        """Search with the existing baseline's bounds and exclusion semantics."""
        return refine_goals(world, goals, self.registry,
                            excluded_programs=excluded_programs)
