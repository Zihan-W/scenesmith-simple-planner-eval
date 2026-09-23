"""Declarations for the currently supported two-skill task domain.

New skills declare samplers and runtime bindings in their SkillSpec. Backend
support is explicit and separate: implementing a skill does not invent its
cuTAMP cost/rollout representation.
"""
from dataclasses import dataclass
from .hierarchy import PredicateGoal, picklift_registry

@dataclass(frozen=True)
class PickLiftDomainDefinition:
    name: str = "mobile_pick_lift"
    default_task: str = "Pick up the red object and hold it."
    cutamp_programs: tuple = (("NavigateToPick",), ("PickLift",),
                              ("NavigateToPick", "PickLift"))

    def registry(self):
        return picklift_registry()

    def goals(self, target):
        return (PredicateGoal("holding", (target,)),)

    def validate_cutamp_program(self, program):
        names = tuple(step.skill for step in program.steps)
        if names not in self.cutamp_programs:
            raise ValueError(f"Unsupported cuTAMP program for {self.name}: {names}")

PICK_LIFT_DOMAIN = PickLiftDomainDefinition()
