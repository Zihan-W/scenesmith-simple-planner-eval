"""PRoC3S-style full-assignment rejection sampling for a fixed skill program.

Official reference: vtamp/policies/ours/policy.py:44-80. A trial samples every
step before checking any constraint, resets the validation state, and evaluates
the instantiated program sequentially. It does not rank feasible batches or
search/rewrite skeletons. SceneSmith geometric checks replace upstream twin
physics rollout; actual contact dynamics remain the executor's responsibility.
"""

from __future__ import annotations

import copy
import random
from collections import Counter

from planner.src.tamp.failures import ProgramFailure

from planner.src.tamp.geometry import (
    GeometricUnsat, _bind_effect, _constraint_objects, _parameter_value, assignment_identity,
    receive_failure_context,
)
from planner.src.tamp.hierarchy import (
    ConstraintResult, ParameterizedSkillAction, ParameterizedSkillPlan, validate_skill_program,
)
from planner.src.tamp.planner import Subgoal
from planner.src.tamp.proc3s import DOMAIN_SAMPLERS


class PRoC3SProgramUnsat(GeometricUnsat):
    """No satisfying assignment found within budget, not a proof of infeasibility."""

    def __init__(self, constraints, *, samples, program):
        super().__init__(constraints)
        self.samples = samples
        failed_steps = [item.details.get("program_step", 0) for item in constraints]
        failed_index = Counter(failed_steps).most_common(1)[0][0] if failed_steps else 0
        self.program_feedback = ProgramFailure.from_constraints(
            constraints, skill=program.steps[failed_index].skill if program.steps else "",
        ).as_feedback()


class Proc3sCCSPSolver:
    """Sample one whole assignment at a time and accept the first feasible one."""

    def __init__(self, registry, domain, *, max_samples=250, seed=0, trace=None, rng=None):
        if max_samples < 1:
            raise ValueError("CCSP sample budget must be positive")
        self.registry, self.domain = registry, domain
        self.max_samples, self.trace = max_samples, trace
        self.random = rng if rng is not None else random.Random(seed)

    def solve(self, world, program, initial_state, *, failure_context=None,
              excluded_assignments=frozenset()):
        """Return the common parameterized plan or structured program failure."""
        self.failure_context = receive_failure_context(failure_context, self.trace)
        validate_skill_program(program, self.registry, frozenset(world.objects))
        domains = getattr(program, "parameter_domains", None)
        if domains is not None:
            expected = {variable: DOMAIN_SAMPLERS[parameter] for step in program.steps
                        for parameter, variable in step.continuous_variables.items()}
            if dict(domains) != expected:
                raise ValueError("CCSP cannot ignore or substitute program domains")
        if not program.steps:
            return ParameterizedSkillPlan((), {}, ())
        failures = []
        for trial in range(self.max_samples):
            # Sample full candidate controls first. These instantiate calibrated
            # poses; derived IK configurations are solved during validation.
            candidates, skills = [], []
            sampling_state = copy.deepcopy(initial_state)
            shared_controls = {}
            for step in program.steps:
                arguments = dict(step.arguments)
                if "object" in arguments:
                    arguments["target"] = arguments.pop("object")
                skill = Subgoal(step.skill, arguments)
                candidate = dict(self.domain.sample_candidate(skill, sampling_state, self.random))
                # Correlate shared open variables before validation, rather than
                # independently sampling continuous values with zero equality probability.
                for parameter, variable in step.continuous_variables.items():
                    keys = self.domain.parameter_control_keys(parameter)
                    if variable in shared_controls:
                        candidate.update(shared_controls[variable])
                    else:
                        shared_controls[variable] = {key: candidate[key] for key in keys}
                candidates.append(candidate)
                skills.append(skill)
                sampling_state = self.domain.predict(skill, candidate, sampling_state)
            if self.trace is not None:
                self.trace({"event": "ccsp_assignment", "trial": trial + 1,
                            "observation_id": world.observation_id,
                            "open_variable_controls": shared_controls,
                            "instantiated_steps": candidates})
            validation_state = copy.deepcopy(initial_state)
            assignments, actions = {}, []
            rejected = False
            for index, (step, skill, candidate) in enumerate(zip(program.steps, skills, candidates, strict=True)):
                feasible, reason, details = self.domain.check(skill, candidate, validation_state)
                details = {**details, "program_step": index}
                values = {variable: _parameter_value(parameter, candidate, details)
                          for parameter, variable in step.continuous_variables.items()}
                candidate_identity = assignment_identity({
                    "skill": step.skill, "arguments": dict(step.arguments), "parameters": candidate})
                if any(variable in assignments and assignments[variable] != value
                       for variable, value in values.items()):
                    feasible, reason = False, "shared_variable_conflict"
                excluded = (candidate_identity in excluded_assignments
                            or assignment_identity(values) in excluded_assignments)
                if self.trace is not None:
                    self.trace({"event": "ccsp_constraint_check", "trial": trial + 1,
                                "step": index, "skill": step.skill, "feasible": feasible,
                                "constraint": reason, "details": details, "excluded": excluded})
                if not feasible or excluded:
                    failures.append(ConstraintResult(
                        False, "excluded_assignment" if excluded else reason,
                        next(iter(values), ""), reason,
                        _constraint_objects(details, step.arguments.get("object")), details))
                    rejected = True
                    break
                spec = self.registry[step.skill]
                actions.append(ParameterizedSkillAction(
                    step.skill, dict(step.arguments),
                    {**candidate, **values, "checks": details, "candidate_identity": candidate_identity},
                    tuple(_bind_effect(effect, step.arguments) for effect in spec.add_effects),
                    spec.supports_geometric_conditioning, dict(step.continuous_variables)))
                assignments.update(values)
                validation_state = self.domain.predict(skill, candidate, validation_state)
            if not rejected:
                if self.trace is not None:
                    self.trace({"event": "ccsp_solved", "samples": trial + 1,
                                "assignments": assignments})
                return ParameterizedSkillPlan(tuple(actions), assignments, tuple(failures))
        raise PRoC3SProgramUnsat(tuple(failures), samples=self.max_samples, program=program)
