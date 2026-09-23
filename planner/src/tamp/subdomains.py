"""Normalized subdomains inside registered envelopes; never a feasibility proof.

Base forward/lateral axes follow the existing fixed heading. Fractions are
relative to the min/max projections of the registered base candidates, not
world meters. Grasp lateral fractions use the registered target-frame range.
"""
import math
import numpy as np

AXES = {"scene_base_pose": {"forward", "lateral"},
        "calibrated_grasp_pose": {"lateral"}, "calibrated_approach_pose": set()}


def validate_subdomain(sampler, value):
    if not isinstance(value, dict) or set(value) - AXES.get(sampler, set()):
        raise ValueError("Unsupported normalized subdomain axes")
    result = {}
    for axis, interval in value.items():
        if (not isinstance(interval, (list, tuple)) or len(interval) != 2
                or any(type(x) not in (int, float) or not math.isfinite(x) for x in interval)
                or not 0 <= interval[0] < interval[1] <= 1):
            raise ValueError("Subdomain must have 0 <= low < high <= 1; envelope expansion is forbidden")
        result[axis] = tuple(float(x) for x in interval)
    return result


def step_bounds(program, step, role):
    variable = step.continuous_variables.get(role)
    return dict(getattr(program, "parameter_subdomains", {}).get(variable, {}))


def base_projection(candidates):
    points = np.asarray(candidates, dtype=float)
    yaw = points[0, 2]
    if not np.allclose(points[:, 2], yaw, atol=1e-12, rtol=0):
        raise ValueError("Base subdomain requires the registered fixed heading")
    axes = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    projected = points[:, :2] @ axes
    return axes, projected.min(axis=0), projected.max(axis=0)


def restrict_base_candidates(candidates, bounds):
    if not bounds:
        return candidates
    axes, low, high = base_projection(candidates)
    start = np.array([bounds.get(name, (0., 1.))[0] for name in ("forward", "lateral")])
    span = np.array([bounds.get(name, (0., 1.))[1] - bounds.get(name, (0., 1.))[0]
                     for name in ("forward", "lateral")])
    # Intersect the actual registered hull with the subdomain rectangle.
    # A contraction of its bounding box could escape a nonrectangular hull.
    from scipy.spatial import ConvexHull
    points = np.asarray(candidates, dtype=float)
    projected = points[:, :2] @ axes
    polygon = list(projected[ConvexHull(projected).vertices])
    for axis in range(2):
        for boundary, sign in ((low[axis] + start[axis]*(high[axis]-low[axis]), -1),
                               (low[axis] + (start[axis]+span[axis])*(high[axis]-low[axis]), 1)):
            clipped = []
            for first, second in zip(polygon, polygon[1:] + polygon[:1]):
                d0, d1 = sign*(first[axis]-boundary), sign*(second[axis]-boundary)
                if d0 <= 0:
                    clipped.append(first)
                if (d0 <= 0) != (d1 <= 0):
                    point = first + d0/(d0-d1)*(second-first)
                    point[axis] = boundary
                    clipped.append(point)
            polygon = clipped
    if len(polygon) < 3:
        raise ValueError("Declared base subdomain has no area inside registered hull")
    xy = np.asarray(polygon) @ axes.T
    return tuple((float(p[0]), float(p[1]), float(points[0, 2])) for p in xy)


def validate_program_subdomains(program, registry):
    """Validate again at solver boundaries, including programs built in Python."""
    declared = getattr(program, "parameter_subdomains", {})
    expected = {variable: registry.domain_samplers[role] for step in program.steps
                for role, variable in step.continuous_variables.items()}
    if set(declared) - set(expected):
        raise ValueError("Subdomain references an unbound variable")
    for variable, bounds in declared.items():
        validate_subdomain(expected[variable], bounds)


def grasp_range(domain, bounds):
    low, high = domain.program_schema["PickLift"]["grasp_lateral_offset_m_range"]
    a, b = bounds.get("lateral", (0., 1.))
    return low + a * (high - low), low + b * (high - low)


def restrict_sample(domain, program, step, candidate):
    result = dict(candidate)
    if step.skill == "NavigateToPick":
        bounds = step_bounds(program, step, "base_pose")
        if bounds:
            axes, low, high = base_projection(domain.candidates)
            p = np.array([result["base_x_m"], result["base_y_m"]]) @ axes
            for i, name in enumerate(("forward", "lateral")):
                a, b = bounds.get(name, (0., 1.))
                p[i] = low[i] + a * (high[i]-low[i]) + (p[i]-low[i]) * (b-a)
            result["base_x_m"], result["base_y_m"] = map(float, p @ axes.T)
    elif step.skill == "PickLift":
        bounds = step_bounds(program, step, "grasp_pose")
        if bounds:
            low, high = domain.program_schema["PickLift"]["grasp_lateral_offset_m_range"]
            a, b = bounds["lateral"]
            new_low, new_high = grasp_range(domain, bounds)
            fraction = (result["grasp_lateral_offset_m"] - low) / (high - low)
            result["grasp_lateral_offset_m"] = min(new_high, max(new_low, new_low + fraction*(new_high-new_low)))
    return result


def candidate_in_subdomain(domain, program, step, candidate):
    """Independently enforce declared restrictions on a solver's output."""
    if step.skill == "NavigateToPick":
        bounds = step_bounds(program, step, "base_pose")
        if not bounds:
            return True
        axes, low, high = base_projection(domain.candidates)
        p = np.array([candidate["base_x_m"], candidate["base_y_m"]]) @ axes
        for i, name in enumerate(("forward", "lateral")):
            a, b = bounds.get(name, (0., 1.))
            if not low[i] + a*(high[i]-low[i]) <= p[i] <= low[i] + b*(high[i]-low[i]):
                return False
    elif step.skill == "PickLift":
        bounds = step_bounds(program, step, "grasp_pose")
        if bounds:
            low, high = grasp_range(domain, bounds)
            return low <= candidate["grasp_lateral_offset_m"] <= high
    return True
