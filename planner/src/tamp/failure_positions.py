"""Checked-sample projections in registered normalized sampler coordinates."""
import math
from collections.abc import Mapping
import numpy as np
from .subdomains import AXES, base_projection

SCOPE = 'checked_sample_projections_not_full_assignments_or_region_unsat'
PHYSICAL_CATEGORIES = frozenset({'ik', 'collision', 'corridor', 'reachability',
                                  'joint_limits', 'grasp_validity', 'approach'})


def validate_positions(positions):
    """Accept only bounded numeric positions and categorical failure evidence."""
    if not isinstance(positions, (list, tuple)) or len(positions) > 8:
        raise ValueError('At most eight checked-sample projections may enter a model prompt')
    result = []
    for item in positions:
        if not isinstance(item, Mapping) or set(item) != {
                'variable', 'sampler', 'program_step', 'position', 'failed_constraint', 'state_token', 'object'}:
            raise ValueError('Invalid sampled failure position fields')
        sampler, variable, point = item['sampler'], item['variable'], item['position']
        token = item['state_token']
        if (not isinstance(token, str) or len(token) != 64
                or any(c not in '0123456789abcdef' for c in token)
                or not isinstance(item['object'], str) or not item['object']
                or sampler not in AXES or not AXES[sampler]
                or not isinstance(variable, str) or not variable.isidentifier()
                or type(item['program_step']) is not int or not 0 <= item['program_step'] < 20
                or item['failed_constraint'] not in PHYSICAL_CATEGORIES
                or not isinstance(point, Mapping) or set(point) != AXES[sampler]
                or any(type(value) not in (int, float) or not math.isfinite(value)
                       or not 0 <= value <= 1 for value in point.values())):
            raise ValueError('Invalid sampled failure position values')
        result.append({**item, 'position': {axis: float(value) for axis,value in point.items()}})
    return tuple(result)


def normalized_failure_positions(domain, step, index, parameters, category):
    """Project checked controls; never label a region or an unchecked control bad."""
    if category not in PHYSICAL_CATEGORIES:
        return ()
    if step.skill == 'NavigateToPick':
        role, sampler = 'base_pose', 'scene_base_pose'
        axes, low, high = base_projection(domain.candidates)
        if np.any(high <= low):
            return ()
        xy = np.array([parameters['base_x_m'], parameters['base_y_m']]) @ axes
        values = (xy - low) / (high - low)
        position = dict(zip(('forward','lateral'), map(float, values), strict=True))
    elif step.skill == 'PickLift':
        role, sampler = 'grasp_pose', 'calibrated_grasp_pose'
        low, high = domain.program_schema['PickLift']['grasp_lateral_offset_m_range']
        position = {'lateral': (parameters['grasp_lateral_offset_m'] - low) / (high-low)}
    else:
        return ()
    # Out-of-envelope values are not clamped into a fictitious checked boundary.
    if any(not math.isfinite(value) or not 0 <= value <= 1 for value in position.values()):
        return ()
    return validate_positions([dict(variable=step.continuous_variables[role], sampler=sampler,
        program_step=index, position=position, failed_constraint=category,
        state_token=domain.snapshot.token, object=step.arguments['object'])])
