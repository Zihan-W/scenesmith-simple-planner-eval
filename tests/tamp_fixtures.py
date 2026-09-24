"""Finite candidate domains for exercising the real PRoC3S CCSP in tests."""


class FiniteCandidateDomain:
    """Supply the CCSP domain protocol from a fixture's finite candidates."""

    def sample_candidate(self, skill, state, rng):
        return dict(rng.choice(tuple(self.samples(skill, state))))

    @staticmethod
    def parameter_control_keys(parameter):
        return {
            'base_pose': ('base_x_m', 'base_y_m', 'base_yaw_rad'),
            'grasp_pose': ('grasp_lateral_offset_m',),
            'approach_pose': (),
        }.get(parameter, (parameter,))
