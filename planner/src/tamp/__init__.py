"""Hierarchical TAMP module with selectable PRoC3S and cuTAMP geometry.

Use ``create_session(...)`` for separate plan/execute calls,
``run(TampRunRequest(...))`` for the supervised planning/execution loop, or
``python -m planner.src.tamp`` for the existing CLI. Lower-level model, solver
and runner protocols remain in their respective modules.
"""

from .api import TampRunRequest, create_session, run
from .session import PlanningSession, PreparedPlan, StalePlanError

__all__ = ['TampRunRequest', 'run', 'create_session',
           'PlanningSession', 'PreparedPlan', 'StalePlanError']
