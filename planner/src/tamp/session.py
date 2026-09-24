"""Separate planning and execution without duplicating the TAMP recovery loop."""

from __future__ import annotations

import copy
import dataclasses
import hashlib
import json
from pathlib import Path
import time


class StalePlanError(RuntimeError):
    """The state changed outside this session; no prepared action was executed."""


@dataclasses.dataclass(frozen=True)
class PreparedPlan:
    """Immutable inspection record and single-use handle owned by one session.

    Only next_action is authorized by this handle; later actions in the solved
    plan must be replanned after a fresh observation. JSON is an audit artifact,
    not an executable plan that can be reloaded into another process/session.
    """

    sequence: int
    snapshot_token: str
    deadline_monotonic_s: float
    document_json: str

    def as_dict(self):
        """Return an independent JSON-compatible copy for inspection."""
        return json.loads(self.document_json)

    def write(self, path):
        """Write the inspection artifact, refusing to overwrite another file."""
        path = Path(path)
        with path.open('x', encoding='utf-8') as stream:
            stream.write(self.document_json + '\n')
        return path


class PlanningSession:
    """Single-owner plan/execute state machine for an already configured runner.

    No background thread or simulation stepping occurs in plan(). execute()
    performs exactly one skill attempt. plan() consumes its measured outcome
    and decides whether to finish, retry, or solve again using the original
    recovery logic. Closing or an error invalidates all outstanding handles.

    This in-process API has cooperative deadlines. Use the supervised run/CLI
    for a process-level hard deadline, or supervise the embedding application.
    """

    def __init__(self, runner, flow, state_token):
        self._runner = runner
        self._flow = flow
        self._state_token = state_token
        self._expected_state = copy.deepcopy(state_token())
        self._pending = None
        self._decision = None
        self._outcome = None
        self._started = False
        self._closed = False
        self._sequence = 0
        self._result = None

    @property
    def result(self):
        """Final OnlineResult, or None until plan() reaches a terminal result."""
        return self._result

    def _require_state(self):
        if self._state_token() != self._expected_state:
            self.close()
            raise StalePlanError('State changed outside the session; create a fresh session')

    def plan(self) -> PreparedPlan | None:
        """Compute the next plan without execution; return None when finished."""
        if self._result is not None:
            return None
        if self._closed:
            raise RuntimeError('Planning session is closed')
        try:
            self._require_state()
            if self._pending is not None:
                return self._pending
            if not self._started:
                self._started = True
                decision = next(self._flow)
            else:
                decision = self._flow.send(self._outcome)
            self._outcome = None
            self._require_state()
            self._sequence += 1
            # Retain an owned action; the inspection document cannot edit it.
            self._decision = copy.deepcopy(decision)
            document = {
                'schema': 'scenesmith.tamp.prepared_step.v1',
                'sequence': self._sequence,
                'snapshot_token': hashlib.sha256(repr(self._expected_state).encode()).hexdigest(),
                'observation_id': decision.world.observation_id,
                'deadline_monotonic_s': decision.deadline,
                'program': dataclasses.asdict(decision.program),
                'plan': dataclasses.asdict(decision.plan),
                'next_action': dataclasses.asdict(decision.action),
                'execution_scope': 'one_skill_attempt_then_observe_and_replan',
            }
            self._pending = PreparedPlan(self._sequence, document['snapshot_token'],
                decision.deadline, json.dumps(document, ensure_ascii=False, indent=2))
            return self._pending
        except StopIteration as finished:
            self._result = finished.value
            self.close()
            return None
        except BaseException:
            self.close()
            raise

    def execute(self, prepared: PreparedPlan):
        """Execute this session's unmodified pending handle once, then observe."""
        if self._closed or self._pending is None or prepared is not self._pending:
            raise ValueError('Expected the unconsumed PreparedPlan from this session')
        try:
            self._require_state()
            if time.perf_counter() >= self._decision.deadline:
                from .geometry import GeometryDeadlineExceeded
                error = GeometryDeadlineExceeded('Prepared plan expired before execution')
                try:
                    self._flow.throw(error)
                except StopIteration as finished:
                    self._result = finished.value
                raise error
            self._pending = None
            outcome = self._runner.executor.execute(self._decision.action)
            self._outcome = outcome
            self._expected_state = copy.deepcopy(self._state_token())
            self._decision = None
            # Inspection must not let a caller rewrite measured recovery evidence.
            from .snapshot import _owned_copy
            return _owned_copy(outcome)
        except BaseException:
            self.close()
            raise

    def run(self):
        """Automatically drive the remaining session to its measured result."""
        from .geometry import GeometryDeadlineExceeded

        try:
            while (prepared := self.plan()) is not None:
                self.execute(prepared)
        except GeometryDeadlineExceeded:
            if self.result is None:
                raise
        return self.result

    def close(self):
        """Cancel pending planning; never issue an action during cleanup."""
        self._flow.close()
        self._pending = None
        self._decision = None
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
