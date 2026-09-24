"""Public interface for a supervised hierarchical TAMP simulation run.

The existing CLI owns setup, execution, deadlines and artifacts. This interface
provides structured Python inputs and returns its persisted result unchanged.
It executes a closed loop. create_session exposes separate plan/execute calls
for an existing simulation, using the same backend assembly and recovery loop.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import json
from pathlib import Path
from typing import Any


@dataclass(frozen=True, kw_only=True)
class TampRunRequest:
    """Inputs for one hierarchical simulation, with the existing CLI defaults.

    Paths follow the CLI's filesystem conventions. Output must be new or empty.
    Credentials come from ``api_key_env``; no key belongs in this request.
    PRoC3S program generation and CCSP solving are the defaults. ``geometry_backend``
    chooses PRoC3S CCSP or cuTAMP; the latter requires cutamp_config.
    """

    repository_root: Path
    scene_root: Path
    experiment: Path
    task: str
    output_root: Path
    config: Path | None = None
    skill_planner: str = 'proc3s'
    geometry_backend: str = 'proc3s'
    cutamp_config: Path | None = None
    seed: int | None = None
    shift_world_x_m: float = 0.0
    max_wall_time_s: float | None = None
    record_html: bool = False
    model_replay: Path | None = None
    recorded_subgoals: Path | None = None
    base_url: str | None = None
    api_key_env: str = 'OPENAI_API_KEY'

    def __post_init__(self):
        if not isinstance(self.task, str) or not self.task.strip():
            raise ValueError('task must be a nonempty string')
        if self.skill_planner not in ('strips', 'proc3s'):
            raise ValueError('skill_planner must be strips or proc3s')
        if self.geometry_backend not in ('proc3s', 'cutamp'):
            raise ValueError('geometry_backend must be proc3s or cutamp')
        if (self.geometry_backend == 'cutamp') != (self.cutamp_config is not None):
            raise ValueError('cutamp_config is required exactly when geometry_backend is cutamp')
        if self.model_replay is not None and self.recorded_subgoals is not None:
            raise ValueError('model_replay cannot be combined with recorded_subgoals')
        if type(self.record_html) is not bool:
            raise TypeError('record_html must be a bool')
        for name in ('repository_root', 'scene_root', 'experiment', 'output_root'):
            if not isinstance(getattr(self, name), (str, Path)) or not str(getattr(self, name)):
                raise ValueError(f'{name} must be a nonempty path')

    def cli_arguments(self) -> list[str]:
        """Return the equivalent existing CLI arguments, without credentials."""
        arguments = ['--planner', 'tamp', '--tamp-mode', 'hierarchical']
        for field in fields(self):
            value = getattr(self, field.name)
            if value is None or (field.name == 'record_html' and not value):
                continue
            arguments.append('--' + field.name.replace('_', '-'))
            if field.name != 'record_html':
                arguments.append(str(value))
        return arguments


def run(request: TampRunRequest) -> dict[str, Any]:
    """Execute a supervised simulation and return its result.json document.

    Task/search/provider failure and watchdog termination return success=False
    with the existing reason/evidence fields. Caller-side argument/configuration
    errors and nonempty output directories raise errors; worker initialization
    errors use the existing worker_process_error result. The run keeps the CLI's
    external process-group watchdog, model replay and exact geometry checks.
    It may write progress to stdout/stderr, just like the command-line entry.
    """
    if not isinstance(request, TampRunRequest):
        raise TypeError('request must be a TampRunRequest')
    # Keep package import lightweight; Drake/solver imports happen on invocation.
    from .cli import main

    try:
        main(request.cli_arguments())
    except SystemExit as error:
        raise ValueError('Invalid TAMP run arguments; see the CLI diagnostic') from error
    result_path = Path(request.output_root).resolve() / 'result.json'
    return json.loads(result_path.read_text(encoding='utf-8'))


def create_session(**kwargs):
    """Create a split planning/execution session over a caller-owned environment.

    See application.create_session for the explicit inputs. Imports of Drake
    and model implementations remain deferred until this function is called.
    """
    from .application import create_session as prepare

    return prepare(**kwargs)
