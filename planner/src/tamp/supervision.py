"""External deadline for simulation CLI workers, including native solver calls."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from .selection_evidence import collect_run_selection_evidence


def supervise_simulation(command, *, output, started_at, max_wall_time_s):
    """Terminate this worker process group at deadline and retain a final result.

    This is a process-level work limit, not an operating-system real-time
    guarantee. Interrupted HTML export may be absent; no success is invented.
    """
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Output directory is not empty: {output}")
    deadline = started_at + max_wall_time_s
    process = subprocess.Popen(command, start_new_session=True)
    timed_out = False
    try:
        returncode = process.wait(timeout=max(0.0, deadline - time.perf_counter()))
    except subprocess.TimeoutExpired:
        timed_out = True
        os.killpg(process.pid, signal.SIGKILL)
        returncode = process.wait()
    except BaseException:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        raise
    elapsed = time.perf_counter() - started_at
    output.mkdir(parents=True, exist_ok=True)
    evidence = {"deadline_enforcement": "external_process_group_watchdog",
                "timed_out": timed_out, "worker_returncode": returncode,
                "max_wall_time_s": max_wall_time_s, "wall_time_s": elapsed}
    (output / "supervision.json").write_text(json.dumps(evidence, indent=2) + "\n")
    result_path = output / "result.json"
    if timed_out or not result_path.exists():
        if result_path.exists():
            result_path.rename(output / "worker_result_before_termination.json")
        result = {"planner": "tamp", "success": False,
                  "reason": "wall_time_budget_exhausted" if timed_out else "worker_process_error",
                  "metrics": {"wall_time_s": elapsed, "wall_time_budget_s": max_wall_time_s},
                  "supervision": evidence, "partial_trace": "tamp_trace.jsonl",
                  "recording_may_be_incomplete": True,
                  **collect_run_selection_evidence(output)}
        temporary = output / "result.json.tmp"
        temporary.write_text(json.dumps(result, indent=2) + "\n")
        temporary.replace(result_path)
    return 2 if timed_out else returncode
