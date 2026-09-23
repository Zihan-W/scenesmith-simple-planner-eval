"""Bound one TAMP generation stage, including retries, by its parent deadline."""
from contextlib import contextmanager
import time


@contextmanager
def model_stage_budget(client, settings):
    previous = getattr(client, "deadline_monotonic_s", None)
    previous_timeout = getattr(client, "timeout_s", None)
    deadline = time.perf_counter() + settings.stage_timeout_s
    if previous is not None:
        deadline = min(deadline, previous)
    if hasattr(client, "set_deadline"):
        client.set_deadline(deadline)
    if previous_timeout is not None:
        client.timeout_s = min(previous_timeout, settings.request_timeout_s)
    try:
        yield deadline
    finally:
        if hasattr(client, "set_deadline"):
            client.set_deadline(previous)
        if previous_timeout is not None:
            client.timeout_s = previous_timeout
