"""Small trusted factory loading and episode assembly shared by entry points."""

import importlib

from src.online_manipulation.factory import make_env
from src.online_manipulation.protocols import Policy
from src.online_manipulation.runner import run_episodes


def load_factory(reference):
    """Import explicitly trusted Python code; this is not a plugin sandbox."""
    module, separator, name = reference.partition(":")
    if not separator or not module or not name:
        raise ValueError("Factory reference must be module:function")
    factory = getattr(importlib.import_module(module), name)
    if not callable(factory):
        raise TypeError(f"Factory is not callable: {reference}")
    return factory


def run(config, policy, *, output_root, seeds=(0,), max_steps=100,
        record_html=False, write_final_dmd=False):
    """Build once and reuse the existing reset/step runner without a new loop."""
    if max_steps <= 0 or not seeds:
        raise ValueError("max_steps must be positive and seeds nonempty")
    if not isinstance(policy, Policy):
        raise TypeError("Policy must implement reset(observation, info) and act")
    return run_episodes(env=make_env(config), policy=policy, seeds=seeds,
                        max_steps=max_steps, output_root=output_root,
                        record_html=record_html, write_final_dmd=write_final_dmd)
