"""Opt-in wall-time admission and first-pass policy for exact postchecks."""
import math


class AdaptivePostcheckBudget:
    """Keep a soft execution reserve under the existing hard run watchdog.

    Native checks cannot be preempted here. The guard is admission headroom,
    not a promise that a native call will return within that many seconds.
    """

    @staticmethod
    def validate(settings):
        """Require all policy values explicitly; no calibration is implied."""
        keys = {"execution_reserve_s", "first_pass_remaining_s", "native_call_guard_s"}
        if not isinstance(settings, dict) or set(settings) != keys:
            raise ValueError(f"adaptive_postcheck requires exactly {sorted(keys)}")
        for key, value in settings.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"adaptive_postcheck {key} must be finite and positive")

    def __init__(self, settings, run_deadline, postcheck_deadline):
        self.validate(settings)
        if run_deadline is None or not math.isfinite(run_deadline):
            raise ValueError("adaptive_postcheck requires a finite enclosing run deadline")
        self.settings = dict(settings)
        self.run_deadline = run_deadline
        self.deadline = min(run_deadline - settings["execution_reserve_s"],
                            postcheck_deadline if postcheck_deadline is not None else math.inf)
        self.mode = "quality_window"
        self.transition = None
        self.max_overrun_s = 0.0

    def before_candidate(self, now, count, has_feasible):
        """Switch once; return a stop reason before admitting more work."""
        remaining = self.deadline - now
        guard = self.settings["native_call_guard_s"]
        if self.mode == "quality_window" and remaining <= self.settings["first_pass_remaining_s"] + guard:
            self.mode = "first_pass"
            self.transition = {"after_checks": count, "remaining_postcheck_s": remaining,
                               "remaining_run_s": self.run_deadline - now,
                               "had_feasible_candidate": has_feasible}
        if self.mode == "first_pass" and has_feasible:
            return "adaptive_first_pass_found"
        if remaining <= guard:
            return "adaptive_native_guard"
        return None

    def after_candidate(self, now):
        """Record actual soft-deadline overrun without accepting partial checks."""
        self.max_overrun_s = max(self.max_overrun_s, now - self.deadline)

    def evidence(self, now):
        """Expose selection policy independently of later dynamic success."""
        return {"clock": "time.perf_counter", "settings": self.settings,
                "mode": self.mode, "transition": self.transition,
                "remaining_run_s": self.run_deadline - now,
                "soft_deadline_overrun_s": self.max_overrun_s,
                "execution_reserve_is_soft": True,
                "hard_limit": "existing_external_process_group_watchdog"}
