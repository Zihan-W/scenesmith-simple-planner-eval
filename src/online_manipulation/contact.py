"""Configurable planning and safety contact policies."""

import dataclasses
from collections.abc import Iterable


def _canonical_pair(body_a: str, body_b: str) -> tuple[str, str]:
    """Return a stable unordered qualified-body pair."""
    if not body_a or not body_b:
        raise ValueError("Contact body names must be nonempty")
    return tuple(sorted((body_a, body_b)))


@dataclasses.dataclass(frozen=True)
class PairContactPolicy:
    """Allow only explicitly listed unordered body pairs."""

    name: str
    allowed_pairs: frozenset[tuple[str, str]] = frozenset()

    def __post_init__(self) -> None:
        """Canonicalize all configured pairs."""
        if not self.name:
            raise ValueError("Contact policy name must be nonempty")
        canonical = frozenset(
            _canonical_pair(body_a, body_b)
            for body_a, body_b in self.allowed_pairs
        )
        object.__setattr__(self, "allowed_pairs", canonical)

    def permits(self, body_a: str, body_b: str) -> bool:
        """Return whether this policy explicitly allows the body pair."""
        return _canonical_pair(body_a, body_b) in self.allowed_pairs

    @classmethod
    def from_pairs(
        cls,
        name: str,
        pairs: Iterable[tuple[str, str]],
    ) -> "PairContactPolicy":
        """Construct a policy from any finite iterable of body pairs."""
        return cls(name=name, allowed_pairs=frozenset(pairs))


FREE_MOTION_CONTACT_POLICY = PairContactPolicy(name="free_motion")
