"""Configurable planning and safety contact policies."""

import dataclasses
import math
from collections.abc import Mapping
from collections.abc import Iterable

from src.online_manipulation.observations import Pose


def _canonical_pair(body_a: str, body_b: str) -> tuple[str, str]:
    """Return a stable unordered qualified-body pair."""
    if not body_a or not body_b:
        raise ValueError("Contact body names must be nonempty")
    return tuple(sorted((body_a, body_b)))


@dataclasses.dataclass(frozen=True)
class CarriedBody:
    """Planning-only rigid relation between a body and a robot frame."""

    body_name: str
    carrier_frame_name: str
    body_pose_world: Pose
    carrier_pose_world: Pose

    def __post_init__(self) -> None:
        """Validate the qualified body and carrier frame names."""
        if not self.body_name or not self.carrier_frame_name:
            raise ValueError("Carried-body names must be nonempty")


@dataclasses.dataclass(frozen=True)
class PairContactPolicy:
    """Allow only explicitly listed unordered body pairs."""

    name: str
    allowed_pairs: frozenset[tuple[str, str]] = frozenset()
    monitored_bodies: frozenset[str] = frozenset()
    carried_bodies: tuple[CarriedBody, ...] = ()
    maximum_allowed_penetration_m: float = 0.0

    def __post_init__(self) -> None:
        """Canonicalize all configured pairs."""
        if not self.name:
            raise ValueError("Contact policy name must be nonempty")
        canonical = frozenset(
            _canonical_pair(body_a, body_b)
            for body_a, body_b in self.allowed_pairs
        )
        carried = tuple(self.carried_bodies)
        if len({body.body_name for body in carried}) != len(carried):
            raise ValueError("A body may have only one carrier")
        monitored = frozenset(self.monitored_bodies) | frozenset(
            body.body_name for body in carried
        )
        if any(not body for body in monitored):
            raise ValueError("Monitored contact body names must be nonempty")
        if (
            not math.isfinite(self.maximum_allowed_penetration_m)
            or self.maximum_allowed_penetration_m < 0.0
        ):
            raise ValueError(
                "maximum_allowed_penetration_m must be finite and nonnegative"
            )
        object.__setattr__(self, "allowed_pairs", canonical)
        object.__setattr__(self, "monitored_bodies", monitored)
        object.__setattr__(self, "carried_bodies", carried)

    def permits(self, body_a: str, body_b: str) -> bool:
        """Return whether this policy explicitly allows the body pair."""
        return _canonical_pair(body_a, body_b) in self.allowed_pairs

    @classmethod
    def from_pairs(
        cls,
        name: str,
        pairs: Iterable[tuple[str, str]],
        *,
        monitored_bodies: Iterable[str] = (),
        carried_bodies: Iterable[CarriedBody] = (),
        maximum_allowed_penetration_m: float = 0.0,
    ) -> "PairContactPolicy":
        """Construct a policy from any finite iterable of body pairs."""
        return cls(
            name=name,
            allowed_pairs=frozenset(pairs),
            monitored_bodies=frozenset(monitored_bodies),
            carried_bodies=tuple(carried_bodies),
            maximum_allowed_penetration_m=maximum_allowed_penetration_m,
        )


FREE_MOTION_CONTACT_POLICY = PairContactPolicy(name="free_motion")


@dataclasses.dataclass(frozen=True)
class SupportContactPolicy:
    """Compose robot-ground support with a task without relaxing task contacts.

    Support thresholds are pair-specific. A wheel's compressible floor contact
    must never increase the allowed penetration of fingers into a target.
    """

    task_policy: object
    support_limits_m: Mapping[tuple[str, str], float]
    geometry_limits_m: Mapping[tuple, float] = dataclasses.field(default_factory=dict)

    @property
    def name(self):
        return f"support+{self.task_policy.name}"

    def permits(self, body_a, body_b):
        return (_canonical_pair(body_a, body_b) in self.support_limits_m
                or self.task_policy.permits(body_a, body_b))

    @property
    def monitored_bodies(self):
        return self.task_policy.monitored_bodies

    @property
    def carried_bodies(self):
        return self.task_policy.carried_bodies

    @property
    def maximum_allowed_penetration_m(self):
        return self.task_policy.maximum_allowed_penetration_m


def permits_contact(policy, body_a, body_b, geometry_a="", geometry_b=""):
    """Test exact support geometries without broadening task body rules."""
    if isinstance(policy, SupportContactPolicy):
        pair = tuple(sorted(((body_a, geometry_a), (body_b, geometry_b))))
        if pair in policy.geometry_limits_m:
            return True
    return policy.permits(body_a, body_b)


def penetration_limit(policy, body_a, body_b, geometry_a="", geometry_b=""):
    """Return a pair-scoped contact bound, preserving ordinary task semantics."""
    if isinstance(policy, SupportContactPolicy):
        geometry_pair = tuple(sorted(((body_a, geometry_a), (body_b, geometry_b))))
        if geometry_pair in policy.geometry_limits_m:
            return policy.geometry_limits_m[geometry_pair]
        pair = _canonical_pair(body_a, body_b)
        if pair in policy.support_limits_m:
            return policy.support_limits_m[pair]
    return policy.maximum_allowed_penetration_m if policy.permits(body_a, body_b) else 0.0
