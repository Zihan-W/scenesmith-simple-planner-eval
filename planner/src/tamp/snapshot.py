"""Observation-owned planning snapshots and explicit physical-state tokens."""
import copy
import dataclasses
from collections.abc import Mapping
import hashlib
import json
import math


def _owned_copy(value):
    """Copy public observation dataclasses including read-only mapping fields."""
    if dataclasses.is_dataclass(value):
        return dataclasses.replace(value, **{
            field.name: _owned_copy(getattr(value, field.name))
            for field in dataclasses.fields(value) if field.init})
    if isinstance(value, Mapping):
        return {key: _owned_copy(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return tuple(_owned_copy(item) for item in value)
    if isinstance(value, list):
        return [_owned_copy(item) for item in value]
    return copy.deepcopy(value)


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def observation_token(observation):
    return _digest({"time": f"{observation.time_s:.6f}",
                    "base": observation.base["base_link_pose"],
                    "names": list(observation.robot.joint_names),
                    "q": list(observation.robot.q),
                    "objects": {name: item.pose.as_dict()
                                for name, item in observation.objects.items()}})


def world_token(world):
    return _digest({"time": world.observation_id,
                    "base": world.robot["base_link_pose"],
                    "names": list(world.robot["joint_names"]),
                    "q": list(world.robot["q"]),
                    "objects": {name: {key: obj[key] for key in
                                ("translation_m", "quaternion_wxyz")}
                                for name, obj in world.objects.items()}})


class PlanningSnapshot:
    """Own one observation/query pair; never expose the mutable query template.

    Offline callers may omit the query and build one from this observation.
    Online capture must supply an environment query from the same time.
    """
    def __init__(self, observation, query=None):
        if query is not None and not math.isclose(
                query.state_time_s, observation.time_s, abs_tol=1e-9, rel_tol=0):
            raise ValueError("Planning query and observation must share the same timestamp")
        self._observation = _owned_copy(observation)
        self._token = observation_token(self._observation)
        self._query = query.fork() if query is not None else None

    @property
    def token(self):
        return self._token

    @property
    def observation(self):
        return _owned_copy(self._observation)

    def fork_query(self):
        return self._query.fork() if self._query is not None else None

    def require_world(self, world):
        if world_token(world) != self.token:
            raise ValueError("World differs from the planning snapshot token")

    def geometry_state(self, state):
        from simulation.src import Pose
        result = dict(state)
        pose = self._observation.base["base_link_pose"]
        result["base_pose"] = Pose(tuple(pose["translation_m"]), tuple(pose["quaternion_wxyz"]))
        result["snapshot_token"] = self.token
        return result
