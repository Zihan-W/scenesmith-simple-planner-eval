"""Static pose navigation independent of the simulator's control loop.

Conservative circular full-robot footprint permits in-place turns, forward
and reverse line segments. Narrow passages can be rejected conservatively.
Navigator never calls env.step or mutates a Drake context.
"""

import dataclasses
import heapq
import math
from collections.abc import Mapping

import numpy as np

from src.online_manipulation.actions import BaseVelocityAction
from src.online_manipulation.observations import Pose
from src.online_manipulation.adapters.description import drake_pose, public_pose


def wrap_angle(value):
    """Return the shortest signed angular displacement."""
    return math.atan2(math.sin(value), math.cos(value))


@dataclasses.dataclass(frozen=True)
class NavigationGoal:
    """Pose of the navigation reference, expressed in a named current frame."""

    pose: Pose
    frame_id: str = "world"


@dataclasses.dataclass(frozen=True)
class NavigationConfig:
    """Default acceptance gates from D07/D08, plus explicit tracking limits."""

    position_tolerance_m: float = 0.03
    yaw_tolerance_rad: float = math.radians(3)
    stopped_velocity_m_s: float = 0.01
    stopped_yaw_rate_rad_s: float = 0.02
    settle_time_s: float = 0.5
    maximum_velocity_m_s: float = 0.12
    maximum_yaw_rate_rad_s: float = 0.45
    timeout_s: float = 180.0
    stall_timeout_s: float = 8.0
    planar_tolerance: float = 1e-6
    control_owner: str = "navigation"


@dataclasses.dataclass(frozen=True)
class StaticNavigationMap:
    """Immutable obstacle AABBs inflated by a full-robot swept disk radius."""

    obstacles: tuple[tuple[float, float, float, float], ...]
    robot_radius_m: float
    bounds: tuple[float, float, float, float] = (-4.5, -4.5, 4.5, 4.5)
    resolution_m: float = 0.1

    def free(self, xy):
        x, y = xy
        r = self.robot_radius_m
        if not (
            self.bounds[0] + r <= x <= self.bounds[2] - r
            and self.bounds[1] + r <= y <= self.bounds[3] - r
        ):
            return False
        for xmin, ymin, xmax, ymax in self.obstacles:
            nearest = np.clip([x, y], [xmin, ymin], [xmax, ymax])
            if np.linalg.norm(nearest - [x, y]) <= r:
                return False
        return True

    def edge_free(self, start, end):
        count = max(
            2,
            math.ceil(np.linalg.norm(np.asarray(end) - start) / (self.resolution_m / 4))
            + 1,
        )
        return all(
            self.free(np.asarray(start) * (1 - a) + np.asarray(end) * a)
            for a in np.linspace(0, 1, count)
        )

    def plan(self, start, goal):
        """Search a 2D lattice; every edge includes a valid circular turn sweep."""
        start, goal = np.asarray(start), np.asarray(goal)
        if not self.free(start) or not self.free(goal):
            return None
        if self.edge_free(start, goal):
            return [tuple(start), tuple(goal)]
        origin = np.array(self.bounds[:2])

        def cell(point):
            return tuple(np.rint((point - origin) / self.resolution_m).astype(int))

        def point(node):
            return origin + np.array(node) * self.resolution_m

        first, last = cell(start), cell(goal)
        if not self.edge_free(start, point(first)) or not self.edge_free(
            point(last), goal
        ):
            return None
        queue = [(0.0, first)]
        costs, parents = {first: 0.0}, {}
        reached = False
        while queue:
            _, current = heapq.heappop(queue)
            if current == last:
                reached = True
                break
            for dx, dy in (
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ):
                neighbor = (current[0] + dx, current[1] + dy)
                new_cost = costs[current] + math.hypot(dx, dy)
                if new_cost >= costs.get(neighbor, float("inf")) or not self.edge_free(
                    point(current), point(neighbor)
                ):
                    continue
                costs[neighbor], parents[neighbor] = new_cost, current
                priority = new_cost + np.linalg.norm(np.array(last) - neighbor)
                heapq.heappush(queue, (priority, neighbor))
        if not reached:
            return None
        nodes = [last]
        while nodes[-1] != first:
            nodes.append(parents[nodes[-1]])
        path = [tuple(start)] + [tuple(point(n)) for n in nodes[::-1]] + [tuple(goal)]
        simplified = [path[0]]
        index = 0
        while index < len(path) - 1:
            end = len(path) - 1
            while end > index + 1 and not self.edge_free(path[index], path[end]):
                end -= 1
            simplified.append(path[end])
            index = end
        return simplified


class Navigator:
    """Pose tracker with controlled cancellation, measured arrival and ownership."""

    def __init__(self, navigation_map, config=NavigationConfig()):
        self.map, self.config = navigation_map, config
        self.reset()

    def reset(self):
        """Clear every episode goal, frozen transform, timer and tracking state."""
        self.status = "idle"
        self.goal = None
        self.original_goal = None
        self.world_goal = None
        self.accepted_time_s = None
        self.last_progress_time = None
        self.last_progress_xy = None
        self.errors = {}
        self.path = []
        self.index = 0
        self.stable_since = None
        self.owner = False
        self.aligning_final_yaw = False

    def set_goal(
        self, goal, observation, frame_poses: Mapping[str, Pose] | None = None
    ):
        """Freeze T_world_reference(t_accept) * T_reference_goal once."""
        if self.owner:
            raise RuntimeError(
                "Cancel and complete parking before replacing control ownership"
            )
        if goal.frame_id in ("world", "map", "odom"):
            reference = Pose((0, 0, 0), (1, 0, 0, 0))
        elif goal.frame_id == "navigation_frame":
            reference = Pose(**observation.base["pose"])
        else:
            frame_poses = (
                observation.robot.frame_poses_world
                if frame_poses is None
                else frame_poses
            )
            reference = frame_poses[goal.frame_id]
        target = drake_pose(reference) @ drake_pose(goal.pose)
        rpy = target.rotation().ToRollPitchYaw().vector()
        if (
            abs(target.translation()[2]) > self.config.planar_tolerance
            or np.max(np.abs(rpy[:2])) > self.config.planar_tolerance
        ):
            raise ValueError(
                "Transformed navigation goal is not on the navigation plane"
            )
        self.original_goal = goal
        self.world_goal = public_pose(target)
        self.goal = np.r_[target.translation()[:2], rpy[2]]
        self.accepted_time_s = observation.time_s
        self.path = self.map.plan(
            observation.base["pose"]["translation_m"][:2], self.goal[:2]
        )
        self.status = "tracking" if self.path is not None else "no_path"
        self.owner = self.path is not None
        self.index = 1
        self.stable_since = None
        self.last_progress_time = observation.time_s
        self.last_progress_xy = np.array(observation.base["pose"]["translation_m"][:2])
        self.aligning_final_yaw = False

    def cancel(self):
        """Request braking; cancellation is incomplete until measured stop."""
        if self.owner:
            self.status = "cancelling"
            self.stable_since = None

    def release(self):
        """Explicitly hand off base control only after controlled parking."""
        if self.status not in ("arrived", "cancelled"):
            raise RuntimeError("Cannot hand off navigation until stopped")
        self.owner = False

    def _stopped(self, observation):
        return (
            np.linalg.norm(observation.base["linear_velocity_world_m_s"][:2])
            <= self.config.stopped_velocity_m_s
            and abs(observation.base["yaw_rate_rad_s"])
            <= self.config.stopped_yaw_rate_rad_s
        )

    def act(self, observation):
        """Read actual state and return one velocity command; never step the env."""
        zero = BaseVelocityAction(
            0, 0, control_owner=self.config.control_owner if self.owner else None
        )
        if self.status in (
            "idle",
            "no_path",
            "arrived",
            "cancelled",
            "timeout",
            "blocked",
        ):
            return zero
        if self.status == "cancelling":
            if self._stopped(observation):
                if self.stable_since is None:
                    self.stable_since = observation.time_s
                if (
                    observation.time_s - self.stable_since
                    >= self.config.settle_time_s - 1e-9
                ):
                    self.status = "cancelled"
            else:
                self.stable_since = None
            return zero
        if observation.time_s - self.accepted_time_s > self.config.timeout_s:
            self.status = "timeout"
            return zero
        pose = observation.base["pose"]
        xy = np.array(pose["translation_m"][:2])
        yaw = drake_pose(Pose(**pose)).rotation().ToRollPitchYaw().yaw_angle()
        error = np.linalg.norm(xy - self.goal[:2])
        yaw_error = wrap_angle(self.goal[2] - yaw)
        self.errors = {"position_m": float(error), "yaw_rad": float(yaw_error)}
        if (
            error <= self.config.position_tolerance_m
            and abs(yaw_error) <= self.config.yaw_tolerance_rad
            and self._stopped(observation)
        ):
            if self.stable_since is None:
                self.stable_since = observation.time_s
            if (
                observation.time_s - self.stable_since
                >= self.config.settle_time_s - 1e-9
            ):
                self.status = "arrived"
            return zero
        self.stable_since = None
        if observation.base["blocked"]:
            self.status = "blocked"
            return zero
        if (
            np.linalg.norm(xy - self.last_progress_xy) > 0.005
            or abs(observation.base["yaw_rate_rad_s"]) > 0.03
        ):
            self.last_progress_time, self.last_progress_xy = (
                observation.time_s,
                xy.copy(),
            )
        if (
            observation.time_s - self.last_progress_time > self.config.stall_timeout_s
            and error > self.config.position_tolerance_m
        ):
            self.status = "blocked"
            return zero
        if error <= min(0.02, self.config.position_tolerance_m):
            self.aligning_final_yaw = True
        if error > self.config.position_tolerance_m:
            self.aligning_final_yaw = False
        if self.aligning_final_yaw:
            return BaseVelocityAction(
                0,
                float(
                    np.clip(
                        1.5 * yaw_error,
                        -self.config.maximum_yaw_rate_rad_s,
                        self.config.maximum_yaw_rate_rad_s,
                    )
                ),
                control_owner=self.config.control_owner,
            )
        waypoint = np.array(self.path[self.index])
        if np.linalg.norm(xy - waypoint) < 0.03 and self.index < len(self.path) - 1:
            self.index += 1
            waypoint = np.array(self.path[self.index])
        delta = waypoint - xy
        heading = math.atan2(delta[1], delta[0])
        forward_error = wrap_angle(heading - yaw)
        direction = 1 if abs(forward_error) <= math.pi / 2 else -1
        heading_error = wrap_angle(heading + (math.pi if direction < 0 else 0) - yaw)
        omega = float(
            np.clip(
                1.5 * heading_error,
                -self.config.maximum_yaw_rate_rad_s,
                self.config.maximum_yaw_rate_rad_s,
            )
        )
        v = (
            direction
            * min(self.config.maximum_velocity_m_s, 0.8 * np.linalg.norm(delta))
            if abs(heading_error) < 0.15
            else 0.0
        )
        return BaseVelocityAction(v, omega, control_owner=self.config.control_owner)
