"""Adapted from https://gist.github.com/jwnimmer-tri/19479ebdd860d00566b7703e96fc630c"""

import dataclasses
import itertools
import logging

from collections import defaultdict

import numpy as np

from pydrake.all import (
    BodyIndex,
    Box,
    Capsule,
    CollisionFilterDeclaration,
    Context,
    ConvertVolumeToSurfaceMesh,
    Convex,
    Cylinder,
    DiagramBuilder,
    Ellipsoid,
    EventStatus,
    GeometryId,
    GeometryInstance,
    GeometrySet,
    JointIndex,
    LeafSystem,
    Mesh,
    ModelInstanceIndex,
    MultibodyPlant,
    ProximityProperties,
    RigidBody,
    RigidTransform,
    SceneGraph,
    SceneGraphInspector,
    ScopedName,
    Shape,
    Sphere,
    TriangleSurfaceMesh,
    VolumeMesh,
)


@dataclasses.dataclass(kw_only=True)
class ItemLockingMonitorConfig:
    # Simulation time at which item locking can first happen. Intended to allow
    # for settling of objects at the start of a simulation.
    t_start: float = 0.1

    # Scoped or non-scoped names for the geometry that triggers (un)locking.
    # Typically, this would be geometry on the EE, e.g., the fingers.
    unlock_near_geometry: list[str] = dataclasses.field(default_factory=list)

    # Distance threshold to trigger (un)locking. Free bodies must be at least
    # this far away from the `unlock_near_geometry` to be eligible for locking.
    # Must be non-negative.
    distance_threshold: float = 0.05

    # Translational speed threshold to trigger (un)locking. Free bodies must be
    # at least this slow translationally (in units of m/s) to be eligible for
    # locking. (Any rotational velocity is ignored.) Must be non-negative.
    speed_threshold: float = 0.002


from src.geometry_bounds import (
    BoundingBox, _GetShapeOabb, _GetMeshOabb, _OabbToAabb, _MergeAabbs, _CalcAabb,
)


def _get_contact_pairs_body_indices(
    plant: MultibodyPlant, context: Context
) -> list[tuple[BodyIndex, BodyIndex]]:
    contact_results = plant.get_contact_results_output_port().Eval(context)
    contact_pairs = []

    # Point contact pairs.
    for i in range(contact_results.num_point_pair_contacts()):
        contact_result = contact_results.point_pair_contact_info(i)
        index_A = contact_result.bodyA_index()
        index_B = contact_result.bodyB_index()
        contact_pairs.append((index_A, index_B))

    # Hydroelastic contact pairs.
    query_object = plant.get_geometry_query_input_port().Eval(context)
    inspector = query_object.inspector()
    for i in range(contact_results.num_hydroelastic_contacts()):
        contact_result = contact_results.hydroelastic_contact_info(i)
        geo_id_M = contact_result.contact_surface().id_M()
        geo_id_N = contact_result.contact_surface().id_N()
        frame_id_M = inspector.GetFrameId(geo_id_M)
        frame_id_N = inspector.GetFrameId(geo_id_N)
        body_M = plant.GetBodyFromFrameId(frame_id_M)
        body_N = plant.GetBodyFromFrameId(frame_id_N)
        contact_pairs.append((body_M.index(), body_N.index()))

    # TODO(dale.mcconachie) Add deformable contacts.

    return contact_pairs


def get_contact_pairs_scoped_names(
    plant: MultibodyPlant, context: Context
) -> list[tuple[ScopedName, ScopedName]]:
    """Extracts the `ScopedName`s of all point contact and hydroelastic contact
    pairs from the contact results output port of the `plant`.
    """
    contact_pairs_body_indices = _get_contact_pairs_body_indices(plant, context)
    contact_pairs_scoped_names = [
        (plant.get_body(index_A).scoped_name(), plant.get_body(index_B).scoped_name())
        for index_A, index_B in contact_pairs_body_indices
    ]
    return contact_pairs_scoped_names


def _contact_pairs_as_dict(
    contact_pairs: list[tuple[BodyIndex, BodyIndex]],
) -> dict[BodyIndex : set[BodyIndex]]:
    contact_dict = defaultdict(set)
    for index_A, index_B in contact_pairs:
        contact_dict[index_A].add(index_B)
        contact_dict[index_B].add(index_A)
    return contact_dict


# TODO(dale.mcconachie) Hoist this to Drake.
def _GetScopedGeometryByName(
    plant: MultibodyPlant, full_body_name: str
) -> list[GeometryId]:
    scoped_name = ScopedName.Parse(full_body_name)
    namespace = scoped_name.get_namespace()
    element = scoped_name.get_element()
    if len(namespace):
        instance = plant.GetModelInstanceByName(namespace)
        body = plant.GetBodyByName(element, instance)
        return plant.GetCollisionGeometriesForBody(body)
    else:
        body = plant.GetBodyByName(element)
        return plant.GetCollisionGeometriesForBody(body)


@dataclasses.dataclass(kw_only=True)
class _TargetDetail:
    """A helper class for ItemLockingMonitor that collects all relevant
    information about each model subject to locking.
    """

    model_instance_index: ModelInstanceIndex
    model_instance_name: str
    body_index: BodyIndex
    joint_index: JointIndex
    bounding_box_id: GeometryId | None = None
    is_locked: bool = False
    want_locked: bool = False
    colliding_bodies_when_fell_asleep: set[BodyIndex] | None = None


def _FindAllTargets(plant) -> list[_TargetDetail]:
    """Returns the "target" model instances the plant that are subject to
    locking (along with some helpful additional details about each model).
    A target is:
    - a model instance with only a single body, where
    - that body has no outboard joints, and
      - is mobilized by a single 6dof inboard joint, and
      - the joint's inboard parent body is anchored to the world.
    In the future, we might extend the definition (e.g., to model instances
    with more than one body, but still welded as an isolated subgraph).
    """
    # Precompute all world-affixed bodies.
    world_body_indices = set(
        [body.index() for body in plant.GetBodiesWeldedTo(plant.GetBodyByName("world"))]
    )

    # Find all model instances that contain exactly one body.
    candidates: dict[BodyIndex, _TargetDetail] = dict()
    for i in range(plant.num_model_instances()):
        i = ModelInstanceIndex(i)
        body_indices = plant.GetBodyIndices(i)
        if len(body_indices) != 1:
            continue
        body_index = body_indices[0]
        candidates[body_index] = _TargetDetail(
            model_instance_name=plant.GetModelInstanceName(i),
            model_instance_index=i,
            body_index=body_index,
            # We'll set the joint_index later on within this function.
            joint_index=None,
        )

    # Precompute all inboard and outboard joints for those bodies.
    inboards: dict[BodyIndex, list[JointIndex]] = dict()
    outboards: dict[BodyIndex, list[JointIndex]] = dict()
    for j in plant.GetJointIndices():
        joint = plant.get_joint(j)
        outboards.setdefault(joint.parent_body().index(), list()).append(j)
        inboards.setdefault(joint.child_body().index(), list()).append(j)

    # Find bodies that meet our criteria.
    matches = []
    for body_index in candidates.keys():
        # Skip bodies that are inboard of something else.
        if len(outboards.get(body_index, [])) > 0:
            continue
        # TODO: uncomment the above once directives files don't have spurious welds
        # Skip bodies without a single 6dof inboard joint.
        if len(inboards.get(body_index, [])) != 1:
            continue
        joint_index = inboards[body_index][0]
        joint = plant.get_joint(joint_index)
        if joint.num_positions() < 6:
            continue
        # Skip bodies with a non-world-affixed parent.
        if joint.parent_body().index() not in world_body_indices:
            continue
        # TODO: uncomment the above once directives files don't have spurious welds
        candidates[body_index].joint_index = joint_index
        matches.append(body_index)

    return [candidates[body_index] for body_index in matches]


# TODO(dale.mcconachie) Convert this to a LeafSystem with inputs and outputs
# once joints can be locked/unlocked from within the systems framework and
# collision filters are similarly supported.
# https://github.com/RobotLocomotion/drake/issues/20571
# TODO(jeremy.nimmer) We've outgrown the name "item", so should adopt something
# more conventional. The typical term of art here is "sleeping".
class ItemLockingMonitor(LeafSystem):
    """This class is a simulation monitor designed to lock manipulands when
    they are far away from the grippers, unlocking when the grippers approach
    that manipuland. Designed purely for simulation speed improvements rather
    than any physical process.
    """

    def __init__(
        self,
        config: ItemLockingMonitorConfig,
        plant: MultibodyPlant,
        scene_graph: SceneGraph,
    ):
        super().__init__()
        assert plant.is_finalized()
        self._t_start = config.t_start
        self._distance_threshold = config.distance_threshold
        self._speed_threshold = config.speed_threshold
        self._unlock_near_ids: list[GeometryId] = list(
            itertools.chain(
                *[
                    _GetScopedGeometryByName(plant, name)
                    for name in config.unlock_near_geometry
                ]
            )
        )
        self._plant = plant
        self._scene_graph = scene_graph
        self._targets = _FindAllTargets(plant)
        self._plant_state = None
        self._geometry_filter_id = None

        # Add proximity bounding box helper geometry for each lockable target.
        inspector = scene_graph.model_inspector()
        for detail in self._targets:
            # Calculate the AABB of all the body's proximity geometry (in body
            # frame B).
            body = plant.get_body(detail.body_index)
            aabb_B = _CalcAabb(inspector, body)

            # Add the bounding box as a proximity shape; it must be a proximity
            # shape to be able to call ComputeSignedDistancePairClosestPoints.
            # (Registering a geometry directly on the scene graph instead of
            # laundering it through the plant would be a problem if the added
            # geometry were to collide with anything; however, we will be
            # filtering out all collisions involving these added geometries,
            # thus there will never be any collisions to be resolved.)
            detail.bounding_box_id = scene_graph.RegisterGeometry(
                source_id=plant.get_source_id(),
                frame_id=plant.GetBodyFrameIdIfExists(detail.body_index),
                geometry=GeometryInstance(
                    X_PG=RigidTransform(aabb_B.center),
                    shape=Box(aabb_B.size),
                    name=body.name() + "_aabb_for_joint_locking",
                ),
            )
            scene_graph.AssignRole(
                source_id=plant.get_source_id(),
                geometry_id=detail.bounding_box_id,
                properties=ProximityProperties(),
            )

        # Filter out all collisions between our added bounding box geometries
        # and everything else (including themselves).
        filter_manager = scene_graph.collision_filter_manager()
        filter_manager.Apply(
            CollisionFilterDeclaration().ExcludeBetween(
                GeometrySet([detail.bounding_box_id for detail in self._targets]),
                GeometrySet(inspector.GetAllGeometryIds()),
            )
        )

    def Monitor(self, root_context: Context) -> EventStatus:
        """Callback for Simulator.set_monitor that computes the desired joint
        locking status for monitored targets, and breaks out of the simulation
        in case any has changed. The desired status is stored as member data;
        users should call the SetItemLockStates() method to update the joint
        locking state.
        """
        if root_context.get_time() < self._t_start:
            return EventStatus.DidNothing()

        plant_context = self._plant.GetMyContextFromRoot(root_context)
        query_object = self._plant.get_geometry_query_input_port().Eval(plant_context)

        # To improve simulation throughput, we early-return to skip the item-locking
        # checking if the MbP state doesn't change.
        new_plant_state = self._plant.GetPositionsAndVelocities(plant_context)
        if np.array_equal(new_plant_state, self._plant_state):
            return EventStatus.DidNothing()
        self._plant_state = new_plant_state

        # Default to locking everything, unlocking only if it is close to a
        # finger. This considers each target independently; we'll consider
        # any target-on-target interaction in a second pass, below.
        velocities_port = self._plant.get_body_spatial_velocities_output_port()
        velocities = velocities_port.Eval(plant_context)
        for detail in self._targets:
            detail.want_locked = True
            # Check the speed limit.
            # TODO(jeremy-nimmer) This only considers translation. There might
            # be cases where rotational speed should also prevent locking even
            # if the target is translationally stationary.
            spatial_velocity = velocities[detail.body_index]
            speed = np.linalg.norm(spatial_velocity.translational())
            if not speed < self._speed_threshold:
                detail.want_locked = False
                continue
            # Check the distance limit.
            for unlock_near_id in self._unlock_near_ids:
                signed_distance_pair = (
                    query_object.ComputeSignedDistancePairClosestPoints(
                        detail.bounding_box_id, unlock_near_id
                    )
                )
                if signed_distance_pair.distance <= self._distance_threshold:
                    detail.want_locked = False
                    # We know at this point that this object should not be
                    # locked, so we don't need to check it against the rest
                    # of the gripper geometries.
                    break

        # Targets that are (conservatively) in contact with each other must
        # either all lock at once, or none of them can lock at all. First,
        # we'll compute the (conservative) collision status for all target
        # pairs (excluding self-collisions).
        num_targets = len(self._targets)
        contacts = np.zeros(shape=(num_targets, num_targets))
        for i in range(num_targets):
            target_i = self._targets[i]
            for j in range(i + 1, num_targets):
                target_j = self._targets[j]
                signed_distance_pair = (
                    query_object.ComputeSignedDistancePairClosestPoints(
                        target_i.bounding_box_id, target_j.bounding_box_id
                    )
                )
                contact = 1 if signed_distance_pair.distance <= 0 else 0
                contacts[i, j] = contact
                contacts[j, i] = contact
        # Second, we'll propagate the "unlocked" attribute across collisions.
        unlocked_to_visit = set(
            [i for i, x in enumerate(self._targets) if not x.want_locked]
        )
        unlocked_visited = set()
        while unlocked_to_visit:
            i = unlocked_to_visit.pop()
            unlocked_visited.add(i)
            target_i = self._targets[i]
            assert not target_i.want_locked
            for j in range(num_targets):
                if contacts[i, j]:
                    target_j = self._targets[j]
                    if target_j.want_locked:
                        assert j not in unlocked_to_visit
                        assert j not in unlocked_visited
                        target_j.want_locked = False
                        unlocked_to_visit.add(j)

        if self._needs_update():
            return EventStatus.ReachedTermination(None, "Joint locking update")
        else:
            return EventStatus.DidNothing()

    def _needs_update(self) -> bool:
        """Returns True iff we need to stop the sim to change the locking."""
        return any([x.is_locked != x.want_locked for x in self._targets])

    def SetItemLockStates(self, root_context: Context) -> None:
        """Updates the context with the lock states computed by the most recent
        call to the Monitor() method.
        """
        if not self._needs_update():
            # It's extremely important not to change the context when locking
            # details are quiescent. Changes are non-trivially expensive.
            return

        # Grab the *prior* list of contact pairs, so that we can memorize it.
        plant_context = self._plant.GetMyContextFromRoot(root_context)
        old_contact_pairs = _contact_pairs_as_dict(
            _get_contact_pairs_body_indices(self._plant, plant_context)
        )

        # Update the MultibodyPlant context.
        plant_context = self._plant.GetMyMutableContextFromRoot(root_context)
        locked_geom_ids = []
        for detail in self._targets:
            if detail.want_locked:
                locked_geom_ids.extend(
                    self._plant.GetCollisionGeometriesForBody(
                        self._plant.get_body(detail.body_index)
                    )
                )
                if not detail.is_locked:
                    # TODO(dale.mcconachie) Consider quieting this.
                    logging.info(f"Locking {detail.model_instance_name}")
                    joint = self._plant.get_joint(detail.joint_index)
                    joint.Lock(plant_context)
                    detail.is_locked = True
                    detail.colliding_bodies_when_fell_asleep = old_contact_pairs[
                        detail.body_index
                    ]
            else:
                if detail.is_locked:
                    # TODO(dale.mcconachie) Consider quieting this.
                    logging.info(f"Unlocking {detail.model_instance_name}")
                    joint = self._plant.get_joint(detail.joint_index)
                    joint.Unlock(plant_context)
                    detail.is_locked = False
                    detail.colliding_bodies_when_fell_asleep = None

        # Update the SceneGraph context.
        query_object = self._plant.get_geometry_query_input_port().Eval(plant_context)
        new_filter = CollisionFilterDeclaration().ExcludeBetween(
            GeometrySet(locked_geom_ids),
            GeometrySet(query_object.inspector().GetAllGeometryIds()),
        )
        filter_manager = self._scene_graph.collision_filter_manager(
            self._scene_graph.GetMyMutableContextFromRoot(root_context)
        )
        if self._geometry_filter_id is not None:
            filter_manager.RemoveDeclaration(self._geometry_filter_id)
        self._geometry_filter_id = filter_manager.ApplyTransient(new_filter)

    def get_colliding_bodies_when_fell_asleep(
        self, body_index: BodyIndex | None = None
    ):
        """This function operates in two modes.

        In the first mode when a `body_index` is passed, this function returns
        the set of body indices that the given body indicated by `body_index`
        was in contact with when it fell asleep. If the body is not asleep or
        it is not managed by this class then `None` is returned.

        In the second mode, a list of sleeping contact pairs is returned in the
        same format as returned by `get_contact_pairs_scoped_names(...)`.
        """
        if body_index is not None:
            for detail in self._targets:
                if detail.body_index == body_index:
                    return detail.colliding_bodies_when_fell_asleep
            else:
                # We don't raise an error to allow users to query for bodies
                # that are not managed by this class.
                return None
        else:
            contact_pairs = []
            for detail in self._targets:
                if detail.colliding_bodies_when_fell_asleep:
                    index_A = detail.body_index
                    for index_B in detail.colliding_bodies_when_fell_asleep:
                        body_A = self._plant.get_body(index_A)
                        body_B = self._plant.get_body(index_B)
                        contact_pairs.append(
                            (body_A.scoped_name(), body_B.scoped_name())
                        )
            return contact_pairs


def ApplyItemLockingMonitorConfig(
    config: ItemLockingMonitorConfig,
    plant: MultibodyPlant,
    builder: DiagramBuilder,
    scene_graph: SceneGraph,
) -> ItemLockingMonitor:
    """Prepares the given plant and scene_graph for free body locking.
    Returns an ItemLockingMonitor that should be added to the simulator.
    @pre The plant is finalized.
    """
    monitor = ItemLockingMonitor(config, plant, scene_graph)
    builder.AddNamedSystem("item_locking_monitor", monitor)
    return monitor
