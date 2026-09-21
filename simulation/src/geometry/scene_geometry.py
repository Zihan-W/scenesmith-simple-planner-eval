"""Resolve explicit proximity selectors consistently for navigation and safety."""

def proximity_records(plant, inspector):
    """Return (geometry ID, qualified body, inspector geometry name) records."""
    records = []
    for geometry in inspector.GetAllGeometryIds():
        if inspector.GetProximityProperties(geometry) is None:
            continue
        body = plant.GetBodyFromFrameId(inspector.GetFrameId(geometry))
        qualified = f"{plant.GetModelInstanceName(body.model_instance())}::{body.name()}"
        records.append((geometry, qualified, inspector.GetName(geometry)))
    return records


def resolve_ground_geometries(plant, inspector, body_names=(), selectors=()):
    """Resolve floors; legacy body selectors require exactly one collision.

    A mixed floor/wall body is deliberately rejected rather than granting
    support contact or removing obstacles for the complete room.
    """
    records = proximity_records(plant, inspector)
    selected = set()
    for body in body_names:
        matches = [r for r in records if r[1] == body]
        if len(matches) != 1:
            raise ValueError(
                f"Ground body {body} has {len(matches)} collision geometries; "
                "use explicit ground_geometries"
            )
        selected.add(matches[0][0])
    for body, name in selectors:
        matches = [r for r in records if r[1] == body and
                   (r[2] == name or r[2].split('::')[-1] == name)]
        if len(matches) != 1:
            raise ValueError(f"Ground geometry {body}/{name}: expected one, got {len(matches)}")
        selected.add(matches[0][0])
    return frozenset(selected)
