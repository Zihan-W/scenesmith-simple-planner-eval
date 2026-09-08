"""Read-only SceneSmith DMD dependency validation and derived scene preparation.

This module needs no SceneSmith Python installation or generation service.
Scene assets are copied, never symlinked, into a caller-selected cache root.
Only the supported DMD/SDF/URDF/glTF/OBJ material references are interpreted;
unresolved package names and dependencies outside declared packages fail loudly.
"""

import dataclasses
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


class Rpy(dict):
    """Preserve Drake's explicit deg/rad rotation tag through YAML editing."""


class AngleAxis(dict):
    """Preserve Drake's AngleAxis tag without changing rotation semantics."""


class Loader(yaml.SafeLoader):
    """Safe YAML loader accepting only the additional Drake Rpy tag."""


class Dumper(yaml.SafeDumper):
    """Safe YAML dumper preserving Rpy units."""


Loader.add_constructor("!Rpy", lambda loader, node: Rpy(loader.construct_mapping(node)))
Dumper.add_representer(Rpy, lambda dumper, value: dumper.represent_mapping("!Rpy", value))
Loader.add_constructor("!AngleAxis", lambda loader, node: AngleAxis(loader.construct_mapping(node, deep=True)))
Dumper.add_representer(AngleAxis, lambda dumper, value: dumper.represent_mapping("!AngleAxis", value))


def load_dmd(path):
    """Read a DMD without evaluating Python or unknown YAML constructors."""
    return yaml.load(Path(path).read_text(), Loader=Loader)


def _references(path):
    suffix = path.suffix.lower()
    if suffix in (".yaml", ".yml"):
        document = load_dmd(path)
        for directive in document.get("directives", []):
            for key in ("add_model", "add_directives"):
                if key in directive:
                    yield directive[key]["file"]
    elif suffix in (".sdf", ".urdf"):
        root = ET.parse(path).getroot()
        for node in root.iter():
            if node.tag in ("uri", "filename") and node.text:
                yield node.text.strip()
            if node.tag in ("mesh", "texture") and "filename" in node.attrib:
                yield node.attrib["filename"]
    elif suffix == ".gltf":
        data = json.loads(path.read_text())
        for key in ("buffers", "images"):
            for item in data.get(key, []):
                if "uri" in item and not item["uri"].startswith("data:"):
                    yield item["uri"]
    elif suffix in (".obj", ".mtl"):
        for line in path.read_text().splitlines():
            tokens = line.strip().split(maxsplit=1)
            if len(tokens) == 2 and (tokens[0] == "mtllib" or tokens[0].startswith("map_") or tokens[0] == "bump"):
                yield tokens[1]


def package_roots(package_xmls):
    """Resolve package names, rejecting ambiguous same-name roots."""
    packages = {}
    for xml in package_xmls:
        xml = Path(xml).resolve()
        name = ET.parse(xml).getroot().findtext("name")
        if not name:
            raise ValueError(f"Missing package name: {xml}")
        if name in packages and packages[name] != xml.parent:
            raise ValueError(f"Conflicting package {name}: {packages[name]} and {xml.parent}")
        packages[name] = xml.parent
    return packages


def _resolve(reference, parent, packages):
    if reference.startswith("package://"):
        name, relative = reference[10:].split("/", 1)
        if name not in packages:
            raise ValueError(f"Unknown package {name} referenced by {parent}")
        return (packages[name] / relative).resolve()
    if reference.startswith("file://"):
        return Path(reference[7:]).resolve()
    if "://" in reference:
        raise ValueError(f"Nonlocal asset {reference} in {parent}")
    return (parent.parent / reference).resolve()


def inspect_dependencies(dmd_path, package_xmls):
    """Return the full local runtime closure or all missing dependency errors."""
    packages = package_roots(package_xmls)
    pending = [Path(dmd_path).resolve()]
    seen, errors = set(), []
    while pending:
        path = pending.pop()
        if path in seen:
            continue
        seen.add(path)
        if not path.is_file():
            errors.append(f"Missing dependency: {path}")
            continue
        if not any(path.is_relative_to(root) for root in packages.values()):
            errors.append(f"Dependency outside declared package roots: {path}")
            continue
        for reference in _references(path):
            pending.append(_resolve(reference, path, packages))
    if errors:
        raise ValueError("Scene dependency validation failed:\n" + "\n".join(sorted(errors)))
    return tuple(sorted(seen)), packages


@dataclasses.dataclass(frozen=True)
class PreparedScene:
    """Portable cached scene and explicit metadata, not a live-state snapshot."""

    dmd_path: Path
    package_xmls: tuple[Path, ...]
    metadata_path: Path
    manifest_path: Path
    ground_geometries: tuple[tuple[str, str], ...]


def prepare_scene(*, scene_root, variant, cache_root, overrides=None,
                  additional_package_xmls=(), metadata_path=None, dmd_relative=None):
    """Validate and copy one complete house into a fresh derived cache.

    variant must explicitly select free or furniture_welded. Overrides key
    exact model instance names and may replace file/default_free_body_pose.
    Replacement geometry facts must be supplied explicitly; unspecified facts
    become unknown. Source JSON is retained only as provenance, never live state.
    Existing robots must be removed by a separate explicit scene recipe; this
    loader never guesses whether an arbitrary model is a robot.
    """
    variants = {"free": "house.dmd.yaml", "furniture_welded": "house_furniture_welded.dmd.yaml"}
    if variant not in variants:
        raise ValueError(f"Explicit scene variant required; got {variant}")
    source = Path(scene_root).resolve()
    destination = Path(cache_root).resolve()
    if destination.is_relative_to(source):
        raise ValueError("Cache must not be inside the read-only source scene")
    if destination.exists():
        raise FileExistsError(f"Use a new cache directory: {destination}")
    dmd = source / dmd_relative if dmd_relative else source / "combined_house" / variants[variant]
    if not dmd.resolve().is_relative_to(source):
        raise ValueError("dmd_relative must remain inside scene_root")
    xmls = (source / "package.xml", *map(Path, additional_package_xmls))
    files, packages = inspect_dependencies(dmd, xmls)
    document = load_dmd(dmd)
    overrides = overrides or {}
    models = {d["add_model"]["name"]: d["add_model"] for d in document["directives"] if "add_model" in d}
    unknown = set(overrides) - models.keys()
    if unknown:
        raise ValueError(f"Overrides refer to unknown models: {sorted(unknown)}")
    extra = set()
    for name, changes in overrides.items():
        if set(changes) - {"file", "default_free_body_pose", "geometry_facts"}:
            raise ValueError(f"Unsupported override fields for {name}")
        if "default_free_body_pose" in changes and any(
            d.get("add_weld", {}).get("child", "").startswith(name + "::")
            for d in document["directives"]
        ):
            raise ValueError(f"Cannot override free pose of welded model {name}; choose free variant")
        if "file" in changes:
            replacement = _resolve(changes["file"], dmd, packages)
            # Validate replacement dependencies without loading a simulator.
            pending = [replacement]
            while pending:
                path = pending.pop()
                if path in extra:
                    continue
                if not path.is_file() or not any(path.is_relative_to(r) for r in packages.values()):
                    raise ValueError(f"Invalid replacement dependency: {path}")
                extra.add(path)
                pending.extend(_resolve(r, path, packages) for r in _references(path))
        for key in ("file", "default_free_body_pose"):
            if key in changes:
                models[name][key] = changes[key]
                if key == "default_free_body_pose":
                    for pose in models[name][key].values():
                        if not isinstance(pose["rotation"], (Rpy, AngleAxis)):
                            if not set(pose["rotation"]).issubset({"deg", "rad"}):
                                raise ValueError("Override rotation requires explicit Rpy deg/rad units")
                            pose["rotation"] = Rpy(pose["rotation"])
    paths = set(files) | extra | {Path(p) for p in xmls}
    def cached(path):
        for name, root in packages.items():
            if path.is_relative_to(root):
                return destination / "packages" / name / path.relative_to(root)
        raise ValueError(f"No package owner: {path}")

    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}
    for path in sorted(paths):
        target = cached(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        if path.suffix.lower() in (".sdf", ".urdf", ".yaml", ".yml", ".gltf", ".obj", ".mtl"):
            content = path.read_text()
            # Absolute references become package URIs in XML/DMD and relative
            # paths in glTF/OBJ, so moving the entire cache remains valid.
            import os
            for ref in _references(path):
                resolved = _resolve(ref, path, packages)
                if ref.startswith("file://") or Path(ref).is_absolute():
                    if path.suffix.lower() in (".gltf", ".obj", ".mtl"):
                        new = os.path.relpath(cached(resolved), target.parent)
                    else:
                        relative = cached(resolved).relative_to(destination / "packages")
                        new = f"package://{relative.as_posix()}"
                    content = content.replace(ref, new)
            target.write_text(content)
        else:
            shutil.copyfile(path, target)
    # DMD model references may have been absolute in the original input.
    for model in models.values():
        resolved = _resolve(model["file"], dmd, packages)
        model["file"] = "package://" + cached(resolved).relative_to(destination / "packages").as_posix()
    target_dmd = cached(dmd)
    target_dmd.write_text(yaml.dump(document, Dumper=Dumper, sort_keys=False))
    metadata_source = Path(metadata_path) if metadata_path else source / "combined_house/house_state.json"
    metadata = json.loads(metadata_source.read_text()) if metadata_source.is_file() else None
    mapping, ground = [], []
    for name, model in models.items():
        asset = _resolve(model["file"], target_dmd, {n: destination / "packages" / n for n in packages})
        root = ET.parse(asset).getroot()
        links = root.findall(".//link")
        for link in links:
            for collision in link.findall("collision"):
                if collision.get("name") == "floor_collision":
                    ground.append((f"{name}::{link.get('name')}", "floor_collision"))
        # Physical names are authoritative. Semantic IDs are mapped only when
        # the source room prefix convention and source object ID agree.
        matches = []
        if metadata:
            rooms = metadata.get("rooms", {"": metadata} if "objects" in metadata else {})
            for room_id, room in rooms.items():
                objects = room.get("objects", {})
                if isinstance(objects, dict):
                    for oid, obj in objects.items():
                        physical_name = f"{room_id}_{oid}" if room_id else oid
                        if physical_name == name:
                            matches.append((room_id or None, oid))
                        member_names = obj.get("member_model_names") or obj.get("metadata", {}).get("member_model_names", [])
                        if name in member_names:
                            matches.append((room_id or None, oid))
        change = overrides.get(name, {})
        original_object = None
        if len(matches) == 1:
            room_id, object_id = matches[0]
            original_object = (metadata["rooms"][room_id] if room_id else metadata)["objects"][object_id]
        mapping.append({"model_instance": name, "body_names": [l.get("name") for l in links],
                        "semantic_binding": matches[0] if len(matches) == 1 else None,
                        "semantic_name": None if "file" in change or original_object is None else original_object.get("name"),
                        "semantic_category": original_object.get("object_type") if original_object else None,
                        "static_in_sdf": any(m.findtext("static", "false").lower() == "true" for m in root.findall(".//model")),
                        "initial_pose_in_dmd": model.get("default_free_body_pose"),
                        "fixed_by_directive": any(d.get("add_weld", {}).get("child", "").startswith(name + "::") for d in document["directives"]),
                        "asset": model["file"], "replaced": "file" in change,
                        "joints": [{"name": j.get("name"), "type": j.get("type"),
                                    "parent": j.findtext("parent"), "child": j.findtext("child"),
                                    "limits": {n: j.findtext(f"axis/limit/{n}") for n in ("lower", "upper", "effort", "velocity")}}
                                   for j in root.findall(".//joint")],
                        "body_inertials": {l.get("name"): {"mass_kg": l.findtext("inertial/mass"),
                                                            "pose": l.findtext("inertial/pose"),
                                                            "inertia": {n: l.findtext(f"inertial/inertia/{n}") for n in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}}
                                           for l in links},
                        "geometry_facts": change.get("geometry_facts"),
                        "geometry_facts_status": "explicit_override" if "geometry_facts" in change else "unknown",
                        "state_semantics": "initial_configuration_not_runtime_truth"})
    metadata_out = destination / "scene_metadata.json"
    metadata_out.write_text(json.dumps({"objects": mapping, "source_metadata_is_provenance_only": True,
                                        "variant": variant, "ground_geometries": ground}, indent=2))
    manifest = destination / "manifest.json"
    manifest.write_text(json.dumps({"source_scene": str(source), "source_sha256": hashes,
                                   "variant": variant, "overrides": overrides,
                                   "source_metadata_sha256": hashlib.sha256(metadata_source.read_bytes()).hexdigest() if metadata_source.is_file() else None,
                                   "dependency_count": len(paths)}, indent=2))
    for path, digest in hashes.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f"Source changed during preparation: {path}")
    return PreparedScene(target_dmd, tuple(cached(Path(p)) for p in xmls), metadata_out, manifest, tuple(ground))
