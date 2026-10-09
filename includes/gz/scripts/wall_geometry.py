#!/usr/bin/env python3
"""Build a 2D obstacle map by slicing world collision geometry.

Each world is cut by a horizontal plane (default z = 0.5 m in the Gazebo
ENU world frame). Every collision shape that crosses that plane contributes
line segments. A control node can treat the segments as walls and obstacles
and compute distance to the nearest one.

See docs/simulation.md for the JSON schema.
"""

from __future__ import annotations

import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

MIN_HEIGHT_M = 0.25
MIN_SEGMENT_M = 0.05


def parse_pose(text: str | None) -> np.ndarray:
    """Parse an SDF pose string into x, y, z, roll, pitch, yaw."""
    if text is None or not text.strip():
        return np.zeros(6, dtype=float)
    values = [float(part) for part in text.split()]
    if len(values) != 6:
        raise ValueError(f"pose must have 6 numbers, got {text!r}")
    return np.array(values, dtype=float)


def pose_matrix(pose: np.ndarray) -> np.ndarray:
    """Homogeneous transform from an SDF pose (R = Rz * Ry * Rx)."""
    x, y, z, roll, pitch, yaw = pose
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = (x, y, z)
    return transform


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to an (N, 3) point array."""
    if len(points) == 0:
        return points.reshape(0, 3)
    homo = np.concatenate([points, np.ones((len(points), 1))], axis=1)
    return (transform @ homo.T).T[:, :3]


def _segment(p0: np.ndarray, p1: np.ndarray) -> tuple[list[float], list[float]] | None:
    if float(np.linalg.norm(p1 - p0)) < MIN_SEGMENT_M:
        return None
    return (
        [round(float(p0[0]), 4), round(float(p0[1]), 4)],
        [round(float(p1[0]), 4), round(float(p1[1]), 4)],
    )


def _plane_hits(points: np.ndarray, slice_z: float) -> list[np.ndarray]:
    """Intersect every edge of a point set with the horizontal plane."""
    hits: list[np.ndarray] = []
    count = len(points)
    for i in range(count):
        a = points[i]
        b = points[(i + 1) % count]
        da = a[2] - slice_z
        db = b[2] - slice_z
        if da == 0.0:
            hits.append(a.copy())
        if da * db < 0.0:
            t = da / (da - db)
            hits.append(a + t * (b - a))
    return hits


def segments_from_polygon(world_points: np.ndarray, slice_z: float) -> list[tuple[list[float], list[float]]]:
    """Convex-ish outline of a planar slice through a closed ring of points."""
    if len(world_points) == 0:
        return []
    z_min = float(world_points[:, 2].min())
    z_max = float(world_points[:, 2].max())
    if (z_max - z_min) < MIN_HEIGHT_M or not (z_min <= slice_z <= z_max):
        return []
    hits = _plane_hits(world_points, slice_z)
    if len(hits) < 2:
        return []
    xy = np.array([[p[0], p[1]] for p in hits], dtype=float)
    center = xy.mean(axis=0)
    angles = np.arctan2(xy[:, 1] - center[1], xy[:, 0] - center[0])
    order = np.argsort(angles)
    ring = xy[order]
    segments = []
    for i in range(len(ring)):
        seg = _segment(ring[i], ring[(i + 1) % len(ring)])
        if seg is not None:
            segments.append(seg)
    return segments


def box_corners(size: np.ndarray) -> np.ndarray:
    """Eight corners of a box centered on the origin. size is (sx, sy, sz)."""
    hx, hy, hz = size / 2.0
    signs = np.array(
        [
            [-1, -1, -1],
            [1, -1, -1],
            [1, 1, -1],
            [-1, 1, -1],
            [-1, -1, 1],
            [1, -1, 1],
            [1, 1, 1],
            [-1, 1, 1],
        ],
        dtype=float,
    )
    return signs * np.array([hx, hy, hz])


def segments_from_box(size: np.ndarray, world_from_box: np.ndarray, slice_z: float):
    """XY outline where a box crosses ``slice_z``. Short boxes are floors and are skipped."""
    corners = transform_points(box_corners(size), world_from_box)
    # Walk the 12 box edges explicitly so a rotated box still slices.
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    z_min = float(corners[:, 2].min())
    z_max = float(corners[:, 2].max())
    if (z_max - z_min) < MIN_HEIGHT_M or not (z_min <= slice_z <= z_max):
        return []
    hits = []
    for i, j in edges:
        a, b = corners[i], corners[j]
        da, db = a[2] - slice_z, b[2] - slice_z
        if da == 0.0:
            hits.append(a[:2].copy())
        if da * db < 0.0:
            t = da / (da - db)
            point = a + t * (b - a)
            hits.append(point[:2].copy())
    if len(hits) < 3:
        return []
    xy = np.unique(np.round(np.array(hits), 5), axis=0)
    if len(xy) < 3:
        return []
    center = xy.mean(axis=0)
    order = np.argsort(np.arctan2(xy[:, 1] - center[1], xy[:, 0] - center[0]))
    ring = xy[order]
    segments = []
    for i in range(len(ring)):
        seg = _segment(ring[i], ring[(i + 1) % len(ring)])
        if seg is not None:
            segments.append(seg)
    return segments


def segments_from_cylinder(radius: float, length: float, world_from_cyl: np.ndarray, slice_z: float, sides: int = 12):
    """XY outline where a cylinder crosses ``slice_z``. Short cylinders are skipped."""
    half = length / 2.0
    angles = np.linspace(0.0, 2.0 * np.pi, sides, endpoint=False)
    ring = np.stack([radius * np.cos(angles), radius * np.sin(angles), np.zeros(sides)], axis=1)
    bottom = ring.copy()
    bottom[:, 2] = -half
    top = ring.copy()
    top[:, 2] = half
    world = transform_points(np.vstack([bottom, top]), world_from_cyl)
    z_min = float(world[:, 2].min())
    z_max = float(world[:, 2].max())
    if (z_max - z_min) < MIN_HEIGHT_M or not (z_min <= slice_z <= z_max):
        return []
    # Slice the vertical edges.
    hits = []
    for i in range(sides):
        a = world[i]
        b = world[i + sides]
        da, db = a[2] - slice_z, b[2] - slice_z
        if abs(da) < 1e-9 and abs(db) < 1e-9:
            continue
        if da * db <= 0.0 and (da - db) != 0.0:
            t = da / (da - db)
            point = a + t * (b - a)
            hits.append(point[:2])
    if len(hits) < 3:
        return []
    xy = np.array(hits)
    center = xy.mean(axis=0)
    order = np.argsort(np.arctan2(xy[:, 1] - center[1], xy[:, 0] - center[0]))
    ring_xy = xy[order]
    segments = []
    for i in range(len(ring_xy)):
        seg = _segment(ring_xy[i], ring_xy[(i + 1) % len(ring_xy)])
        if seg is not None:
            segments.append(seg)
    return segments


def triangle_slice(a: np.ndarray, b: np.ndarray, c: np.ndarray, slice_z: float):
    """Return the XY segment where a triangle crosses z = slice_z."""
    verts = (a, b, c)
    above = [v for v in verts if v[2] >= slice_z]
    below = [v for v in verts if v[2] < slice_z]
    if not above or not below:
        return None
    if len(below) == 1:
        lone, pair = below, above
    else:
        lone, pair = above, below
    hits = []
    for other in pair:
        da = lone[0][2] - slice_z
        db = other[2] - slice_z
        denom = da - db
        if denom == 0.0:
            continue
        t = da / denom
        point = lone[0] + t * (other - lone[0])
        hits.append(point[:2])
    if len(hits) != 2:
        return None
    return _segment(hits[0], hits[1])


def segments_from_triangles(vertices: np.ndarray, faces: np.ndarray, world_from_mesh: np.ndarray, scale: np.ndarray, slice_z: float):
    """XY segments where mesh triangles cross ``slice_z`` in the world frame."""
    if len(vertices) == 0 or len(faces) == 0:
        return []
    scaled = vertices * scale.reshape(1, 3)
    world = transform_points(scaled, world_from_mesh)
    z_min = float(world[:, 2].min())
    z_max = float(world[:, 2].max())
    if (z_max - z_min) < MIN_HEIGHT_M or not (z_min - 1e-6 <= slice_z <= z_max + 1e-6):
        return []
    segments = []
    for face in faces:
        seg = triangle_slice(world[face[0]], world[face[1]], world[face[2]], slice_z)
        if seg is not None:
            segments.append(seg)
    return segments


def load_obj(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load triangle vertices from a Wavefront OBJ file."""
    vertices: list[list[float]] = []
    faces: list[list[int]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("v "):
            parts = line.split()
            vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
        elif line.startswith("f "):
            idx = []
            for token in line.split()[1:]:
                idx.append(int(token.split("/")[0]) - 1)
            for i in range(1, len(idx) - 1):
                faces.append([idx[0], idx[i], idx[i + 1]])
    if not vertices or not faces:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=int)
    return np.array(vertices, dtype=float), np.array(faces, dtype=int)


def load_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load a binary or ASCII STL as triangles."""
    data = path.read_bytes()
    if data[:5].lower() == b"solid" and b"\n" in data[:200]:
        return _load_ascii_stl(data.decode("utf-8", errors="replace"))
    return _load_binary_stl(data)


def _load_ascii_stl(text: str) -> tuple[np.ndarray, np.ndarray]:
    vertices: list[list[float]] = []
    faces: list[list[int]] = []
    current: list[int] = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[0] == "vertex":
            current.append(len(vertices))
            vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            if len(current) == 3:
                faces.append(current)
                current = []
    if not vertices:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=int)
    return np.array(vertices, dtype=float), np.array(faces, dtype=int)


def _load_binary_stl(data: bytes) -> tuple[np.ndarray, np.ndarray]:
    if len(data) < 84:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=int)
    count = struct.unpack_from("<I", data, 80)[0]
    vertices = np.zeros((count * 3, 3), dtype=float)
    faces = np.zeros((count, 3), dtype=int)
    offset = 84
    for i in range(count):
        if offset + 50 > len(data):
            break
        nums = struct.unpack_from("<12fH", data, offset)
        vertices[3 * i: 3 * i + 3] = np.array(nums[3:12]).reshape(3, 3)
        faces[i] = (3 * i, 3 * i + 1, 3 * i + 2)
        offset += 50
    return vertices, faces


def load_collada(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load the first triangle mesh from a COLLADA file."""
    tree = ET.parse(path)
    root = tree.getroot()
    ns = ""
    if root.tag.startswith("{"):
        ns = root.tag.split("}")[0] + "}"

    def local(tag: str) -> str:
        return f"{ns}{tag}"

    arrays: dict[str, np.ndarray] = {}
    for array in root.iter(local("float_array")):
        array_id = array.get("id")
        if not array_id or array.text is None:
            continue
        arrays[array_id] = np.fromstring(array.text, sep=" ")

    sources: dict[str, np.ndarray] = {}
    for source in root.iter(local("source")):
        source_id = source.get("id")
        float_array = source.find(local("float_array"))
        accessor = source.find(f"{local('technique_common')}/{local('accessor')}")
        if source_id is None or float_array is None or accessor is None:
            continue
        values = arrays.get(float_array.get("id", ""), np.array([]))
        stride = int(accessor.get("stride", "3"))
        if stride == 0 or len(values) < stride:
            continue
        usable = values[: (len(values) // stride) * stride]
        sources[source_id] = usable.reshape(-1, stride)[:, :3]

    vertices_map: dict[str, str] = {}
    for vertices in root.iter(local("vertices")):
        vertices_id = vertices.get("id")
        for inp in vertices.findall(local("input")):
            if inp.get("semantic") == "POSITION":
                vertices_map[vertices_id or ""] = inp.get("source", "").lstrip("#")

    all_vertices: list[np.ndarray] = []
    all_faces: list[np.ndarray] = []
    base = 0
    for mesh in root.iter(local("mesh")):
        for triangles in list(mesh.findall(local("triangles"))) + list(mesh.findall(local("polylist"))):
            inputs = triangles.findall(local("input"))
            if not inputs:
                continue
            position_source = None
            position_offset = 0
            stride = 0
            for inp in inputs:
                offset = int(inp.get("offset", "0"))
                stride = max(stride, offset + 1)
                if inp.get("semantic") == "VERTEX":
                    position_offset = offset
                    pointed = inp.get("source", "").lstrip("#")
                    position_source = vertices_map.get(pointed, pointed)
                elif inp.get("semantic") == "POSITION":
                    position_offset = offset
                    position_source = inp.get("source", "").lstrip("#")
            if position_source is None or position_source not in sources:
                continue
            index_node = triangles.find(local("p"))
            if index_node is None or not index_node.text:
                continue
            indices = np.fromstring(index_node.text, sep=" ", dtype=int)
            if stride == 0 or len(indices) < stride:
                continue
            indices = indices[: (len(indices) // stride) * stride].reshape(-1, stride)
            position_index = indices[:, position_offset]
            verts = sources[position_source]
            if position_index.max(initial=0) >= len(verts):
                continue
            # polylist may not be triangles; group every 3 corners when count matches
            if len(position_index) % 3 != 0:
                continue
            faces = position_index.reshape(-1, 3) + base
            all_vertices.append(verts)
            all_faces.append(faces)
            base += len(verts)
    if not all_vertices:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=int)
    return np.vstack(all_vertices), np.vstack(all_faces)


_MESH_CACHE: dict[str, tuple[np.ndarray, np.ndarray]] = {}


def load_mesh(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load a mesh, caching by path so repeated furniture instances are cheap."""
    key = str(path.resolve())
    cached = _MESH_CACHE.get(key)
    if cached is not None:
        return cached
    suffix = path.suffix.lower()
    if suffix == ".obj":
        loaded = load_obj(path)
    elif suffix == ".stl":
        loaded = load_stl(path)
    elif suffix in {".dae", ".collada"}:
        loaded = load_collada(path)
    else:
        loaded = (np.zeros((0, 3)), np.zeros((0, 3), dtype=int))
    _MESH_CACHE[key] = loaded
    return loaded


def resolve_uri(uri: str, sdf_path: Path, model_roots: list[Path]) -> Path | None:
    """Resolve model://, file://, and paths relative to the SDF."""
    uri = uri.strip()
    if uri.startswith("model://"):
        rest = uri[len("model://"):]
        for root in model_roots:
            candidate = root / rest
            if candidate.is_file():
                return candidate
        return None
    if uri.startswith("file://"):
        candidate = Path(uri[len("file://"):])
        return candidate if candidate.is_file() else None
    if uri.startswith("https://") or uri.startswith("http://"):
        return None
    candidate = (sdf_path.parent / uri).resolve()
    return candidate if candidate.is_file() else None


def _child_pose(element: ET.Element) -> np.ndarray:
    pose = element.find("pose")
    values = parse_pose(pose.text if pose is not None else None)
    if pose is not None and pose.get("degrees", "false").lower() == "true":
        values[3:] = np.deg2rad(values[3:])
    return values


def _scale_of(mesh: ET.Element) -> np.ndarray:
    scale = mesh.find("scale")
    if scale is None or not (scale.text or "").strip():
        return np.ones(3)
    values = [float(part) for part in scale.text.split()]
    if len(values) == 1:
        return np.array([values[0], values[0], values[0]])
    return np.array(values[:3], dtype=float)


def _append(records: list[dict], segments, source: str, kind: str) -> None:
    for start, end in segments:
        records.append({"start": start, "end": end, "source": source, "kind": kind})


def collisions_to_segments(sdf_path: Path, model_roots: list[Path], slice_z: float) -> list[dict]:
    """Slice every collision in an SDF world, following local model:// includes."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        return []
    records: list[dict] = []
    for model in list(world.findall("model")):
        _model_segments(model, np.eye(4), sdf_path, model_roots, slice_z, records, model.get("name", "model"))
    for include in world.findall("include"):
        uri_node = include.find("uri")
        if uri_node is None or not (uri_node.text or "").strip():
            continue
        uri = uri_node.text.strip()
        included = resolve_uri(uri if uri.endswith(".sdf") else uri.rstrip("/") + "/model.sdf", sdf_path, model_roots)
        if included is None and uri.startswith("model://"):
            included = resolve_uri(uri.rstrip("/") + "/model.sdf", sdf_path, model_roots)
        if included is None:
            continue
        include_pose = pose_matrix(_child_pose(include))
        included_tree = ET.parse(included)
        included_model = included_tree.getroot().find("model")
        if included_model is None:
            continue
        name = included_model.get("name", included.parent.name)
        _model_segments(included_model, include_pose, included, model_roots, slice_z, records, name)
    return _unique_segments(records)


def _model_segments(model, parent, sdf_path, model_roots, slice_z, records, model_name) -> None:
    model_tf = parent @ pose_matrix(_child_pose(model))
    for link in model.findall("link"):
        link_tf = model_tf @ pose_matrix(_child_pose(link))
        link_name = link.get("name", "link")
        for collision in link.findall("collision"):
            col_tf = link_tf @ pose_matrix(_child_pose(collision))
            source = f"{model_name}/{link_name}/{collision.get('name', 'collision')}"
            geometry = collision.find("geometry")
            if geometry is None:
                continue
            _geometry_segments(geometry, col_tf, sdf_path, model_roots, slice_z, records, source)


def _geometry_segments(geometry, col_tf, sdf_path, model_roots, slice_z, records, source) -> None:
    box = geometry.find("box")
    if box is not None:
        size_text = box.findtext("size", "1 1 1")
        size = np.array([float(v) for v in size_text.split()], dtype=float)
        _append(records, segments_from_box(size, col_tf, slice_z), source, "box")
        return
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.findtext("radius", "0.1"))
        length = float(cylinder.findtext("length", "1"))
        _append(records, segments_from_cylinder(radius, length, col_tf, slice_z), source, "cylinder")
        return
    mesh = geometry.find("mesh")
    if mesh is None:
        return
    uri = mesh.findtext("uri", "").strip()
    path = resolve_uri(uri, sdf_path, model_roots)
    if path is None:
        records.append({
            "start": [0.0, 0.0],
            "end": [0.0, 0.0],
            "source": source,
            "kind": "unresolved",
            "uri": uri,
        })
        return
    vertices, faces = load_mesh(path)
    _append(
        records,
        segments_from_triangles(vertices, faces, col_tf, _scale_of(mesh), slice_z),
        source,
        "mesh",
    )


def _unique_segments(records: list[dict]) -> list[dict]:
    seen: set[tuple] = set()
    unique: list[dict] = []
    for record in records:
        if record.get("kind") == "unresolved":
            unique.append(record)
            continue
        key = tuple(sorted((tuple(record["start"]), tuple(record["end"]))))
        if key in seen:
            continue
        seen.add(key)
        unique.append(record)
    return unique


def export_world(sdf_path: Path, slice_z: float = 0.5) -> dict:
    """Wall-map document for one world. ``frame`` is ``world_enu`` (Gazebo ENU metres)."""
    gz_dir = sdf_path.resolve().parents[1]
    model_roots = [gz_dir / "models", sdf_path.resolve().parent]
    raw = collisions_to_segments(sdf_path, model_roots, slice_z)
    segments = [item for item in raw if item.get("kind") != "unresolved"]
    unresolved = sorted({item.get("uri", "") for item in raw if item.get("kind") == "unresolved"})
    return {
        "world": sdf_path.stem,
        # Gazebo world ENU. Spawn-relative points use TF world -> spawn.
        "frame": "world_enu",
        "units": "m",
        "slice_z_m": slice_z,
        "min_height_m": MIN_HEIGHT_M,
        "min_segment_m": MIN_SEGMENT_M,
        "description": (
            "2D line segments where collision geometry crosses slice_z_m. "
            "Coordinates are Gazebo world ENU metres (x east, y north). "
            "A control node can compute distance from a point to the nearest segment."
        ),
        "segment_count": len(segments),
        "unresolved_uris": unresolved,
        "segments": segments,
    }


def main() -> int:
    """Write ``includes/gz/walls/<world>.json`` for every world SDF."""
    gz_dir = Path(__file__).resolve().parents[1]
    out_dir = gz_dir / "walls"
    out_dir.mkdir(parents=True, exist_ok=True)
    for world in sorted((gz_dir / "worlds").glob("*.sdf")):
        print(f"slicing {world.name}")
        document = export_world(world)
        destination = out_dir / f"{world.stem}.json"
        destination.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
        print(
            f"  {document['segment_count']} segments, "
            f"{len(document['unresolved_uris'])} unresolved"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
