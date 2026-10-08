#!/usr/bin/env python3
"""Rewrite Fuel URIs in Gazebo worlds to local model:// paths.

Worlds that point at https://fuel.gazebosim.org or the older
https://fuel.ignitionrobotics.org host are downloaded into
``includes/gz/models/<name>/`` and the SDF is updated. Re-running the
script skips files that are already on disk.
"""

from __future__ import annotations

import json
import re
import sys
import urllib.parse
import urllib.request
from pathlib import Path

FUEL_HOSTS = ("fuel.gazebosim.org", "fuel.ignitionrobotics.org")
USER_AGENT = "px4-sim-offline-worlds"

FILE_URL = re.compile(
    r"https://(?:" + "|".join(re.escape(h) for h in FUEL_HOSTS) + r")"
    r"/1\.0/([^/]+)/models/([^/]+)/(\d+)/files/([^\s\"'<>]+)"
)
INCLUDE_URL = re.compile(
    r"https://(?:" + "|".join(re.escape(h) for h in FUEL_HOSTS) + r")"
    r"/1\.0/([^/]+)/models/([^<\"']+)"
)

OBJ_MTLLIB = re.compile(r"^\s*mtllib\s+(\S+)", re.MULTILINE)
MTL_MAP = re.compile(r"^\s*map_\w+\s+(\S+)", re.MULTILINE)


def repo_gz_dir() -> Path:
    return Path(__file__).resolve().parents[1]


def models_dir() -> Path:
    return repo_gz_dir() / "models"


def worlds_dir() -> Path:
    return repo_gz_dir() / "worlds"


def _get(url: str) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(request, timeout=120) as response:
        return response.read()


def fuel_model_version(owner: str, model_name: str) -> int:
    """Look up the current Fuel version number for a model."""
    quoted = urllib.parse.quote(model_name)
    url = f"https://fuel.gazebosim.org/1.0/{owner}/models/{quoted}"
    meta = json.loads(_get(url).decode("utf-8"))
    return int(meta["version"])


def download_fuel_file(owner: str, model: str, version: int, relative: str, dest: Path) -> None:
    """Download one file from a pinned Fuel model version."""
    if dest.is_file() and dest.stat().st_size > 0:
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    quoted_model = urllib.parse.quote(model)
    quoted_rel = "/".join(urllib.parse.quote(part) for part in relative.split("/"))
    url = (
        f"https://fuel.gazebosim.org/1.0/{owner}/models/"
        f"{quoted_model}/{version}/files/{quoted_rel}"
    )
    dest.write_bytes(_get(url))
    print(f"downloaded {dest.relative_to(models_dir())} ({dest.stat().st_size} bytes)")


def _follow_mesh_sidecars(owner: str, model: str, version: int, relative: str, dest: Path) -> None:
    """Pull MTL files and textures referenced by an OBJ or MTL."""
    if dest.suffix.lower() == ".obj":
        text = dest.read_text(encoding="utf-8", errors="replace")
        for match in OBJ_MTLLIB.finditer(text):
            sidecar = match.group(1)
            sidecar_rel = str(Path(relative).parent / sidecar)
            sidecar_dest = dest.parent / sidecar
            download_fuel_file(owner, model, version, sidecar_rel, sidecar_dest)
            _follow_mesh_sidecars(owner, model, version, sidecar_rel, sidecar_dest)
    elif dest.suffix.lower() == ".mtl":
        text = dest.read_text(encoding="utf-8", errors="replace")
        for match in MTL_MAP.finditer(text):
            texture = match.group(1)
            texture_rel = str(Path(relative).parent / texture)
            download_fuel_file(owner, model, version, texture_rel, dest.parent / texture)


def slug_model_dir(model_sdf: str, fuel_name: str) -> str:
    """Directory name Gazebo will resolve for model://."""
    match = re.search(r"<model\s+name=[\"']([^\"']+)[\"']", model_sdf)
    if match:
        return match.group(1)
    return urllib.parse.unquote(fuel_name).strip().lower().replace(" ", "_")


def vendor_file_url(url: str) -> str:
    """Download the file behind a Fuel URL and return a model:// URI."""
    match = FILE_URL.fullmatch(url)
    if match is None:
        raise ValueError(f"not a Fuel file URL: {url}")
    owner, model, version, relative = match.groups()
    model = urllib.parse.unquote(model)
    relative = urllib.parse.unquote(relative)
    dest = models_dir() / model / relative
    download_fuel_file(owner, model, int(version), relative, dest)
    _follow_mesh_sidecars(owner, model, int(version), relative, dest)
    _ensure_model_config(models_dir() / model, model)
    return f"model://{model}/{relative}"


def vendor_include_url(url: str) -> str:
    """Download a Fuel model include and return model://<name>."""
    match = INCLUDE_URL.fullmatch(url)
    if match is None:
        raise ValueError(f"not a Fuel model URL: {url}")
    owner, fuel_name = match.groups()
    fuel_name = urllib.parse.unquote(fuel_name).strip()
    version = fuel_model_version(owner, fuel_name)
    sdf_bytes = _download_named(owner, fuel_name, version, "model.sdf")
    model_dir_name = slug_model_dir(sdf_bytes.decode("utf-8", errors="replace"), fuel_name)
    root = models_dir() / model_dir_name
    root.mkdir(parents=True, exist_ok=True)
    sdf_path = root / "model.sdf"
    if not sdf_path.is_file():
        sdf_path.write_bytes(sdf_bytes)
    _try_download(owner, fuel_name, version, "model.config", root / "model.config")
    _pull_uris_from_sdf(sdf_bytes.decode("utf-8", errors="replace"), owner, fuel_name, version, root)
    print(f"vendored model {fuel_name} -> {root}")
    return f"model://{model_dir_name}"


def _download_named(owner: str, model: str, version: int, relative: str) -> bytes:
    quoted_model = urllib.parse.quote(model)
    quoted_rel = "/".join(urllib.parse.quote(part) for part in relative.split("/"))
    url = (
        f"https://fuel.gazebosim.org/1.0/{owner}/models/"
        f"{quoted_model}/{version}/files/{quoted_rel}"
    )
    return _get(url)


def _try_download(owner: str, model: str, version: int, relative: str, dest: Path) -> None:
    if dest.is_file() and dest.stat().st_size > 0:
        return
    try:
        data = _download_named(owner, model, version, relative)
    except Exception as exc:
        print(f"skip {relative}: {exc}")
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(data)


def _pull_uris_from_sdf(sdf_text: str, owner: str, fuel_name: str, version: int, root: Path) -> None:
    """Download model:// meshes referenced by a vendored model.sdf."""
    for uri in re.findall(r"<uri>\s*([^<]+?)\s*</uri>", sdf_text):
        uri = uri.strip()
        if not uri.startswith("model://"):
            continue
        rest = uri[len("model://"):]
        parts = rest.split("/", 1)
        if len(parts) == 2:
            relative = parts[1]
        else:
            continue
        dest = root / relative
        _try_download(owner, fuel_name, version, relative, dest)
        if dest.is_file():
            _follow_local_sidecars(owner, fuel_name, version, relative, dest)


def _follow_local_sidecars(owner: str, model: str, version: int, relative: str, dest: Path) -> None:
    if dest.suffix.lower() == ".obj":
        text = dest.read_text(encoding="utf-8", errors="replace")
        for match in OBJ_MTLLIB.finditer(text):
            sidecar_rel = str(Path(relative).parent / match.group(1))
            sidecar_dest = dest.parent / match.group(1)
            _try_download(owner, model, version, sidecar_rel, sidecar_dest)
            if sidecar_dest.is_file():
                _follow_local_sidecars(owner, model, version, sidecar_rel, sidecar_dest)
    elif dest.suffix.lower() == ".mtl":
        text = dest.read_text(encoding="utf-8", errors="replace")
        for match in MTL_MAP.finditer(text):
            texture_rel = str(Path(relative).parent / match.group(1))
            _try_download(owner, model, version, texture_rel, dest.parent / match.group(1))


def _ensure_model_config(directory: Path, name: str) -> None:
    config = directory / "model.config"
    if config.is_file():
        return
    directory.mkdir(parents=True, exist_ok=True)
    config.write_text(
        "<?xml version=\"1.0\"?>\n"
        "<model>\n"
        f"  <name>{name}</name>\n"
        "  <version>1.0</version>\n"
        "  <sdf version=\"1.6\">model.sdf</sdf>\n"
        "  <description>Vendored from Gazebo Fuel for offline simulation.</description>\n"
        "</model>\n",
        encoding="utf-8",
    )


def rewrite_world(path: Path) -> int:
    """Replace Fuel URIs in one world file. Return the number of replacements."""
    text = path.read_text(encoding="utf-8")
    count = 0

    def replace_file(match: re.Match) -> str:
        nonlocal count
        count += 1
        return vendor_file_url(match.group(0))

    text = FILE_URL.sub(replace_file, text)

    def replace_include(match: re.Match) -> str:
        nonlocal count
        url = match.group(0)
        if "/files/" in url:
            return url
        count += 1
        return vendor_include_url(url)

    text = INCLUDE_URL.sub(replace_include, text)
    if count:
        path.write_text(text, encoding="utf-8")
        print(f"rewrote {count} Fuel URI(s) in {path.name}")
    return count


def find_fuel_references(root: Path) -> list[str]:
    """Return 'path:line: text' hits for Fuel hosts under root."""
    hits: list[str] = []
    if not root.exists():
        return hits
    files = [root] if root.is_file() else sorted(root.rglob("*.sdf"))
    for sdf in files:
        for number, line in enumerate(sdf.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            if any(host in line for host in FUEL_HOSTS):
                hits.append(f"{sdf}:{number}: {line.strip()}")
    return hits


def main() -> int:
    total = 0
    for world in sorted(worlds_dir().glob("*.sdf")):
        total += rewrite_world(world)
    remaining = find_fuel_references(worlds_dir())
    if remaining:
        print("Fuel URIs still present:", file=sys.stderr)
        for hit in remaining:
            print(hit, file=sys.stderr)
        return 1
    print(f"offline worlds OK ({total} URI(s) rewritten this run)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
