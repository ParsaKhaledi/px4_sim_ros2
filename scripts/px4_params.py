#!/usr/bin/env python3
"""Merge PX4 parameter files with PX4_PARAM_* overrides and render a .post script.

PX4 v1.17 sources `$autostart_file.post` after commander and ekf2 have started.
apply() writes one .post beside every airframe, moves that source to just
before `dataman start`, and clears the SITL parameter store.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

NAME_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
VALUE_RE = re.compile(r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)$")
PARAM_META_RE = re.compile(r'<parameter\b[^>]*\bname="([^"]+)"')
JSON_NAME_RE = re.compile(r'"name"\s*:\s*"([A-Z][A-Z0-9_]*)"')
POST_LINE = '[ -e "$autostart_file".post ] && . "$autostart_file".post'
EARLY_HOOK = (
    "# Parameter files run after the airframe and before commander and ekf2.\n"
    f"{POST_LINE}\n"
)


class ParamConflict(Exception):
    """Two listed files set one parameter to different values."""


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_params_dir() -> Path:
    return repo_root() / "config" / "px4" / "params"


def parse_params_text(text: str, filename: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for line_number, raw in enumerate(text.splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 2 or not NAME_RE.match(parts[0]) or not VALUE_RE.match(parts[1]):
            raise ValueError(f"{filename}:{line_number}: expected NAME VALUE, got {raw!r}")
        values[parts[0]] = parts[1]
    return values


def file_list(environ: dict[str, str] | None = None) -> list[str]:
    env = os.environ if environ is None else environ
    raw = env.get("PX4_PARAM_FILES", "sim.params")
    return [part.strip() for part in raw.split(",") if part.strip()]


def load_param_files(directory: Path, names: list[str]) -> tuple[dict[str, str], list[tuple[str, str, str]]]:
    merged: dict[str, str] = {}
    origin: dict[str, str] = {}
    ordered: list[tuple[str, str, str]] = []
    for name in names:
        path = directory / name
        if not path.is_file():
            raise FileNotFoundError(f"Parameter file not found: {path}")
        for key, value in parse_params_text(path.read_text(encoding="utf-8"), name).items():
            if key in merged and merged[key] != value:
                raise ParamConflict(
                    f"{key} differs in {origin[key]} ({merged[key]}) and {name} ({value})"
                )
            if key not in merged:
                ordered.append((key, value, name))
            merged[key] = value
            origin[key] = name
    return merged, ordered


def env_overrides(environ: dict[str, str] | None = None) -> list[tuple[str, str]]:
    env = os.environ if environ is None else environ
    found: list[tuple[str, str]] = []
    for key in sorted(env):
        if not key.startswith("PX4_PARAM_") or key == "PX4_PARAM_FILES":
            continue
        name = key[len("PX4_PARAM_"):]
        value = env[key]
        if not NAME_RE.match(name) or not VALUE_RE.match(value):
            raise ValueError(f"{key}={value} is not a NAME VALUE override")
        found.append((name, value))
    return found


def merge_expected(
    directory: Path | None = None,
    environ: dict[str, str] | None = None,
    overrides: list[tuple[str, str]] | None = None,
) -> dict[str, float]:
    env = os.environ if environ is None else environ
    params_dir = default_params_dir() if directory is None else directory
    merged, _ordered = load_param_files(params_dir, file_list(env))
    for name, value in env_overrides(env):
        merged[name] = value
    for name, value in overrides or []:
        merged[name] = value
    return {name: float(value) for name, value in merged.items()}


def render_post(ordered: list[tuple[str, str, str]], overrides: list[tuple[str, str]]) -> str:
    lines = ["# Generated at container start from PX4_PARAM_FILES."]

    def emit(name: str, value: str, label: str) -> None:
        lines.append(f'echo "PX4 params: {label} {name} {value}"')
        lines.append(f"if ! param set {name} {value}")
        lines.append("then")
        lines.append(f'\techo "PX4 params: failed to set {name} ({label})"')
        lines.append("\texit 1")
        lines.append("fi")

    for name, value, filename in ordered:
        emit(name, value, filename)
    for name, value in overrides:
        emit(name, value, "override")
    lines.append("")
    return "\n".join(lines)


def ensure_post_before_ekf2(text: str) -> str:
    """Source the airframe .post after set-defaults and before startup modules.

    PX4 v1.17 posix rcS sources the airframe, then starts dataman, commander,
    and ekf2, and only then sources `$autostart_file.post`. Modules that read
    parameters at start would otherwise miss the file. The hook moves to just
    before `dataman start`.
    """
    ekf = text.find("ekf2 start")
    dataman = text.find("dataman start")
    if ekf < 0 or dataman < 0 or dataman > ekf:
        raise ValueError("rcS has no dataman start before ekf2")
    first = text.find(POST_LINE)
    if first != -1 and first < ekf:
        return text
    updated = text
    if first != -1:
        end = first + len(POST_LINE)
        if end < len(updated) and updated[end] == "\n":
            end += 1
        updated = updated[:first] + updated[end:]
        dataman = updated.find("dataman start")
    return updated[:dataman] + EARLY_HOOK + "\n" + updated[dataman:]


def names_in_metadata(text: str) -> set[str]:
    names = set(PARAM_META_RE.findall(text))
    names.update(JSON_NAME_RE.findall(text))
    return names


def unknown_names(metadata_text: str, directory: Path) -> list[str]:
    known = names_in_metadata(metadata_text)
    if not known:
        raise ValueError("parameter metadata has no parameter names")
    problems = []
    for path in sorted(directory.glob("*.params")):
        for name in parse_params_text(path.read_text(encoding="utf-8"), path.name):
            if name not in known:
                problems.append(f"{path.name}: {name} is not in the PX4 parameter metadata")
    return problems


def clear_parameter_store(rootfs: Path) -> None:
    rootfs.mkdir(parents=True, exist_ok=True)
    for name in ("parameters.bson", "parameters_backup.bson"):
        path = rootfs / name
        if path.exists() or path.is_symlink():
            path.unlink()
    print("Cleared SITL parameters.bson and parameters_backup.bson")


def write_posts(airframes: Path, script: str) -> list[Path]:
    if not airframes.is_dir():
        raise FileNotFoundError(f"Airframe directory not found: {airframes}")
    written = []
    for path in sorted(airframes.iterdir()):
        if not path.is_file() or "." in path.name:
            continue
        dest = path.with_name(path.name + ".post")
        dest.write_text(script, encoding="utf-8")
        written.append(dest)
        print(f"Wrote {dest.name}")
    if not written:
        raise FileNotFoundError(f"No airframe scripts in {airframes}")
    return written


def apply(params_dir: Path, airframes: Path, rcs: Path, rootfs: Path) -> None:
    _merged, ordered = load_param_files(params_dir, file_list())
    overrides = env_overrides()
    for name, value in overrides:
        print(f"PX4 params: override {name} {value}")
    script = render_post(ordered, overrides)
    write_posts(airframes, script)
    if not rcs.is_file():
        raise FileNotFoundError(f"rcS not found: {rcs}")
    original = rcs.read_text(encoding="utf-8")
    updated = ensure_post_before_ekf2(original)
    if updated != original:
        rcs.write_text(updated, encoding="utf-8")
        print(f"Sourced airframe .post before ekf2 in {rcs}")
    else:
        print(f"Airframe .post already runs before ekf2 in {rcs}")
    clear_parameter_store(rootfs)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    apply_parser = sub.add_parser("apply", help="Write one .post per airframe and clear the SITL store")
    apply_parser.add_argument("--params-dir", type=Path, required=True)
    apply_parser.add_argument("--airframes", type=Path, required=True)
    apply_parser.add_argument("--rcs", type=Path, required=True)
    apply_parser.add_argument("--rootfs", type=Path, required=True)

    meta = sub.add_parser("check-metadata", help="Reject parameter names missing from the image metadata")
    meta.add_argument("--metadata", type=Path, required=True)
    meta.add_argument("--params-dir", type=Path, default=default_params_dir())

    args = parser.parse_args(argv)
    try:
        if args.command == "apply":
            apply(args.params_dir, args.airframes, args.rcs, args.rootfs)
            return 0
        problems = unknown_names(args.metadata.read_text(encoding="utf-8"), args.params_dir)
    except (ParamConflict, ValueError, FileNotFoundError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    if problems:
        print("Unknown PX4 parameters:", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1
    print(f"Parameter names in {args.params_dir} match {args.metadata}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
