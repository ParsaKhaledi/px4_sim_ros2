#!/usr/bin/env python3
"""Decide whether CI must build the PX4 image or can pull the published tag.

A rebuild is required when a changed path is copied into the image or
changes what the Dockerfile clones and compiles:

- dockerFile/**
- DockerBuild.sh
- versions.env
- includes/gz/patch_dds_topics.py
- ros2_ws package manifests (package.xml, CMakeLists.txt, setup.py,
  setup.cfg, pyproject.toml)

Worlds, start scripts, Compose, parameter files, and tests are mounted
or interpreted at runtime and do not rebuild the image.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import PurePosixPath

EXACT = {
    "DockerBuild.sh",
    "versions.env",
    "includes/gz/patch_dds_topics.py",
}
MANIFESTS = {
    "package.xml",
    "CMakeLists.txt",
    "setup.py",
    "setup.cfg",
    "pyproject.toml",
}


def needs_rebuild(paths: list[str]) -> bool:
    for raw in paths:
        path = raw.strip()
        if not path:
            continue
        if path in EXACT or path.startswith("dockerFile/"):
            return True
        pure = PurePosixPath(path)
        if pure.parts and pure.parts[0] == "ros2_ws" and pure.name in MANIFESTS:
            return True
    return False


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", help="Changed paths. Empty reads stdin.")
    args = parser.parse_args(argv)
    paths = list(args.paths)
    if not paths:
        paths = [line.strip() for line in sys.stdin.read().splitlines()]
    mode = "build" if needs_rebuild(paths) else "pull"
    print(mode)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
