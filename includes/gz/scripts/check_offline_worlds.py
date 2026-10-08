#!/usr/bin/env python3
"""Fail if a Gazebo world still references Gazebo Fuel.

Checked hosts: fuel.gazebosim.org and fuel.ignitionrobotics.org.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from fuel_assets import find_fuel_references, worlds_dir


def main(argv: list[str] | None = None) -> int:
    """Exit 1 when a world SDF still names a Gazebo Fuel host."""
    parser = argparse.ArgumentParser(
        description="Fail if a Gazebo world still references Gazebo Fuel.",
    )
    parser.add_argument(
        "worlds_dir",
        nargs="?",
        help="directory of SDF worlds (default: includes/gz/worlds)",
    )
    args = parser.parse_args(argv)
    root = Path(args.worlds_dir) if args.worlds_dir else worlds_dir()
    if not root.is_dir():
        print(f"offline world check failed: not a directory: {root}", file=sys.stderr)
        return 1
    hits = find_fuel_references(root)
    if hits:
        print(f"offline world check failed: {len(hits)} Fuel reference(s)", file=sys.stderr)
        for hit in hits:
            print(hit, file=sys.stderr)
        return 1
    print(f"offline world check passed ({root})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
