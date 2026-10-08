#!/usr/bin/env python3
"""Fail if a Gazebo world still references Gazebo Fuel.

Checked hosts: fuel.gazebosim.org and fuel.ignitionrobotics.org.
"""

from __future__ import annotations

import sys
from pathlib import Path

from fuel_assets import find_fuel_references, worlds_dir


def main() -> int:
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else worlds_dir()
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
