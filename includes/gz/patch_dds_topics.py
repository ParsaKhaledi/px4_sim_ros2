#!/usr/bin/env python3
"""Publish /fmu/out/distance_sensor from the PX4 DDS client.

PX4 v1.17 lists the inbound distance_sensor topic and does not publish
the outbound one. The edit is idempotent: a second run leaves the file
unchanged. Call it before `make px4_sitl`, which compiles this yaml in.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

TOPIC = "/fmu/out/distance_sensor"
BLOCK = (
    "  - topic: /fmu/out/distance_sensor\n"
    "    type: px4_msgs::msg::DistanceSensor\n"
)


def ensure_distance_sensor(text: str) -> tuple[str, bool]:
    pub = text.find("\npublications:")
    if pub < 0 and not text.startswith("publications:"):
        raise ValueError("dds_topics.yaml has no publications section")
    if text.startswith("publications:"):
        pub = 0
    else:
        pub += 1
    sub = text.find("\nsubscriptions:", pub)
    section = text[pub:] if sub < 0 else text[pub:sub]
    if TOPIC in section:
        return text, False
    line_end = text.find("\n", pub)
    if line_end < 0:
        insert_at = len(text)
        prefix = text + "\n"
    else:
        insert_at = line_end + 1
        prefix = text[:insert_at]
    suffix = text[insert_at:]
    return prefix + BLOCK + suffix, True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    args = parser.parse_args(argv)
    original = args.path.read_text(encoding="utf-8")
    try:
        updated, changed = ensure_distance_sensor(original)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    if not changed:
        print(f"distance_sensor publication already present in {args.path}")
        return 0
    args.path.write_text(updated, encoding="utf-8")
    print(f"Added {TOPIC} to publications in {args.path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
