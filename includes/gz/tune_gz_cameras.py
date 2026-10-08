#!/usr/bin/env python3
"""Lower camera rate and resolution on a copied Gazebo model.

The Oak-D SDF files in this repo are left alone. Call this on the copy
under the PX4 tree when a software-rendered flight sets
GZ_CAMERA_UPDATE_RATE. IMU sensors are not cameras and stay as they are.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

_SENSOR = re.compile(r"<sensor\b.*?</sensor>", re.DOTALL)
_RATE = re.compile(r"<update_rate>\s*[0-9.]+\s*</update_rate>")
_WIDTH = re.compile(r"<width>\s*[0-9]+\s*</width>")
_HEIGHT = re.compile(r"<height>\s*[0-9]+\s*</height>")


def _is_camera(block: str) -> bool:
    return 'type="camera"' in block or 'type="depth_camera"' in block


def tune_cameras(text: str, rate: str, width: str, height: str) -> str:
    def replace(match: re.Match) -> str:
        block = match.group(0)
        if not _is_camera(block):
            return block
        block = _RATE.sub(f"<update_rate>{rate}</update_rate>", block, count=1)
        block = _WIDTH.sub(f"<width>{width}</width>", block, count=1)
        block = _HEIGHT.sub(f"<height>{height}</height>", block, count=1)
        return block

    return _SENSOR.sub(replace, text)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--rate", required=True)
    parser.add_argument("--width", required=True)
    parser.add_argument("--height", required=True)
    args = parser.parse_args(argv)
    original = args.model.read_text(encoding="utf-8")
    updated = tune_cameras(original, str(args.rate), str(args.width), str(args.height))
    if updated == original:
        print(f"No camera sensors updated in {args.model}", file=sys.stderr)
        return 1
    args.model.write_text(updated, encoding="utf-8")
    print(f"Camera sensors in {args.model} set to {args.rate} Hz, {args.width}x{args.height}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
