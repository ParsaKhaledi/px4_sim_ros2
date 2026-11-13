#!/usr/bin/env python3
"""Fail if `param show` does not match the merged parameter files and env."""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

from px4_params import merge_expected


def parse_param_value(text: str, name: str) -> float | None:
    pattern = re.compile(rf"\b{re.escape(name)}\b")
    for line in text.splitlines():
        if pattern.search(line) is None:
            continue
        tail = line.rsplit(":", 1)[-1] if ":" in line else line
        numbers = re.findall(r"[-+]?\d+(?:\.\d+)?", tail)
        if numbers:
            return float(numbers[-1])
    return None


def mismatches(text: str, expected: dict[str, float] | None = None) -> list[str]:
    wanted = merge_expected() if expected is None else expected
    problems = []
    for name, target in wanted.items():
        actual = parse_param_value(text, name)
        if actual is None:
            problems.append(f"{name} missing from param show")
        elif not math.isclose(actual, target, rel_tol=0.0, abs_tol=1e-3):
            problems.append(f"{name} is {actual:g}, expected {target:g}")
    return problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params-dir", type=Path, default=None)
    parser.add_argument(
        "--override",
        action="append",
        default=[],
        help="NAME=VALUE that replaces the merged expectation. Used to prove the gate can fail.",
    )
    args = parser.parse_args(argv)
    overrides = []
    for item in args.override:
        name, separator, value = item.partition("=")
        if not separator or not name:
            print(f"Bad override {item!r}. Use NAME=VALUE.", file=sys.stderr)
            return 2
        overrides.append((name, value))
    text = sys.stdin.read()
    expected = merge_expected(directory=args.params_dir, overrides=overrides)
    problems = mismatches(text, expected)
    if problems:
        print("PX4 parameter overrides did not apply:", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1
    for name, target in expected.items():
        print(f"{name}={target:g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
