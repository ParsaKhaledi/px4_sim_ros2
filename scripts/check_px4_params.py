#!/usr/bin/env python3
"""Fail if `param show` does not match the headless failsafe overrides."""

from __future__ import annotations

import math
import re
import sys

# NAV_DLL_ACT 0 disables the GCS datalink preflight. NAV_RCL_ACT 1 is Hold.
# COM_RC_IN_MODE 1 is the SITL joystick default. COM_RC_LOSS_T matches the
# value the start script used to request.
EXPECTED = {
    "NAV_DLL_ACT": 0.0,
    "NAV_RCL_ACT": 1.0,
    "COM_RC_IN_MODE": 1.0,
    "COM_RC_LOSS_T": 35.0,
}


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
    wanted = EXPECTED if expected is None else expected
    problems = []
    for name, target in wanted.items():
        actual = parse_param_value(text, name)
        if actual is None:
            problems.append(f"{name} missing from param show")
        elif not math.isclose(actual, target, rel_tol=0.0, abs_tol=1e-3):
            problems.append(f"{name} is {actual:g}, expected {target:g}")
    return problems


def main() -> int:
    text = sys.stdin.read()
    problems = mismatches(text)
    if problems:
        print("PX4 parameter overrides did not apply:", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1
    for name, target in EXPECTED.items():
        print(f"{name}={target:g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
