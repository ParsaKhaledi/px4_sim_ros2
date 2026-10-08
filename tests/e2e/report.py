#!/usr/bin/env python3
"""Merge per-attempt flight JSON into one trajectory report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def crash_reset_count(attempts: list[dict]) -> int:
    """How many times a crash was followed by a sim restart.

    The last attempt is not followed by a restart, even when it crashed.
    """

    if len(attempts) < 2:
        return 0
    resets = 0
    for attempt in attempts[:-1]:
        if attempt.get("status") == "crashed" or attempt.get("crash_reason"):
            resets += 1
    return resets


def build_report(attempts: list[dict]) -> dict:
    last = attempts[-1] if attempts else {}
    passed = bool(last.get("passed")) and last.get("status") == "passed"
    reasons = []
    for attempt in attempts:
        reason = attempt.get("crash_reason")
        if reason:
            reasons.append(
                {
                    "attempt": attempt.get("attempt"),
                    "crash_reason": reason,
                }
            )
    return {
        "status": "passed" if passed else "failed",
        "passed": passed,
        "driver": last.get("driver"),
        "checks": last.get("checks", []),
        "px4_position_error_m": last.get("px4_position_error_m"),
        "gz_rtf": last.get("gz_rtf"),
        "cameras": last.get("cameras"),
        "camera_reason": last.get("camera_reason"),
        "thresholds": last.get("thresholds"),
        "attempts": attempts,
        "crash_resets": crash_reset_count(attempts),
        "crash_reasons": reasons,
    }


def load_attempts(directory: Path) -> list[dict]:
    rows = []
    paths = sorted(directory.glob("attempt-*.json"))
    for path in paths:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            payload = {"status": "failed", "reason": f"bad json in {path.name}"}
        stem = path.stem
        number = stem.split("-", 1)[-1]
        if "attempt" not in payload:
            payload["attempt"] = int(number) if number.isdigit() else number
        rows.append(payload)
    rows.sort(key=lambda row: int(row["attempt"]) if str(row["attempt"]).isdigit() else 0)
    return rows


def write_report(directory: Path, out: Path) -> dict:
    report = build_report(load_attempts(directory))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def write_stub(path: Path, exit_code: int, attempt: int) -> None:
    """Record an attempt that died before it could write its own JSON."""

    crashed = exit_code in {2, 134, 137, 139}
    reason = f"attempt exited {exit_code}"
    payload = {
        "attempt": attempt,
        "status": "crashed" if crashed else "failed",
        "passed": False,
        "exit_code": exit_code,
        "reason": reason,
    }
    if crashed:
        payload["crash_reason"] = reason
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def needs_render_fallback(attempts: list[dict]) -> bool:
    """True when a finished attempt saw blank or missing camera frames.

    A crashed attempt is left to the crash restart. Switching renderers
    is for a flight that completed with a bad image.
    """

    if not attempts:
        return False
    last = attempts[-1]
    if last.get("status") == "crashed" or last.get("crash_reason"):
        return False
    reason = last.get("camera_reason")
    if reason in {"blank_camera", "no_camera_frames"}:
        return True
    cameras = last.get("cameras") or {}
    return cameras.get("reason") in {"blank_camera", "no_camera_frames"}


def main() -> int:
    parser = argparse.ArgumentParser(description="Write the flight summary JSON")
    parser.add_argument("--attempts-dir", default="")
    parser.add_argument("--out", default="")
    parser.add_argument("--stub", action="store_true")
    parser.add_argument("--should-retry", action="store_true")
    parser.add_argument("--needs-render-fallback", action="store_true")
    parser.add_argument("--exit-code", type=int, default=0)
    parser.add_argument("--attempt", type=int, default=1)
    parser.add_argument("--max-retries", type=int, default=2)
    args = parser.parse_args()
    if args.should_retry:
        from crash_monitor import should_retry

        return 0 if should_retry(args.exit_code, args.attempt, args.max_retries) else 1
    if args.needs_render_fallback:
        if not args.attempts_dir:
            raise SystemExit("fallback check needs --attempts-dir")
        return 0 if needs_render_fallback(load_attempts(Path(args.attempts_dir))) else 1
    if args.stub:
        if not args.out:
            raise SystemExit("stub mode needs --out")
        write_stub(Path(args.out), args.exit_code, args.attempt)
        return 0
    if not args.attempts_dir or not args.out:
        raise SystemExit("summary mode needs --attempts-dir and --out")
    report = write_report(Path(args.attempts_dir), Path(args.out))
    print(json.dumps({
        "status": report["status"],
        "passed": report["passed"],
        "crash_resets": report["crash_resets"],
        "crash_reasons": report["crash_reasons"],
        "checks": report.get("checks"),
        "px4_position_error_m": report.get("px4_position_error_m"),
        "gz_rtf": report.get("gz_rtf"),
        "cameras": report.get("cameras"),
    }, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
