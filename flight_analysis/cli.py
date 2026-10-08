"""Command line entry point for one ``.ulg`` file."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from e2e_limits import E2ELimitError
from e2e_limits import load_limits
from flight_analysis.analyze import analyze_flight
from flight_analysis.analyze import write_report
from flight_analysis.log import LogError
from flight_analysis.log import LogLoader
from flight_analysis.log import ULogLoader
from flight_analysis.spawn import SpawnError
from flight_analysis.tum import TimeAlignmentError


def build_parser() -> argparse.ArgumentParser:
    """Return the argument parser for the flight-analysis command."""

    parser = argparse.ArgumentParser(
        prog="flight_analysis",
        description=(
            "Grade a PX4 ULog into control metrics and plots. "
            "Limits come from the E2E_* process environment, then .env."
        ),
    )
    parser.add_argument("log", type=Path, help="PX4 .ulg flight log")
    parser.add_argument(
        "--ground-truth",
        type=Path,
        default=None,
        help="optional TUM trajectory (timestamp tx ty tz qx qy qz qw), ROS ENU",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="directory for metrics.json and plots (default logs/<run_id>/control)",
    )
    parser.add_argument(
        "--run-id",
        default=None,
        help="name used in the output path and plots (default: the log file stem)",
    )
    parser.add_argument(
        "--spawn-json",
        type=Path,
        default=None,
        help="spawn.json path (default: spawn.json in the same directory as the .ulg)",
    )
    parser.add_argument(
        "--spawn-xyz",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="override spawn_xyz, metres in Gazebo world ENU",
    )
    parser.add_argument(
        "--spawn-yaw",
        type=float,
        default=None,
        help="override spawn_yaw, radians in Gazebo world ENU",
    )
    parser.add_argument(
        "--clock-offset",
        type=float,
        default=None,
        help="override px4_offset_s: ROS sim time minus PX4 boot time, seconds",
    )
    return parser


def main(argv: list[str] | None = None, loader: LogLoader | None = None) -> int:
    """Grade one log. Returns 0 on pass, 1 when a check fails, 2 on input errors."""

    parser = build_parser()
    args = parser.parse_args(argv)
    run_id = args.run_id or args.log.stem
    output = args.output
    if output is None:
        output = Path("logs") / run_id / "control"
    try:
        limits = load_limits()
        log = (loader or ULogLoader()).load(args.log)
        report = analyze_flight(
            log,
            limits,
            tum_path=args.ground_truth,
            log_path=args.log,
            run_id=run_id,
            spawn_json=args.spawn_json,
            spawn_xyz=None if args.spawn_xyz is None else tuple(args.spawn_xyz),
            spawn_yaw=args.spawn_yaw,
            clock_offset_s=args.clock_offset,
        )
        write_report(report, output)
    except (E2ELimitError, LogError, TimeAlignmentError, SpawnError, OSError) as exc:
        print(f"flight_analysis: {exc}", file=sys.stderr)
        return 2
    passed = bool(report["passed"])
    failed = [
        check["name"]
        for check in report["checks"]
        if not check["passed"] and not check["skipped"]
    ]
    print(f"{output / 'metrics.json'}")
    if passed:
        print("passed=true")
        return 0
    print("passed=false failed=" + ",".join(failed))
    return 1
