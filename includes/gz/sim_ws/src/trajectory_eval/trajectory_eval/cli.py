"""Command line for trajectory_eval.

Examples
--------
Record a live sim (use_sim_time) for 20 seconds::

    python3 -m trajectory_eval record --duration 20 --output /tmp/traj

Score an existing rosbag::

    python3 -m trajectory_eval bag --bag /path/to/bag --output /tmp/traj

Score TUM files that are already in ENU::

    python3 -m trajectory_eval offline --ground-truth gt.tum --rtabmap rtab.tum --output /tmp/traj
"""

from __future__ import annotations

import argparse
from pathlib import Path

from trajectory_eval.report import write_report
from trajectory_eval.tum import read_tum


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="trajectory_eval", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    record = sub.add_parser("record", help="subscribe to live topics and write a report")
    _add_topic_args(record)
    record.add_argument("--duration", type=float, default=0.0, help="seconds of sim time; 0 waits for Ctrl-C")
    record.add_argument("--output", type=Path, required=True)
    record.add_argument("--use-sim-time", action=argparse.BooleanOptionalAction, default=True)

    bag = sub.add_parser("bag", help="read a rosbag2 directory and write a report")
    _add_topic_args(bag)
    bag.add_argument("--bag", type=Path, required=True)
    bag.add_argument("--storage", default="sqlite3")
    bag.add_argument("--output", type=Path, required=True)

    offline = sub.add_parser("offline", help="score TUM files that are already in ENU")
    offline.add_argument("--ground-truth", type=Path, required=True)
    offline.add_argument("--gps", type=Path)
    offline.add_argument("--rtabmap", type=Path)
    offline.add_argument("--ekf", type=Path)
    offline.add_argument("--output", type=Path, required=True)
    offline.add_argument("--rpe", type=float, nargs="+", default=[1.0, 5.0])
    return parser


def _add_topic_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--gt-topic", default="/ground_truth/odom")
    parser.add_argument("--gps-topic", default="/fmu/out/vehicle_gps_position")
    parser.add_argument("--rtabmap-topic", default="/rtabmap/odom")
    parser.add_argument("--ekf-topic", default="/fmu/out/vehicle_odometry")
    parser.add_argument("--rpe", type=float, nargs="+", default=[1.0, 5.0])


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    distances = tuple(args.rpe)
    if args.command == "offline":
        streams = {"ground_truth": read_tum(args.ground_truth)}
        if args.gps:
            streams["gps"] = read_tum(args.gps)
        if args.rtabmap:
            streams["rtabmap"] = read_tum(args.rtabmap)
        if args.ekf:
            streams["ekf2"] = read_tum(args.ekf)
        document = write_report(args.output, streams, distances)
        _print_summary(document)
        return 0
    if args.command == "record":
        from trajectory_eval.record import record_live
        streams = record_live(args)
        document = write_report(args.output, streams, distances)
        _print_summary(document)
        return 0
    if args.command == "bag":
        from trajectory_eval.bag import read_bag
        streams = read_bag(args)
        document = write_report(args.output, streams, distances)
        _print_summary(document)
        return 0
    return 2


def _print_summary(document: dict) -> None:
    print("trajectory_eval")
    for name, entry in document.get("trajectories", {}).items():
        if not entry.get("available"):
            print(f"  {name}: not available")
            continue
        ate = entry["ate"]["unaligned_relative_to_takeoff"]
        aligned = entry["ate"].get("aligned_se3") or {}
        print(
            f"  {name}: ATE unaligned rmse {ate['rmse']:.3f} m, "
            f"aligned rmse {aligned.get('rmse', float('nan')):.3f} m, "
            f"final {entry['final_position_error']['unaligned_relative_to_takeoff_m']:.3f} m"
        )


if __name__ == "__main__":
    raise SystemExit(main())
