#!/usr/bin/env python3
"""Compare two TUM trajectories with evo, SE(3) alignment only.

``evo_ape`` and ``evo_rpe`` are called with ``-a`` and never with ``-as`` or
``--correct_scale``, so a scale error stays visible. Pass/fail uses
SLAM_APE_RMS_MAX (metres, APE RMSE) and SLAM_DRIFT_PER_M_MAX (metres of
translational RPE over a 1 m segment). Values come from the environment, then
from a ``.env`` file, then from the defaults below.

Exit 0 on pass, 1 when a threshold fails, 2 when evo is missing or its output
cannot be read. The two trajectory arguments are any TUM files; nothing else
in the repo has to exist.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path


DEFAULT_APE_RMS_MAX = 0.50
DEFAULT_DRIFT_PER_M_MAX = 0.10
BANNED_ALIGNMENT = {"-as", "-s", "--correct_scale"}


def ape_command(reference: str, estimate: str) -> list[str]:
    return ["evo_ape", "tum", reference, estimate, "-a"]


def rpe_command(reference: str, estimate: str) -> list[str]:
    # One-metre segments, translation only. -a is SE(3); scale is not freed.
    return [
        "evo_rpe",
        "tum",
        reference,
        estimate,
        "-a",
        "--delta",
        "1",
        "--delta_unit",
        "m",
        "--pose_relation",
        "trans_part",
    ]


def se3_alignment_only(command: list[str]) -> bool:
    tokens = set(command)
    return "-a" in tokens and tokens.isdisjoint(BANNED_ALIGNMENT)


def parse_evo_rmse(text: str) -> float:
    for line in text.splitlines():
        parts = line.replace("\t", " ").split()
        if parts and parts[0].lower() == "rmse":
            return float(parts[-1])
    raise ValueError("evo output has no rmse row")


def load_env_file(path: Path) -> None:
    """Fill unset variables from a .env file. Existing environment wins."""
    if not path.is_file():
        return
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        if key.startswith("export "):
            key = key[len("export ") :].strip()
        value = value.strip().strip('"').strip("'")
        os.environ.setdefault(key, value)


def load_repo_env() -> None:
    candidates = [Path.cwd() / ".env", Path(__file__).resolve().parents[1] / ".env"]
    for path in candidates:
        load_env_file(path)


def threshold(name: str, default: float) -> float:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    return float(raw)


def passes(ape_rms: float, drift_per_m: float, ape_max: float, drift_max: float) -> bool:
    return ape_rms <= ape_max and drift_per_m <= drift_max


def run_evo(command: list[str]) -> float:
    if not se3_alignment_only(command):
        raise RuntimeError(f"refusing alignment that hides scale: {command}")
    if shutil.which(command[0]) is None:
        raise FileNotFoundError(command[0])
    completed = subprocess.run(command, check=False, text=True, capture_output=True)
    output = (completed.stdout or "") + "\n" + (completed.stderr or "")
    if completed.returncode != 0:
        raise RuntimeError(output.strip() or f"{command[0]} exited {completed.returncode}")
    return parse_evo_rmse(output)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="APE/RPE gate for two TUM trajectories.")
    parser.add_argument("reference", help="Ground-truth TUM file")
    parser.add_argument("estimate", help="Estimated TUM file")
    parser.add_argument("--json-out", default="")
    args = parser.parse_args(argv)

    load_repo_env()
    ape_max = threshold("SLAM_APE_RMS_MAX", DEFAULT_APE_RMS_MAX)
    drift_max = threshold("SLAM_DRIFT_PER_M_MAX", DEFAULT_DRIFT_PER_M_MAX)
    reference = str(Path(args.reference))
    estimate = str(Path(args.estimate))

    try:
        ape_rms = run_evo(ape_command(reference, estimate))
        drift_per_m = run_evo(rpe_command(reference, estimate))
    except FileNotFoundError as exc:
        print(f"evo is not installed ({exc})", file=sys.stderr)
        return 2
    except (RuntimeError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2

    ok = passes(ape_rms, drift_per_m, ape_max, drift_max)
    result = {
        "ape_rms_m": ape_rms,
        "drift_per_m": drift_per_m,
        "ape_rms_max": ape_max,
        "drift_per_m_max": drift_max,
        "alignment": "se3",
        "pass": ok,
    }
    text = json.dumps(result, sort_keys=True)
    print(text)
    if args.json_out:
        Path(args.json_out).write_text(text + "\n", encoding="utf-8")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
