"""Whether a GPU is visible to this process.

A render node under ``/dev/dri`` or a working ``nvidia-smi`` counts.
"""

from __future__ import annotations

import subprocess
from pathlib import Path


def _nvidia_smi_ok() -> bool:
    """True when ``nvidia-smi`` runs and exits 0."""
    try:
        completed = subprocess.run(
            ["nvidia-smi"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return completed.returncode == 0


def gpu_visible(dri_nodes: list[str] | None = None, nvidia_ok: bool | None = None) -> bool:
    """A render node under ``/dev/dri`` or a working ``nvidia-smi`` counts."""
    if dri_nodes is None:
        dri = Path("/dev/dri")
        dri_nodes = sorted(str(path) for path in dri.glob("renderD*")) if dri.is_dir() else []
    if nvidia_ok is None:
        nvidia_ok = _nvidia_smi_ok()
    return bool(dri_nodes) or bool(nvidia_ok)
