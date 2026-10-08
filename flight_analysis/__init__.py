"""Offline grading of a PX4 SITL flight log.

Run ``python -m flight_analysis`` from the repository root. The shared
limit loader lives next to this package so the in-flight tests can import
it without pulling in pyulog or matplotlib.
"""

from __future__ import annotations

import sys
from pathlib import Path

# e2e_limits.py sits at the repo root, beside this package.
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

__all__ = ["__version__"]

__version__ = "1.0.0"
