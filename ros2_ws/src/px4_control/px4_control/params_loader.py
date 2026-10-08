"""Runtime parameter normalisation.

``ESTIMATION_MODE`` is the env key DevOps should pass into the PX4 container.
``vision`` (default) fuses external vision as the primary aid and keeps GPS
publishing. ``gps`` is classic SITL GPS fusion with external vision off.
The matching PX4 parameters are applied by
``includes/gz/params/install_px4_control_params.bash`` before EKF2 starts.
"""

from __future__ import annotations

import os

ESTIMATION_ENV = 'ESTIMATION_MODE'
YAW_RATE_ENV = 'PX4_MAX_YAW_RATE_DEG_S'
WALL_FILE_ENV = 'PX4_WALL_SEGMENTS_FILE'


def normalize_estimation_mode(value: str | None) -> str:
    """Return ``vision`` or ``gps``. Empty means the default, ``vision``."""
    if value is None or str(value).strip() == '':
        return 'vision'
    mode = str(value).strip().lower()
    if mode not in ('vision', 'gps'):
        raise ValueError(f'estimation_mode must be vision or gps, got {value!r}')
    return mode


def estimation_mode_from_env(environ: dict[str, str] | None = None) -> str:
    source = os.environ if environ is None else environ
    return normalize_estimation_mode(source.get(ESTIMATION_ENV, 'vision'))


def yaw_rate_from_env(environ: dict[str, str] | None = None, default: float = 30.0) -> float:
    """Max yaw rate in degrees per second. Env ``PX4_MAX_YAW_RATE_DEG_S``."""
    source = os.environ if environ is None else environ
    raw = source.get(YAW_RATE_ENV, '').strip()
    if raw == '':
        rate = default
    else:
        rate = float(raw)
    if rate <= 0.0:
        raise ValueError(f'{YAW_RATE_ENV} must be positive, got {rate}')
    return rate
