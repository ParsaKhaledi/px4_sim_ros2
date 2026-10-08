"""PX4 uXRCE topic names, including the ``_vN`` message-version suffix.

PX4 v1.17 appends ``_v<MESSAGE_VERSION>`` when that constant is non-zero.
``VehicleStatus`` in px4_msgs v1.17.0 has ``MESSAGE_VERSION = 1``, so the live
topic is ``/fmu/out/vehicle_status_v1``. Version 0 topics, including
``vehicle_visual_odometry`` and ``trajectory_setpoint``, keep the base name.
The candidates below let the node follow whichever name is actually in the graph.
"""

from __future__ import annotations

from collections.abc import Iterable


def message_version(msg_type: type) -> int:
    """Return ``MESSAGE_VERSION`` or 0 when the message is unversioned."""
    return int(getattr(msg_type, 'MESSAGE_VERSION', 0) or 0)


def version_suffix(msg_type: type) -> str:
    version = message_version(msg_type)
    if version <= 0:
        return ''
    return f'_v{version}'


def topic_candidates(base: str, msg_type: type) -> list[str]:
    """Preferred name first, then the unversioned base name when it differs."""
    suffix = version_suffix(msg_type)
    preferred = f'{base}{suffix}'
    names = [preferred]
    if base not in names:
        names.append(base)
    return names


def select_live_topic(available: Iterable[str], candidates: Iterable[str]) -> str | None:
    """First candidate that is present in the ROS graph."""
    present = set(available)
    for name in candidates:
        if name in present:
            return name
    return None
