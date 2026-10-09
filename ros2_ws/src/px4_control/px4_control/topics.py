"""PX4 uXRCE topic names for px4_msgs v1.17.0.

The names live in ``config/topics.yaml``. PX4 appends ``_v<MESSAGE_VERSION>``
when that constant is non-zero. In v1.17.0 that is ``vehicle_status_v1`` and
``vehicle_local_position_v1``. ``vehicle_odometry``, ``vehicle_attitude``, and
``vehicle_command_ack`` are version 0 and keep the bare name.
"""

from __future__ import annotations

import os
import re
from collections.abc import Iterable
from functools import lru_cache


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


def topics_config_path() -> str:
    """Share path when the package is installed, otherwise the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        shared = os.path.join(get_package_share_directory('px4_control'), 'config', 'topics.yaml')
        if os.path.isfile(shared):
            return shared
    except Exception:
        pass
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.normpath(os.path.join(here, '..', 'config', 'topics.yaml'))


def _parse_topic_file(text: str) -> dict[str, str]:
    topics: dict[str, str] = {}
    for raw in text.splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line or ':' not in line:
            continue
        key, value = line.split(':', 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and value.startswith('/'):
            topics[key] = value
    return topics


@lru_cache(maxsize=1)
def load_px4_topics(path: str | None = None) -> dict[str, str]:
    file_path = path or topics_config_path()
    with open(file_path, encoding='utf-8') as handle:
        topics = _parse_topic_file(handle.read())
    if 'vehicle_status' not in topics or 'vehicle_odometry' not in topics:
        raise ValueError(f'{file_path} is missing PX4 topic names')
    return topics


_VERSION_SUFFIX = re.compile(r'_v\d+$')


def subscription_names(configured: str) -> list[str]:
    """Configured name first, then the unversioned base when it differs."""
    names = [configured]
    head, leaf = configured.rsplit('/', 1)
    if _VERSION_SUFFIX.search(leaf):
        base = f'{head}/{_VERSION_SUFFIX.sub("", leaf)}'
        if base not in names:
            names.append(base)
    return names
