"""Camera selection for the out-and-back flight.

The same name drives the Gazebo model and the RTAB-Map launch. The flight
test reads ``--camera-type`` or the ``CameraType`` environment variable so
the next run can be stereo or rgbd without editing the mission.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

CAMERA_TYPES = ('stereo', 'rgbd')


def parse_camera_type(argv: Sequence[str], environ: Mapping[str, str]) -> str:
    """Return ``stereo`` or ``rgbd``.

    ``--camera-type stereo`` and ``--camera-type=stereo`` win, then a bare
    positional argument, then ``CameraType``.
    """
    value: str | None = None
    args = list(argv)
    index = 0
    while index < len(args):
        item = args[index]
        if item == '--camera-type':
            if index + 1 >= len(args):
                raise ValueError('CameraType needs a value: stereo or rgbd')
            value = args[index + 1]
            index += 2
            continue
        if item.startswith('--camera-type='):
            value = item.split('=', 1)[1]
            index += 1
            continue
        if value is None and item and not item.startswith('-'):
            value = item
        index += 1
    if value is None:
        raw = environ.get('CameraType', '')
        value = raw if str(raw).strip() else None
    if value is None:
        raise ValueError('CameraType is required: stereo or rgbd')
    name = str(value).strip().lower()
    if name not in CAMERA_TYPES:
        raise ValueError(f'CameraType must be stereo or rgbd, got {value!r}')
    return name
