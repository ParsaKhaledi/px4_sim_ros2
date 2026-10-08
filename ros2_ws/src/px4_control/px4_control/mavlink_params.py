"""Read PX4 parameters over the SITL onboard MAVLink port.

v1.17 does not bridge a parameter-by-name topic on uXRCE. The onboard
instance listens on UDP 14580. The first packet becomes the partner,
and a later socket is not answered, so the heartbeat and the requests
share one socket.

This module does not set ``NAV_DLL_ACT`` or ``NAV_RCL_ACT``. Those belong
to the headless profile. Reboot-required EKF values come from
``PX4_PARAM_*`` before ``ekf2 start``.
"""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass

_HEARTBEAT = 0
_PARAM_REQUEST_READ = 20
_PARAM_VALUE = 22
_PARAM_SET = 23
_CRC_HEARTBEAT = 50
_CRC_REQUEST = 214
_CRC_VALUE = 220
_CRC_SET = 168
_INT32 = 6


def _x25(data: bytes, extra: int) -> int:
    crc = 0xFFFF
    for byte in data + bytes((extra,)):
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def _ident(name: str) -> bytes:
    raw = name.encode('ascii')[:16]
    return raw + bytes(16 - len(raw))


def _v2(msgid: int, payload: bytes, seq: int, crc_extra: int) -> bytes:
    # MAVLink 2 drops trailing zero bytes. The receiver fills them back in.
    trimmed = payload.rstrip(b'\x00')
    header = bytes((
        len(trimmed), 0, 0, seq & 0xFF, 255, 190,
        msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF,
    ))
    crc = _x25(header + trimmed, crc_extra)
    return bytes((0xFD,)) + header + trimmed + struct.pack('<H', crc)


def heartbeat(seq: int) -> bytes:
    """MAVLink 2 heartbeat from an onboard companion."""
    payload = struct.pack('<IBBBBB', 0, 18, 8, 0, 4, 3)
    return _v2(_HEARTBEAT, payload, seq, _CRC_HEARTBEAT)


def param_request_read(name: str, seq: int, target_system: int = 1, target_component: int = 1) -> bytes:
    """MAVLink 2 PARAM_REQUEST_READ for ``name`` (index -1)."""
    payload = struct.pack('<hBB16s', -1, target_system, target_component, _ident(name))
    return _v2(_PARAM_REQUEST_READ, payload, seq, _CRC_REQUEST)


def param_set(name: str, value: float, seq: int, target_system: int = 1, target_component: int = 1) -> bytes:
    """MAVLink 2 PARAM_SET. Integer parameters use MAV_PARAM_TYPE_INT32.

    The control node does not call this for ``NAV_DLL_ACT`` or
    ``NAV_RCL_ACT``. Those values are owned by the headless profile.
    """
    payload = struct.pack('<fBB16sB', float(value), target_system, target_component, _ident(name), _INT32)
    return _v2(_PARAM_SET, payload, seq, _CRC_SET)


@dataclass(frozen=True)
class ParamValue:
    name: str
    value: float


def _parse_param_value(payload: bytes) -> ParamValue | None:
    if len(payload) < 25:
        return None
    value, count, index = struct.unpack_from('<fHH', payload, 0)
    raw = payload[8:24]
    kind = payload[24]
    del count, index, kind
    name = raw.split(b'\x00', 1)[0].decode('ascii', errors='replace')
    return ParamValue(name, float(value))


def parse_frames(blob: bytes) -> list[ParamValue]:
    """Pull PARAM_VALUE messages out of a MAVLink 1 or 2 byte stream."""
    found: list[ParamValue] = []
    index = 0
    while index < len(blob):
        magic = blob[index]
        if magic == 0xFE and index + 8 <= len(blob):
            length = blob[index + 1]
            end = index + 6 + length + 2
            if end > len(blob):
                break
            msgid = blob[index + 5]
            payload = blob[index + 6:index + 6 + length]
            if msgid == _PARAM_VALUE:
                parsed = _parse_param_value(payload)
                if parsed is not None:
                    found.append(parsed)
            index = end
            continue
        if magic == 0xFD and index + 12 <= len(blob):
            length = blob[index + 1]
            end = index + 10 + length + 2
            if end > len(blob):
                break
            msgid = blob[index + 7] | (blob[index + 8] << 8) | (blob[index + 9] << 16)
            payload = blob[index + 10:index + 10 + length]
            if msgid == _PARAM_VALUE:
                parsed = _parse_param_value(payload)
                if parsed is not None:
                    found.append(parsed)
            index = end
            continue
        index += 1
    return found


def read_params(
    names: list[str],
    host: str = '127.0.0.1',
    port: int = 14580,
    attempts: int = 4,
    recv_timeout: float = 2.0,
    sets: dict[str, float] | None = None,
) -> dict[str, float]:
    """Ask SITL for each name. Optionally PARAM_SET some of them first.

    Everything happens on one socket. PX4 locks its UDP partner to the
    first sender and will not answer a second socket. ``recv_timeout``
    is a socket wait, not a flight timeout.
    """
    wanted = list(names)
    found: dict[str, float] = {}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(('0.0.0.0', 0))
        sock.settimeout(recv_timeout)
        seq = 0
        sock.sendto(heartbeat(seq), (host, port))
        seq = (seq + 1) & 0xFF
        if sets:
            # The partner is chosen from the first packet. Give PX4 a
            # moment to lock onto this socket before the sets.
            time.sleep(0.2)
            for name, value in sets.items():
                sock.sendto(param_set(name, value, seq), (host, port))
                seq = (seq + 1) & 0xFF
            time.sleep(0.2)
        for _attempt in range(attempts):
            pending = [name for name in wanted if name not in found]
            if not pending:
                break
            sock.sendto(heartbeat(seq), (host, port))
            seq = (seq + 1) & 0xFF
            for name in pending:
                sock.sendto(param_request_read(name, seq), (host, port))
                seq = (seq + 1) & 0xFF
            deadline = time.monotonic() + recv_timeout
            while time.monotonic() < deadline and any(name not in found for name in wanted):
                try:
                    data, _addr = sock.recvfrom(2048)
                except socket.timeout:
                    break
                for item in parse_frames(data):
                    if item.name in wanted:
                        found[item.name] = item.value
    finally:
        sock.close()
    return found
