"""Read PX4 parameters over the SITL MAVLink UDP port.

v1.17 does not bridge a parameter-by-name topic on uXRCE. The SITL
GCS port (18570) answers PARAM_REQUEST_READ. This module speaks just
enough MAVLink 1 to ask and to parse the PARAM_VALUE reply. It does
not set parameters: reboot-required EKF values have to be in the
process environment before ekf2 starts.
"""

from __future__ import annotations

import socket
import struct
from dataclasses import dataclass

_PARAM_REQUEST_READ = 20
_PARAM_VALUE = 22
_CRC_REQUEST = 214
_CRC_VALUE = 220


def _x25(data: bytes, extra: int) -> int:
    crc = 0xFFFF
    for byte in data + bytes((extra,)):
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def param_request_read(name: str, seq: int, target_system: int = 1, target_component: int = 1) -> bytes:
    """MAVLink 1 PARAM_REQUEST_READ for ``name`` (index -1)."""
    ident = name.encode('ascii')[:16]
    ident = ident + bytes(16 - len(ident))
    payload = struct.pack('<hBB16s', -1, target_system, target_component, ident)
    header = bytes((len(payload), seq & 0xFF, 255, 190, _PARAM_REQUEST_READ))
    crc = _x25(header + payload, _CRC_REQUEST)
    return bytes((0xFE,)) + header + payload + struct.pack('<H', crc)


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
    port: int = 18570,
    attempts: int = 4,
    recv_timeout: float = 2.0,
) -> dict[str, float]:
    """Ask SITL for each name. Missing names are absent from the result.

    ``recv_timeout`` is a socket wait, not a flight timeout. PX4 answers
    on the wall clock of the MAVLink thread.
    """
    wanted = list(names)
    found: dict[str, float] = {}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(('0.0.0.0', 0))
        sock.settimeout(recv_timeout)
        seq = 0
        for _attempt in range(attempts):
            pending = [name for name in wanted if name not in found]
            if not pending:
                break
            for name in pending:
                sock.sendto(param_request_read(name, seq), (host, port))
                seq = (seq + 1) & 0xFF
            # One reply window per attempt, long enough for every name.
            for _ in range(len(pending) * 3):
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
