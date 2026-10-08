"""Sim-profile PX4 parameters.

Stock PX4 v1.17 ``rcS`` applies ``PX4_PARAM_<NAME>`` from the process
environment before the airframe file and before ``ekf2 start``. That is
the path this stack uses. Appending ``param set`` lines to ``px4-rc.params``
does nothing: v1.17 never sources that file.

``NAV_DLL_ACT`` is 0 only in this sim profile. The x500 airframe default
is 2, which refuses to arm when no QGroundControl heartbeat is present.
"""

from __future__ import annotations

# Read back after PX4 is up. A mismatch means the running estimator or
# the arming check is still on the airframe default.
READBACK_PARAMS = (
    'EKF2_EV_CTRL',
    'EKF2_GPS_CTRL',
    'EKF2_HGT_REF',
    'EKF2_MAG_TYPE',
    'SYS_HAS_MAG',
    'NAV_RCL_ACT',
    'NAV_DLL_ACT',
    'UXRCE_DDS_SYNCT',
)


def expected_sim_params(
    mode: str,
    ev_ctrl: float = 11.0,
    ev_delay: float = 50.0,
) -> dict[str, float]:
    """Parameter values the SITL process must show for ``mode``."""
    name = mode.strip().lower()
    if name not in ('vision', 'gps'):
        raise ValueError(f'estimation mode must be vision or gps, got {mode!r}')
    values = {
        'COM_OF_LOSS_T': 1.0,
        'COM_OBL_RC_ACT': 4.0,
        'COM_RC_LOSS_T': 35.0,
        'NAV_RCL_ACT': 1.0,
        'NAV_DLL_ACT': 0.0,
        'UXRCE_DDS_SYNCT': 0.0,
    }
    if name == 'gps':
        values.update({
            'EKF2_EV_CTRL': 0.0,
            'EKF2_HGT_REF': 1.0,
            'EKF2_GPS_CTRL': 7.0,
            'EKF2_GPS_P_NOISE': 0.5,
            'EKF2_GPS_V_NOISE': 0.3,
            # Firmware default. Automatic. The gps profile does not export it.
            'EKF2_MAG_TYPE': 0.0,
            # Firmware default. Neither profile exports it.
            'SYS_HAS_MAG': 1.0,
        })
    else:
        values.update({
            'EKF2_EV_CTRL': float(ev_ctrl),
            # None. common.h MagFuseType::NONE. Yaw comes from external vision.
            'EKF2_MAG_TYPE': 5.0,
            # Firmware default. The compass stays present; fusion is EKF2_MAG_TYPE.
            'SYS_HAS_MAG': 1.0,
            'EKF2_HGT_REF': 3.0,
            'EKF2_EV_DELAY': float(ev_delay),
            'EKF2_EV_NOISE_MD': 0.0,
            'EKF2_GPS_CTRL': 5.0,
            'EKF2_GPS_P_NOISE': 5.0,
            'EKF2_GPS_V_NOISE': 1.0,
            'COM_ARM_WO_GPS': 1.0,
        })
    return values


def params_match(actual: float, expected: float) -> bool:
    return abs(float(actual) - float(expected)) <= 1e-3


def readback_mismatch(actual: dict[str, float], expected: dict[str, float]) -> str | None:
    """Empty when every read-back parameter matches, otherwise one error line."""
    parts: list[str] = []
    for name in READBACK_PARAMS:
        if name not in actual:
            parts.append(f'{name} was not read back')
            continue
        want = expected[name]
        got = actual[name]
        if not params_match(got, want):
            parts.append(f'{name}={got:g} expected {want:g}')
    if not parts:
        return None
    return (
        'PX4 parameters did not take effect: '
        + '; '.join(parts)
        + '. Stock rcS applies PX4_PARAM_* before ekf2 start. '
        + 'px4-rc.params is not sourced in v1.17.'
    )
