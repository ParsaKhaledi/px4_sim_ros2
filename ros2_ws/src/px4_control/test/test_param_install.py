import subprocess
from pathlib import Path

import pytest


INSTALLER = Path(__file__).resolve().parents[4] / 'includes' / 'gz' / 'params' / 'install_px4_control_params.bash'


def _install(env_file: Path, mode: str) -> str:
    subprocess.check_call(
        ['bash', '-c', f'. "{INSTALLER}"; install_px4_control_params "{env_file}" {mode}'],
    )
    return env_file.read_text(encoding='utf-8')


def test_param_env_is_idempotent_and_selects_mode(tmp_path):
    env_file = tmp_path / 'px4_control_params.env'
    first = _install(env_file, 'vision')
    second = _install(env_file, 'vision')
    assert second == first
    assert 'export PX4_PARAM_EKF2_EV_CTRL=11' in second
    assert 'export PX4_PARAM_EKF2_MAG_TYPE=5' in second
    assert 'SYS_HAS_MAG' not in second
    assert 'export PX4_PARAM_EKF2_HGT_REF=3' in second
    assert 'export PX4_PARAM_EKF2_EV_DELAY=0' in second
    assert 'export PX4_PARAM_EKF2_GPS_CTRL=0' in second
    assert 'EKF2_GPS_P_NOISE' not in second
    assert 'EKF2_GPS_V_NOISE' not in second
    assert 'export PX4_PARAM_UXRCE_DDS_SYNCT=0' in second
    assert 'export PX4_PARAM_NAV_RCL_ACT=1' in second
    assert 'export PX4_PARAM_NAV_DLL_ACT=0' in second
    assert 'param set' not in second
    assert second.count('export PX4_PARAM_EKF2_EV_CTRL=') == 1
    gps = _install(env_file, 'gps')
    assert 'export PX4_PARAM_EKF2_EV_CTRL=0' in gps
    assert 'export PX4_PARAM_EKF2_HGT_REF=1' in gps
    assert 'export PX4_PARAM_EKF2_GPS_CTRL=7' in gps
    assert 'export PX4_PARAM_EKF2_GPS_P_NOISE=0.5' in gps
    assert 'export PX4_PARAM_EKF2_GPS_V_NOISE=0.3' in gps
    assert 'export PX4_PARAM_NAV_DLL_ACT=0' in gps
    assert 'export PX4_PARAM_EKF2_EV_CTRL=11' not in gps
    assert 'EKF2_MAG_TYPE' not in gps
    assert 'SYS_HAS_MAG' not in gps
    assert 'EKF2_EV_DELAY' not in gps


def test_vision_delay_and_ctrl_come_from_the_environment(tmp_path, monkeypatch):
    env_file = tmp_path / 'px4_control_params.env'
    monkeypatch.setenv('EKF2_EV_DELAY', '80')
    monkeypatch.setenv('EKF2_EV_CTRL', '15')
    text = _install(env_file, 'vision')
    assert 'export PX4_PARAM_EKF2_EV_DELAY=80' in text
    assert 'export PX4_PARAM_EKF2_EV_CTRL=15' in text
    monkeypatch.setenv('EKF2_EV_CTRL', '16')
    with pytest.raises(subprocess.CalledProcessError):
        _install(env_file, 'vision')


def test_start_script_sources_the_env_file():
    script = Path(__file__).resolve().parents[4] / 'includes' / 'gz' / 'startFiles' / 'gz_start_px4_gz_sim.sh'
    text = script.read_text(encoding='utf-8')
    assert 'px4_control_params.env' in text
    assert 'PX4_PARAM_' not in text or '. "${PARAM_ENV}"' in text
