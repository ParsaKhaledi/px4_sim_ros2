import subprocess
from pathlib import Path


INSTALLER = Path(__file__).resolve().parents[4] / 'includes' / 'gz' / 'params' / 'install_px4_control_params.bash'


def _install(rc: Path, mode: str) -> str:
    subprocess.check_call(
        ['bash', '-c', f'. "{INSTALLER}"; install_px4_control_params "{rc}" {mode}'],
    )
    return rc.read_text(encoding='utf-8')


def test_param_block_is_idempotent_and_selects_mode(tmp_path):
    rc = tmp_path / 'px4-rc.params'
    rc.write_text(
        'param set-default SYS_AUTOSTART 4001\n'
        'param set-default COM_RC_LOSS_T 35.0\n'
        'param set-default NAV_RCL_ACT 1\n',
        encoding='utf-8',
    )
    first = _install(rc, 'vision')
    second = _install(rc, 'vision')
    assert second.count('# BEGIN px4_control') == 1
    assert second.count('# END px4_control') == 1
    assert second.count('param set COM_OF_LOSS_T 1.0') == 1
    assert 'EKF2_EV_CTRL 15' in second
    assert 'EKF2_HGT_REF 3' in second
    assert 'EKF2_GPS_CTRL 5' in second
    assert 'EKF2_GPS_CTRL 0' not in second
    assert 'SYS_AUTOSTART' in second
    assert first.count('param set-default COM_RC_LOSS_T') == 0
    gps = _install(rc, 'gps')
    assert gps.count('# BEGIN px4_control') == 1
    assert 'EKF2_EV_CTRL 0' in gps
    assert 'EKF2_HGT_REF 1' in gps
    assert 'EKF2_GPS_CTRL 7' in gps
    assert 'EKF2_EV_CTRL 15' not in gps


def test_missing_params_file_is_created_and_rcs_is_hooked_once(tmp_path):
    rc = tmp_path / 'px4-rc.params'
    rcs = tmp_path / 'rcS'
    rcs.write_text(
        'commander start\n'
        '#\n'
        '# state estimator selection\n'
        'if param compare -s EKF2_EN 1\n'
        'then\n'
        '\tekf2 start &\n'
        'fi\n',
        encoding='utf-8',
    )
    subprocess.check_call(
        ['bash', '-c', f'. "{INSTALLER}"; install_px4_control_params "{rc}" gps'],
    )
    subprocess.check_call(
        ['bash', '-c', f'. "{INSTALLER}"; install_px4_control_params "{rc}" gps'],
    )
    text = rc.read_text(encoding='utf-8')
    hooked = rcs.read_text(encoding='utf-8')
    assert text.count('# BEGIN px4_control') == 1
    assert 'EKF2_GPS_CTRL 7' in text
    assert hooked.count('# BEGIN px4_control source') == 1
    assert hooked.count('. ${R}etc/init.d-posix/px4-rc.params') == 1
    assert hooked.index('# BEGIN px4_control source') < hooked.index('ekf2 start')


def test_romfs_cmake_lists_the_params_file_once(tmp_path):
    rc = tmp_path / 'px4-rc.params'
    cmake = tmp_path / 'CMakeLists.txt'
    cmake.write_text('px4_add_romfs_files(\n\trcS\n)\n', encoding='utf-8')
    subprocess.check_call(
        ['bash', '-c', f'. "{INSTALLER}"; install_px4_control_params "{rc}" vision'],
    )
    subprocess.check_call(
        ['bash', '-c', f'. "{INSTALLER}"; install_px4_control_params "{rc}" vision'],
    )
    text = cmake.read_text(encoding='utf-8')
    assert text.count('px4-rc.params') == 1
    assert text.index('px4-rc.params') < text.index('rcS')
