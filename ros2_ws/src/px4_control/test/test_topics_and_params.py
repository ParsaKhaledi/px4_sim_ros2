import socket
import threading

from px4_control.mavlink_params import param_request_read, parse_frames, read_params
from px4_control.px4_params import expected_sim_params, readback_mismatch
from px4_control.topics import load_px4_topics, subscription_names


def test_topic_config_uses_v1_only_where_px4_msgs_does():
    topics = load_px4_topics()
    assert topics['vehicle_status'] == '/fmu/out/vehicle_status_v1'
    assert topics['vehicle_local_position'] == '/fmu/out/vehicle_local_position_v1'
    assert topics['vehicle_odometry'] == '/fmu/out/vehicle_odometry'
    assert topics['vehicle_attitude'] == '/fmu/out/vehicle_attitude'
    assert topics['vehicle_command_ack'] == '/fmu/out/vehicle_command_ack'
    assert topics['failsafe_flags'] == '/fmu/out/failsafe_flags'
    assert topics['vehicle_land_detected'] == '/fmu/out/vehicle_land_detected'
    assert topics['vehicle_visual_odometry'] == '/fmu/in/vehicle_visual_odometry'
    assert subscription_names(topics['vehicle_status']) == [
        '/fmu/out/vehicle_status_v1',
        '/fmu/out/vehicle_status',
    ]
    assert subscription_names(topics['vehicle_odometry']) == ['/fmu/out/vehicle_odometry']


def test_readback_fails_when_ekf_params_stay_at_the_airframe_default():
    expected = expected_sim_params('vision')
    assert expected['NAV_DLL_ACT'] == 0.0
    assert expected['EKF2_EV_CTRL'] == 11.0
    actual = {
        'EKF2_EV_CTRL': 0.0,
        'EKF2_GPS_CTRL': 7.0,
        'EKF2_HGT_REF': 1.0,
        'NAV_RCL_ACT': 2.0,
        'NAV_DLL_ACT': 2.0,
        'UXRCE_DDS_SYNCT': 1.0,
    }
    message = readback_mismatch(actual, expected)
    assert message is not None
    assert 'EKF2_EV_CTRL=0' in message
    assert 'NAV_RCL_ACT=2' in message
    assert 'NAV_DLL_ACT=2' in message
    assert readback_mismatch(
        {name: expected[name] for name in (
            'EKF2_EV_CTRL', 'EKF2_GPS_CTRL', 'EKF2_HGT_REF',
            'NAV_RCL_ACT', 'NAV_DLL_ACT', 'UXRCE_DDS_SYNCT',
        )},
        expected,
    ) is None


def test_param_request_round_trip(tmp_path):
    del tmp_path
    packet = bytes.fromhex(
        'fe1902ffbe160000304164000700454b46325f45565f4354524c00000000065c84'
    )
    parsed = parse_frames(packet)
    assert parsed[0].name == 'EKF2_EV_CTRL'
    assert parsed[0].value == 11.0

    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    port = server.getsockname()[1]
    stop = threading.Event()

    def serve():
        while not stop.is_set():
            server.settimeout(0.2)
            try:
                _data, addr = server.recvfrom(2048)
            except socket.timeout:
                continue
            server.sendto(packet, addr)

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        values = read_params(['EKF2_EV_CTRL'], host='127.0.0.1', port=port, attempts=2, recv_timeout=0.5)
    finally:
        stop.set()
        thread.join(timeout=1.0)
        server.close()
    assert values['EKF2_EV_CTRL'] == 11.0
    assert param_request_read('EKF2_EV_CTRL', 0).startswith(bytes((0xFD,)))


def test_vision_timeout_default_covers_three_cpu_frames():
    text = (
        __import__('pathlib').Path(__file__).resolve().parents[1] / 'config' / 'px4_control.yaml'
    ).read_text(encoding='utf-8')
    line = next(item for item in text.splitlines() if item.strip().startswith('vision_timeout_s:'))
    value = float(line.split(':', 1)[1])
    assert value >= 0.3
