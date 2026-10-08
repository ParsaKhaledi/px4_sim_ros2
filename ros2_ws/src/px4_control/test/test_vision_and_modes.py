import numpy as np
import pytest

from px4_control.arming import ACK_TEMPORARILY_REJECTED, ack_failure_text, prearm_block_reason, sim_preflight_decision
from px4_control.mode_switch import OffboardStreamGate, parse_mode
from px4_control.params_loader import normalize_estimation_mode
from px4_control.topics import select_live_topic, topic_candidates
from px4_control.vision_bridge import OdomSample, VisionOdometryBridge


class _Msg:
    MESSAGE_VERSION = 1


class _Plain:
    MESSAGE_VERSION = 0


def test_versioned_topic_candidates_prefer_the_suffix():
    assert topic_candidates('/fmu/out/vehicle_status', _Msg) == [
        '/fmu/out/vehicle_status_v1',
        '/fmu/out/vehicle_status',
    ]
    assert topic_candidates('/fmu/in/vehicle_visual_odometry', _Plain) == [
        '/fmu/in/vehicle_visual_odometry',
    ]
    available = {'/fmu/out/vehicle_status', '/clock'}
    # The unversioned name is the fallback when the versioned topic is absent.
    assert select_live_topic(available, topic_candidates('/fmu/out/vehicle_status', _Msg)) == (
        '/fmu/out/vehicle_status'
    )
    available.add('/fmu/out/vehicle_status_v1')
    assert select_live_topic(available, topic_candidates('/fmu/out/vehicle_status', _Msg)) == (
        '/fmu/out/vehicle_status_v1'
    )


def test_offboard_burst_is_not_a_warm_stream():
    gate = OffboardStreamGate(min_hz=2.0, warmup_s=1.0)
    for _ in range(100):
        gate.tick(5.0)
    assert gate.ready(5.0) is False
    for index in range(30):
        gate.tick(5.0 + index * 0.05)
    assert gate.ready(5.0 + 29 * 0.05) is True


def test_parse_mode_offboard():
    mode = parse_mode('offboard')
    assert mode.param2 == 6.0
    assert mode.nav_state == 14
    with pytest.raises(ValueError):
        parse_mode('nope')


def test_estimation_mode_names():
    assert normalize_estimation_mode(None) == 'vision'
    assert normalize_estimation_mode('GPS') == 'gps'
    with pytest.raises(ValueError):
        normalize_estimation_mode('vio')


def test_temporary_reject_is_not_a_final_ack():
    assert ack_failure_text(ACK_TEMPORARILY_REJECTED) is None
    assert ack_failure_text(2) is not None


def test_prearm_reports_the_flag_and_ignores_healthy_vehicles():
    class Status:
        pre_flight_checks_pass = False
        failsafe = False

    class Flags:
        local_position_invalid = True
        manual_control_signal_lost = True
        battery_warning = 0

    reason = prearm_block_reason(Status(), Flags())
    assert 'local_position_invalid' in reason
    assert 'pre_flight_checks_pass is false' in reason

    class GcsFlags:
        gcs_connection_lost = True
        battery_warning = 0

    gcs = prearm_block_reason(Status(), GcsFlags())
    assert gcs is not None
    assert 'No connection to the GCS' in gcs
    assert 'NAV_DLL_ACT' in gcs

    class Healthy:
        pre_flight_checks_pass = True
        failsafe = False

    assert prearm_block_reason(Healthy(), Flags()) is None
    assert prearm_block_reason(None, None) == 'no vehicle_status received'


def test_sim_preflight_failure_is_returned_verbatim():
    assert sim_preflight_decision(False, True, False, 'nope') is None
    decision = sim_preflight_decision(True, True, False, 'battery disconnected')
    assert decision is not None
    assert decision.success is False
    assert decision.message == 'battery disconnected'
    assert sim_preflight_decision(True, True, True, 'ok') is None


def _sample(stamp, position, lost=False, covariance=0.01):
    pose = np.zeros(36)
    twist = np.zeros(36)
    for index in (0, 7, 14, 21, 28, 35):
        pose[index] = covariance
        twist[index] = covariance
    return OdomSample(
        stamp_sec=stamp,
        position_enu=np.asarray(position, dtype=float),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        linear_flu=np.zeros(3),
        angular_flu=np.zeros(3),
        pose_covariance=pose,
        twist_covariance=twist,
        tracking_lost=lost,
    )


def test_vision_covariance_is_positive_and_reset_bumps_on_a_jump():
    bridge = VisionOdometryBridge(timeout_s=0.3, reset_jump_m=0.5, max_variance=25.0)
    first = bridge.push(_sample(1.0, (1.0, 2.0, 3.0)))
    assert first is not None
    assert np.all(first.position_variance > 0.0)
    assert first.position_ned == pytest.approx(np.array([2.0, 1.0, -3.0]))
    assert first.reset_counter == 0
    assert first.stamp_sec == pytest.approx(1.0)
    jumped = bridge.push(_sample(1.1, (8.0, 2.0, 3.0)))
    assert jumped is not None
    assert jumped.reset_counter == 1
    assert bridge.push(_sample(1.2, (8.0, 2.0, 3.0), lost=True)) is None
    assert bridge.current(1.25) is None
    recovered = bridge.push(_sample(1.5, (8.0, 2.0, 3.0)))
    assert recovered is not None
    assert recovered.reset_counter == 2
    assert recovered.stamp_sec == pytest.approx(1.5)


def test_vision_staleness_follows_the_caller_clock():
    bridge = VisionOdometryBridge(timeout_s=0.3)
    assert bridge.push(_sample(10.0, (0.0, 0.0, 1.0))) is not None
    assert bridge.current(10.2) is not None
    assert bridge.current(10.4) is None


def test_vision_does_not_restamp_a_stale_pose():
    bridge = VisionOdometryBridge(timeout_s=0.2)
    sample = bridge.push(_sample(2.0, (0.0, 0.0, 1.0)))
    assert sample is not None
    current = bridge.current(2.1)
    assert current is not None
    assert current.stamp_sec == pytest.approx(2.0)
    assert bridge.current(2.3) is None


def test_negative_vision_variance_is_tracking_loss():
    bridge = VisionOdometryBridge()
    assert bridge.push(_sample(1.0, (0.0, 0.0, 1.0), covariance=-0.1)) is None
