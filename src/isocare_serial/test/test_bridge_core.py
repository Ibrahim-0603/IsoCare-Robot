"""Tests for isocare_serial.bridge_core. Plain pytest, no ROS needed."""

import math
from pathlib import Path

from isocare_serial import bridge_core
from isocare_serial import codec
import pytest

MSG_DIR = Path(__file__).resolve().parents[2] / 'isocare_interfaces' / 'msg'


# --------------------------------------------------------------------------
# twist_to_wheels
# --------------------------------------------------------------------------

def test_straight_line():
    """Going 0.2 m/s forward is 200 mm/s on both wheels."""
    assert bridge_core.twist_to_wheels(0.2, 0.0, 150.0, 500.0) == (200, 200)


def test_spin_in_place_counter_clockwise_is_right_wheel_faster():
    """Positive angular speed spins counter-clockwise: the right wheel goes forward."""
    assert bridge_core.twist_to_wheels(0.0, 1.0, 150.0, 500.0) == (-75, 75)
    assert bridge_core.twist_to_wheels(0.0, -1.0, 150.0, 500.0) == (75, -75)


def test_curve_combines_forward_and_turn():
    """Forward plus a left turn: the right wheel is faster than the left."""
    assert bridge_core.twist_to_wheels(0.2, 0.4, 150.0, 500.0) == (170, 230)


def test_wheels_scale_together_above_the_limit():
    """Over the limit, both wheels shrink by the same factor so the curve is kept."""
    left, right = bridge_core.twist_to_wheels(1.0, 2.0, 150.0, 500.0)
    assert max(abs(left), abs(right)) == 500
    ratio_before = (1000 - 150) / (1000 + 150)
    assert abs(left / right - ratio_before) < 0.01


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_non_finite_twist_stops_the_robot(bad):
    """A non-finite Twist value must never become a wheel speed."""
    assert bridge_core.twist_to_wheels(bad, 0.0, 150.0, 500.0) == (0, 0)
    assert bridge_core.twist_to_wheels(0.0, bad, 150.0, 500.0) == (0, 0)


# --------------------------------------------------------------------------
# CmdVelHold
# --------------------------------------------------------------------------

def test_hold_sends_zero_until_a_command_arrives_and_after_it_goes_stale():
    """No command, or a command older than 0.3 s, means zero speed."""
    hold = bridge_core.CmdVelHold(0.3)
    assert hold.current(0.0) == (0, 0)
    hold.update(200, 200, 1.0)
    assert hold.current(1.2) == (200, 200)
    assert hold.current(1.29) == (200, 200)
    assert hold.current(1.31) == (0, 0)
    hold.update(100, 100, 2.0)
    assert hold.current(2.1) == (100, 100)


# --------------------------------------------------------------------------
# LinkMonitor
# --------------------------------------------------------------------------

def sensor(t_ms):
    """Build a SensorData with the given ESP32 time."""
    return codec.SensorData(0, 0, t_ms, 0, None, None, None)


def test_link_is_down_until_data_arrives_and_goes_down_after_200ms():
    """The link is up only while SENSOR_DATA keeps arriving."""
    monitor = bridge_core.LinkMonitor(0.2)
    assert not monitor.link_ok(0.0)
    monitor.on_sensor_data(sensor(0), 5.0)
    assert monitor.link_ok(5.1)
    assert monitor.link_ok(5.19)
    assert not monitor.link_ok(5.21)


def test_jitter_ignores_constant_delay_and_sees_a_late_frame():
    """A fixed offset between the clocks is not jitter; one late frame is."""
    monitor = bridge_core.LinkMonitor(0.2)
    for i in range(40):                       # steady 50 Hz, Pi clock 100 s ahead
        monitor.on_sensor_data(sensor(i * 20), 100.0 + i * 0.02)
    assert abs(monitor.jitter_ms) < 0.01
    monitor.on_sensor_data(sensor(40 * 20), 100.0 + 40 * 0.02 + 0.030)   # 30 ms late
    assert 2.5 < monitor.jitter_ms < 3.5      # smoothed: 10 percent of 30 ms


def test_jitter_is_nan_until_enough_samples():
    """No jitter figure before 10 samples."""
    monitor = bridge_core.LinkMonitor(0.2)
    monitor.on_sensor_data(sensor(0), 0.0)
    assert math.isnan(monitor.jitter_ms)


def test_esp32_reboot_is_detected_from_the_clock_going_backwards():
    """A smaller esp time than before means the ESP32 restarted."""
    monitor = bridge_core.LinkMonitor(0.2)
    for i in range(20):
        monitor.on_sensor_data(sensor(5000 + i * 20), i * 0.02)
    monitor.on_sensor_data(sensor(30), 0.5)
    assert monitor.reboots == 1
    assert math.isnan(monitor.jitter_ms)


# --------------------------------------------------------------------------
# AckTracker
# --------------------------------------------------------------------------

def unlock():
    """Return a DrawerCmd for the tests."""
    return codec.DrawerCmd(1, codec.DrawerAction.UNLOCK, 30)


def test_tracker_resends_three_times_then_times_out():
    """Sends at 0, 0.2, 0.4, 0.6 s, then gives up at 0.8 s with STATUS_TIMEOUT."""
    tracker = bridge_core.AckTracker(0.2, 3)
    message = unlock()
    request = tracker.submit(message.TYPE, message.to_frame())
    sent = []
    for step in range(0, 100):
        now = step * 0.01
        sent += [round(now, 2) for _ in tracker.poll(now)]
        if request.done:
            finished_at = round(now, 2)
            break
    assert sent == [0.0, 0.2, 0.4, 0.6]
    assert finished_at == 0.8
    assert request.status == bridge_core.STATUS_TIMEOUT
    assert not request.success
    assert request.sends == 4


def test_tracker_ack_completes_the_request_and_stops_resends():
    """An ACK ends the request, records the round trip, and later polls send nothing."""
    tracker = bridge_core.AckTracker(0.2, 3)
    message = unlock()
    request = tracker.submit(message.TYPE, message.to_frame())
    assert len(tracker.poll(0.0)) == 1
    assert tracker.on_ack(codec.Ack(0x02, codec.AckStatus.OK), 0.015)
    assert request.done and request.success
    assert abs(tracker.last_rtt_s - 0.015) < 1e-9
    assert tracker.poll(0.5) == []


def test_tracker_ack_after_a_resend_gives_no_rtt_sample():
    """After a resend we cannot tell which send the ACK answers, so no RTT is recorded."""
    tracker = bridge_core.AckTracker(0.2, 3)
    message = unlock()
    request = tracker.submit(message.TYPE, message.to_frame())
    tracker.poll(0.0)
    tracker.poll(0.2)
    tracker.on_ack(codec.Ack(0x02, codec.AckStatus.OK), 0.25)
    assert request.success
    assert tracker.last_rtt_s is None


def test_tracker_does_not_retry_a_refusal():
    """A non-OK ACK completes at once with that status and no resend."""
    tracker = bridge_core.AckTracker(0.2, 3)
    message = unlock()
    request = tracker.submit(message.TYPE, message.to_frame())
    tracker.poll(0.0)
    tracker.on_ack(codec.Ack(0x02, codec.AckStatus.REFUSED_SAFETY), 0.01)
    assert request.status == codec.AckStatus.REFUSED_SAFETY
    assert not request.success
    assert tracker.poll(0.5) == []


def test_tracker_counts_acks_nobody_waits_for():
    """A late duplicate ACK is ignored and counted."""
    tracker = bridge_core.AckTracker(0.2, 3)
    assert not tracker.on_ack(codec.Ack(0x02, codec.AckStatus.OK), 1.0)
    assert tracker.unmatched_acks == 1


def test_tracker_allows_one_request_per_type_and_cancel_frees_it():
    """A second request of the same type is refused until the first finishes or is cancelled."""
    tracker = bridge_core.AckTracker(0.2, 3)
    message = unlock()
    first = tracker.submit(message.TYPE, message.to_frame())
    with pytest.raises(bridge_core.RequestInFlight):
        tracker.submit(message.TYPE, message.to_frame())
    other = codec.SystemCmd(codec.SystemCommand.CLEAR_FAULT)
    tracker.submit(other.TYPE, other.to_frame())          # a different type is fine
    tracker.cancel(first)
    assert first.done and first.status == bridge_core.STATUS_TIMEOUT
    tracker.submit(message.TYPE, message.to_frame())


# --------------------------------------------------------------------------
# BridgeCore
# --------------------------------------------------------------------------

def feed_frames(core, messages, now):
    """Feed ESP32 messages into the core as wire bytes and return what it publishes."""
    return core.feed(b''.join(m.to_frame() for m in messages), now)


def drive_frames(core, now):
    """Poll the core and return the DriveCmds it wrote."""
    parser = codec.Parser()
    frames = parser.feed(core.poll(now), now)
    return [codec.decode_frame(t, p) for t, p in frames if t == codec.MsgType.DRIVE_CMD]


def test_no_drive_commands_until_the_link_is_up():
    """Before any SENSOR_DATA arrives the bridge stays quiet."""
    core = bridge_core.BridgeCore()
    core.set_cmd_vel(0.2, 0.0, 0.0)
    assert core.poll(0.0) == b''
    feed_frames(core, [sensor(0)], 0.01)
    assert drive_frames(core, 0.01) == [codec.DriveCmd(200, 200)]


def test_drive_commands_at_20hz_with_the_latest_cmd_vel():
    """One DRIVE_CMD every 50 ms, carrying the newest wheel speeds."""
    core = bridge_core.BridgeCore()
    feed_frames(core, [sensor(0)], 0.0)
    sent = 0
    for step in range(0, 101):
        now = step * 0.01
        feed_frames(core, [sensor(step * 10)], now)      # keep the link alive
        if step % 10 == 0:                               # navigator publishes at 10 Hz
            core.set_cmd_vel(0.1 if step < 50 else 0.3, 0.0, now)
        frames = drive_frames(core, now)
        sent += len(frames)
        if frames and step > 60:
            assert frames == [codec.DriveCmd(300, 300)]
    assert 19 <= sent <= 22


def test_stale_cmd_vel_becomes_zero_but_heartbeat_continues():
    """If /cmd_vel goes silent the bridge keeps the heartbeat, with zero speeds."""
    core = bridge_core.BridgeCore()
    core.set_cmd_vel(0.2, 0.0, 0.0)
    feed_frames(core, [sensor(0)], 0.0)
    assert drive_frames(core, 0.0) == [codec.DriveCmd(200, 200)]
    feed_frames(core, [sensor(1000)], 0.5)
    assert drive_frames(core, 0.5) == [codec.DriveCmd(0, 0)]


def test_link_loss_stops_the_heartbeat():
    """No SENSOR_DATA for over 200 ms: no DRIVE_CMD, so the ESP32 watchdog stops the motors."""
    core = bridge_core.BridgeCore()
    core.set_cmd_vel(0.2, 0.0, 0.0)
    feed_frames(core, [sensor(0)], 0.0)
    assert drive_frames(core, 0.0)
    assert drive_frames(core, 0.1)
    assert core.poll(0.3) == b''
    assert not core.link_ok(0.3)


def test_feed_publishes_messages_and_routes_acks():
    """Data messages are returned for publishing; ACKs go to the tracker instead."""
    core = bridge_core.BridgeCore()
    request = core.submit(unlock())
    core.poll(0.0)
    messages = feed_frames(core, [
        sensor(0),
        codec.Ack(0x02, codec.AckStatus.OK),
        codec.Status(codec.StatusFlag.ESTOP, 7400),
        codec.DrawerStatus(2, 0),
    ], 0.01)
    assert [type(m) for m in messages] == [codec.SensorData, codec.Status, codec.DrawerStatus]
    assert request.success


def test_feed_survives_nonsense_from_the_esp32():
    """Command-type frames and bad payloads from the ESP32 are counted, not fatal."""
    core = bridge_core.BridgeCore()
    junk = (codec.build_frame(0x01, bytes(4))          # a command type from the ESP32
            + codec.build_frame(0x84, b'\x01')         # DRAWER_STATUS, wrong LEN
            + codec.build_frame(0xEE, b''))            # unknown type
    assert core.feed(junk, 0.0) == []
    assert core.unexpected_frames == 1
    assert core.decode_errors == 2


def test_snapshot_reports_counters_and_nan_before_data():
    """The link snapshot starts with NaN for round trip and jitter."""
    core = bridge_core.BridgeCore()
    snap = core.snapshot(0.0)
    assert not snap.link_ok
    assert math.isnan(snap.ack_rtt_ms) and math.isnan(snap.esp_clock_jitter_ms)
    core.feed(b'\x00\x01' + sensor(0).to_frame(), 0.0)
    snap = core.snapshot(0.0)
    assert snap.link_ok and snap.frames_ok == 1 and snap.junk_bytes == 2


# --------------------------------------------------------------------------
# Field conversion, checked against the real .msg files
# --------------------------------------------------------------------------

def msg_fields(name):
    """Return the field names declared in isocare_interfaces/msg/<name>.msg."""
    path = MSG_DIR / f'{name}.msg'
    if not path.exists():
        pytest.skip('isocare_interfaces sources not found next to this package')
    names = set()
    for line in path.read_text().splitlines():
        line = line.split('#')[0].strip()
        if line and '=' not in line:
            names.add(line.split()[1])
    return names


def test_sensor_data_fields_match_the_msg_file():
    """Every SensorData field the bridge sets exists, and none is forgotten."""
    message = codec.SensorData(10, -20, 123456, -150, None, 300, 450)
    fields = bridge_core.sensor_data_fields(message)
    assert set(fields) | {'header'} == msg_fields('SensorData')
    assert fields['esp_time_ms'] == 123456
    assert fields['gyro_z_centideg_s'] == -150
    assert math.isnan(fields['range_front_mm'])
    assert fields['range_left_mm'] == 300.0


def test_vitals_fields_match_the_msg_file():
    """Vitals flags unpack into bools and raw values pass through."""
    flags = codec.VitalsFlag.HR_VALID | codec.VitalsFlag.FINGER_DETECTED
    fields = bridge_core.vitals_fields(codec.Vitals(flags, 72, 98, 3650, 0))
    assert set(fields) | {'header'} == msg_fields('Vitals')
    assert fields['hr_valid'] and fields['finger_detected']
    assert not fields['spo2_valid'] and not fields['ir_temp_valid']
    assert fields['heart_rate_bpm'] == 72 and fields['temp_ir_centi_c'] == 3650


def test_drawer_status_fields_match_the_msg_file():
    """The masks become four booleans each, index 3 being the lid."""
    fields = bridge_core.drawer_status_fields(codec.DrawerStatus(0b0101, 0b1000))
    assert set(fields) | {'header'} == msg_fields('DrawerStatus')
    assert fields['unlocked'] == [True, False, True, False]
    assert fields['open'] == [False, False, False, True]


def test_esp_status_fields_match_the_msg_file():
    """Status flags become named booleans."""
    flags = codec.StatusFlag.ESTOP | codec.StatusFlag.OBST_STOP
    fields = bridge_core.esp_status_fields(codec.Status(flags, 7400))
    assert set(fields) | {'header'} == msg_fields('EspStatus')
    assert fields['estop'] and fields['obstacle_stop']
    assert not fields['watchdog'] and not fields['obstacle_slow']
    assert fields['battery_mv'] == 7400


def test_link_stats_fields_match_the_msg_file():
    """The snapshot maps onto every LinkStats field."""
    fields = bridge_core.link_stats_fields(bridge_core.BridgeCore().snapshot(0.0))
    assert set(fields) | {'header'} == msg_fields('LinkStats')


def test_message_fields_picks_the_topic_kind():
    """Each ESP32 message type maps to one publisher kind; ACKs have none."""
    kinds = [bridge_core.message_fields(m)[0] for m in (
        sensor(0), codec.Vitals(0, 0, 0, 0, 0), codec.DrawerStatus(0, 0),
        codec.Status(0, 0), codec.Ack(2, 0))]
    assert kinds == ['sensor_data', 'vitals', 'drawer_status', 'esp_status', None]
