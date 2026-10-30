"""Tests for tools/fake_esp32.py (run with: python3 -m pytest tools -q)."""

import os
import select
import threading
import time
import tty

import fake_esp32
from isocare_serial import codec
import pytest


class Rig:
    """A fake Pi wired to a FakeEsp32 with simulated time (no serial port, no sleeping)."""

    def __init__(self):
        """Create a fake booted at t=0 and a Pi-side parser."""
        self.log = []
        self.esp = fake_esp32.FakeEsp32(now=0.0, log=self.log.append)
        self.pi = codec.Parser()
        self.now = 0.0
        self.received = []  # (arrival time, decoded message)
        self._next_drive = 0.0

    def send(self, message):
        """Send a message from the Pi to the fake."""
        self.esp.feed(message.to_frame(), self.now)

    def send_raw(self, frame):
        """Send raw frame bytes from the Pi to the fake."""
        self.esp.feed(frame, self.now)

    def cmd(self, line):
        """Type a prompt command and return the reply."""
        return self.esp.command(line, self.now)

    def run(self, seconds, drive=None, step=0.01):
        """Advance time, sending DRIVE_CMD at 20 Hz if drive=(left, right) is given."""
        end = self.now + seconds
        self._next_drive = max(self._next_drive, self.now)
        while self.now < end - 1e-9:
            self.now = round(self.now + step, 6)
            if drive is not None and self.now + 1e-9 >= self._next_drive:
                self.send(codec.DriveCmd(*drive))
                self._next_drive += 0.05
            out = self.esp.tick(self.now)
            for msg_type, payload in self.pi.feed(out, self.now):
                self.received.append((self.now, codec.decode_frame(msg_type, payload)))

    def msgs(self, cls, since=0.0):
        """Return the messages of one class received at or after time since."""
        return [m for t, m in self.received if isinstance(m, cls) and t >= since]

    def last(self, cls):
        """Return the most recent message of one class."""
        return self.msgs(cls)[-1]

    def acks(self, acked_type):
        """Return the ACK statuses received for one command type."""
        return [m.status for m in self.msgs(codec.Ack) if m.acked_type == acked_type]


# --------------------------------------------------------------------------
# Streaming and motion
# --------------------------------------------------------------------------

def test_sensor_data_streams_at_50hz():
    """SENSOR_DATA arrives about 50 times per second with rising esp_time_ms."""
    rig = Rig()
    rig.run(1.0, drive=(0, 0))
    sensors = rig.msgs(codec.SensorData)
    assert 48 <= len(sensors) <= 52
    times = [s.t_ms for s in sensors]
    assert times == sorted(times)
    assert 15 <= times[11] - times[10] <= 25


def test_ticks_follow_commanded_speed():
    """200 mm/s for 2 s covers about 380 mm, which is about 760 ticks."""
    rig = Rig()
    rig.run(2.0, drive=(200, 200))
    last = rig.last(codec.SensorData)
    assert 740 <= last.left_ticks <= 780
    assert abs(last.left_ticks - last.right_ticks) <= 1
    assert last.gyro_z == 0


def test_gyro_sign_is_counter_clockwise_positive():
    """Right wheel faster than left means turning left, which is positive."""
    rig = Rig()
    rig.run(2.0, drive=(-100, 100))
    assert 7400 < rig.last(codec.SensorData).gyro_z < 7700
    rig = Rig()
    rig.run(2.0, drive=(100, -100))
    assert -7700 < rig.last(codec.SensorData).gyro_z < -7400


def test_ranges_none_become_no_reading():
    """A range set to 'none' travels as no valid reading."""
    rig = Rig()
    assert rig.cmd('range front none') == 'ok'
    rig.run(0.2, drive=(0, 0))
    last = rig.last(codec.SensorData)
    assert last.range_front_mm is None
    assert last.range_left_mm == 800


# --------------------------------------------------------------------------
# Safety rules
# --------------------------------------------------------------------------

def test_watchdog_stops_motors_and_clears_on_next_drive():
    """No DRIVE_CMD for 0.5 s stops the motors and sets WATCHDOG, a new one clears it."""
    rig = Rig()
    rig.run(1.0, drive=(200, 200))
    assert not any(codec.StatusFlag.WATCHDOG in s.flags for s in rig.msgs(codec.Status))
    rig.run(1.0)  # no more commands
    assert any(codec.StatusFlag.WATCHDOG in s.flags for s in rig.msgs(codec.Status, 1.5))
    ticks_before = rig.last(codec.SensorData).left_ticks
    rig.run(0.2)
    assert abs(rig.last(codec.SensorData).left_ticks - ticks_before) <= 1
    rig.run(0.3, drive=(100, 100))
    assert codec.StatusFlag.WATCHDOG not in rig.last(codec.Status).flags


def test_watchdog_is_armed_from_boot():
    """If the Pi never sends a DRIVE_CMD, the WATCHDOG flag appears after 0.5 s."""
    rig = Rig()
    rig.run(1.0)
    assert codec.StatusFlag.WATCHDOG in rig.last(codec.Status).flags


def test_obstacle_slow_then_stop():
    """Front obstacle under 40 cm limits speed, under 15 cm stops the motors."""
    rig = Rig()
    rig.cmd('range front 300')
    rig.run(2.0, drive=(300, 300))
    status = rig.last(codec.Status)
    assert codec.StatusFlag.OBST_SLOW in status.flags
    assert codec.StatusFlag.OBST_STOP not in status.flags
    assert rig.last(codec.SensorData).left_ticks < 420  # limited to 100 mm/s
    rig.cmd('range front 100')
    rig.run(0.5, drive=(300, 300))
    stopped_at = rig.last(codec.SensorData).left_ticks
    rig.run(0.5, drive=(300, 300))
    assert codec.StatusFlag.OBST_STOP in rig.last(codec.Status).flags
    assert abs(rig.last(codec.SensorData).left_ticks - stopped_at) <= 1


def test_estop_cuts_motors_and_refuses_unlock():
    """E-stop stops the wheels at once, refuses unlock, and still allows lock."""
    rig = Rig()
    rig.run(1.0, drive=(200, 200))
    rig.cmd('estop on')
    rig.send(codec.DrawerCmd(0, codec.DrawerAction.UNLOCK, 0))
    rig.send(codec.DrawerCmd(0, codec.DrawerAction.LOCK, 0))
    rig.run(0.3, drive=(200, 200))
    assert rig.acks(codec.MsgType.DRAWER_CMD) == [codec.AckStatus.REFUSED_SAFETY,
                                                  codec.AckStatus.OK]
    assert codec.StatusFlag.ESTOP in rig.last(codec.Status).flags
    assert not rig.last(codec.DrawerStatus).is_unlocked(0)
    ticks = rig.last(codec.SensorData).left_ticks
    rig.run(0.3, drive=(200, 200))
    assert rig.last(codec.SensorData).left_ticks == ticks


def test_fault_latches_until_cleared():
    """A fault flag stays after its cause goes, and only CLEAR_FAULT removes it."""
    rig = Rig()
    rig.cmd('fault imu on')
    rig.run(0.2, drive=(0, 0))
    assert codec.StatusFlag.IMU_FAULT in rig.last(codec.Status).flags
    rig.send(codec.SystemCmd(codec.SystemCommand.CLEAR_FAULT))  # cause still active
    rig.cmd('fault imu off')
    rig.run(0.2, drive=(0, 0))
    assert codec.StatusFlag.IMU_FAULT in rig.last(codec.Status).flags  # still latched
    rig.send(codec.DrawerCmd(0, codec.DrawerAction.UNLOCK, 0))  # latched fault blocks unlock
    rig.send(codec.SystemCmd(codec.SystemCommand.CLEAR_FAULT))
    rig.run(0.2, drive=(0, 0))
    assert rig.acks(codec.MsgType.SYSTEM_CMD) == [codec.AckStatus.REFUSED_SAFETY,
                                                  codec.AckStatus.OK]
    assert rig.acks(codec.MsgType.DRAWER_CMD) == [codec.AckStatus.REFUSED_SAFETY]
    assert codec.StatusFlag.IMU_FAULT not in rig.last(codec.Status).flags


def test_gyro_calibration_is_busy_then_done():
    """Calibration sets GYRO_CAL for 2 s and answers BUSY to a second request."""
    rig = Rig()
    rig.send(codec.SystemCmd(codec.SystemCommand.RECALIBRATE_GYRO))
    rig.run(0.2, drive=(0, 0))
    assert codec.StatusFlag.GYRO_CAL in rig.last(codec.Status).flags
    rig.send(codec.SystemCmd(codec.SystemCommand.RECALIBRATE_GYRO))
    rig.run(2.2, drive=(0, 0))
    assert rig.acks(codec.MsgType.SYSTEM_CMD) == [codec.AckStatus.OK, codec.AckStatus.BUSY]
    assert codec.StatusFlag.GYRO_CAL not in rig.last(codec.Status).flags


def test_soft_reset_restarts_counters():
    """SOFT_RESET is ACKed, then ticks and esp_time_ms restart from zero."""
    rig = Rig()
    rig.run(1.0, drive=(200, 200))
    assert rig.last(codec.SensorData).left_ticks > 100
    rig.send(codec.SystemCmd(codec.SystemCommand.SOFT_RESET))
    rig.run(0.1, drive=(0, 0))
    assert rig.acks(codec.MsgType.SYSTEM_CMD) == [codec.AckStatus.OK]
    last = rig.last(codec.SensorData)
    assert last.t_ms < 200
    assert abs(last.left_ticks) < 10


# --------------------------------------------------------------------------
# Drawers and vitals
# --------------------------------------------------------------------------

def test_drawer_unlock_open_and_status():
    """Unlock is ACKed and shown in DRAWER_STATUS; a locked drawer cannot be opened."""
    rig = Rig()
    assert 'locked' in rig.cmd('open 1')
    rig.send(codec.DrawerCmd(1, codec.DrawerAction.UNLOCK, 0))
    rig.run(0.1, drive=(0, 0))
    assert rig.acks(codec.MsgType.DRAWER_CMD) == [codec.AckStatus.OK]
    assert rig.last(codec.DrawerStatus).is_unlocked(1)
    rig.cmd('open 1')
    rig.run(0.1, drive=(0, 0))
    assert rig.last(codec.DrawerStatus).is_open(1)
    rig.cmd('close 1')
    rig.run(0.1, drive=(0, 0))
    status = rig.last(codec.DrawerStatus)
    assert status.is_unlocked(1) and not status.is_open(1)


def test_auto_relock_waits_for_close():
    """The relock timer runs only while the drawer is closed."""
    rig = Rig()
    rig.send(codec.DrawerCmd(2, codec.DrawerAction.UNLOCK, 2))
    rig.send(codec.DrawerCmd(0, codec.DrawerAction.UNLOCK, 1))
    rig.cmd('open 0')
    rig.run(3.0, drive=(0, 0))
    status = rig.last(codec.DrawerStatus)
    assert not status.is_unlocked(2)   # relocked after 2 s
    assert status.is_unlocked(0)       # open, so no relock yet
    rig.cmd('close 0')
    rig.run(1.5, drive=(0, 0))
    assert not rig.last(codec.DrawerStatus).is_unlocked(0)


def test_vitals_warm_up_valid_flags_and_stop():
    """HR becomes valid after warm-up, temperature follows the prompt, STOP ends VITALS."""
    rig = Rig()
    both = codec.SensorMask.MAX30102 | codec.SensorMask.MLX90614
    rig.send(codec.MeasureCmd(codec.MeasureAction.START, both))
    rig.run(1.5, drive=(0, 0))
    first = rig.msgs(codec.Vitals)
    assert first and all(codec.VitalsFlag.IR_TEMP_VALID in v.flags for v in first)
    assert all(v.heart_rate_bpm is None for v in first)
    rig.cmd('temp 38.4')
    rig.run(2.5, drive=(0, 0))
    last = rig.last(codec.Vitals)
    assert 70 <= last.heart_rate_bpm <= 74
    assert 96 <= last.spo2_percent <= 98
    assert abs(last.temp_ir_c - 38.4) < 0.1
    assert last.temp_contact_c is None   # MAX30205 was not selected
    rig.send(codec.MeasureCmd(codec.MeasureAction.STOP, both))
    rig.run(0.1, drive=(0, 0))
    count = len(rig.msgs(codec.Vitals))
    rig.run(2.0, drive=(0, 0))
    assert len(rig.msgs(codec.Vitals)) == count


def test_measure_refusals():
    """An empty sensor mask is a bad payload; a latched vitals fault refuses measuring."""
    rig = Rig()
    rig.send(codec.MeasureCmd(codec.MeasureAction.START, codec.SensorMask(0)))
    rig.cmd('fault vitals on')
    rig.send(codec.MeasureCmd(codec.MeasureAction.START, codec.SensorMask.MAX30102))
    rig.run(0.2, drive=(0, 0))
    assert rig.acks(codec.MsgType.MEASURE_CMD) == [codec.AckStatus.BAD_PAYLOAD,
                                                   codec.AckStatus.REFUSED_SAFETY]


# --------------------------------------------------------------------------
# Malformed input
# --------------------------------------------------------------------------

def test_malformed_input_is_answered_as_the_protocol_says():
    """Bad payloads and unknown types get ACKs, bad DRIVE_CMD and bad checksums get nothing."""
    rig = Rig()
    rig.send_raw(codec.build_frame(0x02, b'\x01'))             # DRAWER_CMD, wrong LEN
    rig.send_raw(codec.build_frame(0x7F, b''))                 # unknown type
    rig.send_raw(codec.build_frame(0x82, bytes(20)))           # a Pi-bound type sent to the ESP
    rig.send_raw(codec.build_frame(0x01, b'\x00\x01'))         # DRIVE_CMD, wrong LEN: no ACK
    corrupted = bytearray(codec.DrawerCmd(0, codec.DrawerAction.UNLOCK, 0).to_frame())
    corrupted[-1] ^= 0xFF
    rig.send_raw(bytes(corrupted))                             # bad checksum: no reply
    rig.run(0.1, drive=(0, 0))
    acks = [(a.acked_type, a.status) for a in rig.msgs(codec.Ack)]
    assert acks == [(0x02, codec.AckStatus.BAD_PAYLOAD),
                    (0x7F, codec.AckStatus.UNKNOWN_TYPE),
                    (0x82, codec.AckStatus.UNKNOWN_TYPE)]
    assert rig.esp.bad_drive == 1
    assert not rig.last(codec.DrawerStatus).is_unlocked(0)


# --------------------------------------------------------------------------
# Fault injection
# --------------------------------------------------------------------------

def test_corrupt_flips_one_checksum():
    """'corrupt' makes exactly the next frame fail its checksum at the Pi."""
    rig = Rig()
    rig.cmd('corrupt')
    rig.run(0.2, drive=(0, 0))
    assert rig.pi.bad_checksum >= 1
    assert len(rig.msgs(codec.SensorData)) > 5


def test_drop_and_dropack():
    """'drop' removes outgoing frames, 'dropack' removes ACKs so the Pi sees no reply."""
    rig = Rig()
    rig.cmd('drop 3')
    rig.run(0.1, drive=(0, 0))
    assert sum('dropped an outgoing frame' in line for line in rig.log) == 3
    rig.cmd('dropack')
    rig.send(codec.DrawerCmd(0, codec.DrawerAction.LOCK, 0))
    rig.run(0.1, drive=(0, 0))
    assert rig.acks(codec.MsgType.DRAWER_CMD) == []
    rig.send(codec.DrawerCmd(0, codec.DrawerAction.LOCK, 0))   # a retry now gets through
    rig.run(0.1, drive=(0, 0))
    assert rig.acks(codec.MsgType.DRAWER_CMD) == [codec.AckStatus.OK]


def test_ackdelay_postpones_acks():
    """With ackdelay 0.3 the ACK arrives at least 0.3 s after the command."""
    rig = Rig()
    rig.cmd('ackdelay 0.3')
    sent_at = rig.now
    rig.send(codec.DrawerCmd(0, codec.DrawerAction.LOCK, 0))
    rig.run(0.6, drive=(0, 0))
    arrival = [t for t, m in rig.received if isinstance(m, codec.Ack)][0]
    assert arrival - sent_at >= 0.3


def test_silent_sends_nothing():
    """'silent 1' mutes the fake for one second and it resumes afterwards."""
    rig = Rig()
    rig.cmd('silent 1.0')
    rig.run(1.5, drive=(0, 0))
    assert min(t for t, _ in rig.received) >= 1.0
    assert len(rig.msgs(codec.SensorData)) > 10


# --------------------------------------------------------------------------
# Prompt
# --------------------------------------------------------------------------

def test_prompt_handles_bad_input():
    """Unknown commands and bad arguments give a message, never an exception."""
    rig = Rig()
    assert 'unknown command' in rig.cmd('bogus')
    assert 'bad arguments' in rig.cmd('range front')
    assert 'bad arguments' in rig.cmd('range up 3')
    assert 'bad arguments' in rig.cmd('estop maybe')
    assert 'estop' in rig.cmd('help')
    assert 'uptime' in rig.cmd('status')
    assert rig.cmd('') == ''
    rig.cmd('quit')
    assert rig.esp.quit_requested


def test_describe_flags():
    """Flag sets print as readable names."""
    flags = codec.StatusFlag.ESTOP | codec.StatusFlag.WATCHDOG
    assert fake_esp32.describe_flags(flags) == 'ESTOP|WATCHDOG'
    assert fake_esp32.describe_flags(codec.StatusFlag(0)) == 'none'


# --------------------------------------------------------------------------
# Real serial port
# --------------------------------------------------------------------------

def test_runs_over_a_real_serial_port():
    """The real-time runner works through pyserial on a pseudo-terminal pair."""
    pytest.importorskip('serial')
    master, slave = os.openpty()
    tty.setraw(slave)
    port = os.ttyname(slave)
    stop = threading.Event()
    thread = threading.Thread(
        target=fake_esp32.run, args=(port,),
        kwargs={'prompt': False, 'stop_event': stop}, daemon=True)
    thread.start()
    parser = codec.Parser()
    frames = []
    next_drive = 0.0
    deadline = time.monotonic() + 1.5
    try:
        while time.monotonic() < deadline:
            if time.monotonic() >= next_drive:
                os.write(master, codec.DriveCmd(200, 200).to_frame())
                next_drive = time.monotonic() + 0.05
            ready, _, _ = select.select([master], [], [], 0.01)
            if ready:
                frames += parser.feed(os.read(master, 4096))
    finally:
        stop.set()
        thread.join(2.0)
        os.close(master)
        os.close(slave)
    messages = [codec.decode_frame(t, p) for t, p in frames]
    sensors = [m for m in messages if isinstance(m, codec.SensorData)]
    assert len(sensors) > 40
    assert sensors[-1].left_ticks > 100
    assert any(isinstance(m, codec.Status) for m in messages)
    assert parser.bad_checksum == 0
