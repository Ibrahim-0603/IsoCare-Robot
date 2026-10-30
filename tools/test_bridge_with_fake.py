"""Run the bridge core against the fake ESP32 in simulated time (the Week 5 proof)."""

import math

import fake_esp32
from isocare_serial import bridge_core
from isocare_serial import codec


class Loop:
    """Bridge core and fake ESP32 wired back to back, with no port and no sleeping."""

    def __init__(self):
        """Create both ends booted at t=0."""
        self.log = []
        self.esp = fake_esp32.FakeEsp32(now=0.0, log=self.log.append)
        self.core = bridge_core.BridgeCore()
        self.now = 0.0
        self.published = []     # (time, codec message) the bridge would publish
        self._next_cmd_vel = 0.0

    def run(self, seconds, cmd_vel=None, step=0.005):
        """Advance time; if cmd_vel=(linear, angular) is given, publish it at 10 Hz."""
        end = self.now + seconds
        while self.now < end - 1e-9:
            self.now = round(self.now + step, 6)
            if cmd_vel is not None and self.now + 1e-9 >= self._next_cmd_vel:
                self.core.set_cmd_vel(cmd_vel[0], cmd_vel[1], self.now)
                self._next_cmd_vel = self.now + 0.1
            self.esp.feed(self.core.poll(self.now), self.now)
            for message in self.core.feed(self.esp.tick(self.now), self.now):
                self.published.append((self.now, message))

    def msgs(self, cls):
        """Return the published messages of one class."""
        return [m for _, m in self.published if isinstance(m, cls)]

    def commands_received(self, text):
        """Count the fake's log lines that mention text."""
        return sum(text in line for line in self.log)


def test_fake_cmd_vel_goes_out_and_fake_ticks_come_back():
    """The Week 5 proof: /cmd_vel in, wheel ticks climbing in /sensor_data out."""
    loop = Loop()
    loop.run(2.0, cmd_vel=(0.2, 0.0))
    sensors = loop.msgs(codec.SensorData)
    assert 90 <= len(sensors) <= 102                 # about 50 Hz for 2 s
    assert 740 <= sensors[-1].left_ticks <= 790      # 200 mm/s for 2 s, with the wheel lag
    assert abs(sensors[-1].left_ticks - sensors[-1].right_ticks) <= 1
    assert loop.core.link_ok(loop.now)
    assert codec.StatusFlag.WATCHDOG not in loop.msgs(codec.Status)[-1].flags


def test_two_watchdogs_navigator_dies_but_the_esp32_stays_happy():
    """If /cmd_vel stops, the bridge sends zeros: the robot stops without any ESP32 fault."""
    loop = Loop()
    loop.run(1.0, cmd_vel=(0.2, 0.0))
    loop.run(1.0)                                    # navigator is gone
    sensors = loop.msgs(codec.SensorData)
    before = sensors[-1].left_ticks
    loop.run(0.3)
    assert abs(loop.msgs(codec.SensorData)[-1].left_ticks - before) <= 1   # stopped
    assert not any(codec.StatusFlag.WATCHDOG in s.flags for s in loop.msgs(codec.Status))
    loop.run(0.5, cmd_vel=(0.2, 0.0))                # navigator comes back
    assert loop.msgs(codec.SensorData)[-1].left_ticks > before + 50


def test_turning_command_reaches_the_gyro():
    """A counter-clockwise Twist produces a positive gyro value back."""
    loop = Loop()
    loop.run(2.0, cmd_vel=(0.0, 1.0))
    assert loop.msgs(codec.SensorData)[-1].gyro_z > 5000     # 1 rad/s is about 5730


def test_unlock_gets_its_ack_and_shows_in_drawer_status():
    """A command is ACKed OK on the first send and the new state is published."""
    loop = Loop()
    loop.run(0.1)
    request = loop.core.submit(codec.DrawerCmd(1, codec.DrawerAction.UNLOCK, 0))
    loop.run(0.2)
    assert request.done and request.success
    assert request.sends == 1
    assert loop.core.tracker.last_rtt_s is not None
    assert loop.msgs(codec.DrawerStatus)[-1].is_unlocked(1)


def test_lost_acks_are_resent_and_the_command_still_succeeds():
    """The fake loses two ACKs: the bridge resends twice and the third attempt succeeds."""
    loop = Loop()
    loop.run(0.1)
    loop.esp.command('dropack 2', loop.now)
    request = loop.core.submit(codec.DrawerCmd(1, codec.DrawerAction.UNLOCK, 0))
    loop.run(1.0)
    assert request.success
    assert request.sends == 3
    assert loop.commands_received('RX DrawerCmd') == 3       # state-setting, so repeats are safe


def test_no_ack_at_all_times_out_after_four_sends():
    """With every ACK lost, the bridge sends 4 times, then reports STATUS_TIMEOUT."""
    loop = Loop()
    loop.run(0.1)
    loop.esp.command('dropack 100', loop.now)
    request = loop.core.submit(codec.DrawerCmd(1, codec.DrawerAction.UNLOCK, 0))
    loop.run(0.7)
    assert not request.done
    loop.run(0.3)
    assert request.done and not request.success
    assert request.status == bridge_core.STATUS_TIMEOUT
    assert request.sends == 4
    assert loop.commands_received('RX DrawerCmd') == 4
    # The drawer really did unlock: the lost ACK does not mean the command was lost.
    assert loop.msgs(codec.DrawerStatus)[-1].is_unlocked(1)


def test_a_refusal_is_not_retried():
    """REFUSED_SAFETY is returned at once, after a single send."""
    loop = Loop()
    loop.run(0.1)
    loop.esp.command('estop on', loop.now)
    request = loop.core.submit(codec.DrawerCmd(1, codec.DrawerAction.UNLOCK, 0))
    loop.run(0.5)
    assert request.done and not request.success
    assert request.status == codec.AckStatus.REFUSED_SAFETY
    assert request.sends == 1
    assert loop.commands_received('RX DrawerCmd') == 1


def test_a_slow_ack_wins_over_the_resend_and_the_duplicate_is_ignored():
    """A 0.3 s ACK delay: one resend goes out, the first ACK wins, the second is unmatched."""
    loop = Loop()
    loop.run(0.1)
    loop.esp.command('ackdelay 0.3', loop.now)
    request = loop.core.submit(codec.DrawerCmd(1, codec.DrawerAction.UNLOCK, 0))
    loop.run(1.0)
    assert request.success
    assert request.sends == 2
    assert loop.core.tracker.unmatched_acks == 1
    assert loop.core.tracker.last_rtt_s is None              # ambiguous after a resend


def test_measure_and_vitals_flow_through():
    """MEASURE_CMD starts the VITALS stream and the bridge hands the messages on."""
    loop = Loop()
    loop.run(0.1)
    request = loop.core.submit(codec.MeasureCmd(
        codec.MeasureAction.START, codec.SensorMask.MLX90614))
    loop.esp.command('temp 38.6', loop.now)
    loop.run(2.0)
    assert request.success
    vitals = loop.msgs(codec.Vitals)
    assert len(vitals) >= 1
    assert abs(vitals[-1].temp_ir_c - 38.6) < 0.1


def test_silent_esp32_means_link_lost_and_the_heartbeat_stops():
    """When the fake goes quiet, link_ok turns false and DRIVE_CMDs stop; both recover."""
    loop = Loop()
    loop.run(1.0, cmd_vel=(0.2, 0.0))
    assert loop.core.link_ok(loop.now)
    loop.esp.command('silent 1.0', loop.now)
    loop.run(0.5, cmd_vel=(0.2, 0.0))
    assert not loop.core.link_ok(loop.now)
    last_drive_seen = loop.esp.last_drive
    loop.run(0.2, cmd_vel=(0.2, 0.0))
    assert loop.esp.last_drive == last_drive_seen            # nothing arrived while down
    loop.run(1.0, cmd_vel=(0.2, 0.0))
    assert loop.core.link_ok(loop.now)
    assert loop.esp.last_drive > last_drive_seen


def test_corrupt_and_dropped_frames_show_up_in_the_link_stats():
    """Injected faults are visible in the counters and do not stop the stream."""
    loop = Loop()
    loop.esp.command('corrupt', loop.now)
    loop.run(0.5, cmd_vel=(0.0, 0.0))
    snapshot = loop.core.snapshot(loop.now)
    assert snapshot.bad_checksum >= 1
    assert snapshot.frames_ok > 20
    assert snapshot.link_ok
    assert math.isclose(snapshot.esp_clock_jitter_ms, 0.0, abs_tol=6.0)


def test_soft_reset_is_noticed_by_the_monitor():
    """After SOFT_RESET the esp clock restarts and the bridge records a reboot."""
    loop = Loop()
    loop.run(1.0, cmd_vel=(0.2, 0.0))
    request = loop.core.submit(codec.SystemCmd(codec.SystemCommand.SOFT_RESET))
    loop.run(0.5, cmd_vel=(0.2, 0.0))
    assert request.success
    assert loop.core.monitor.reboots == 1
