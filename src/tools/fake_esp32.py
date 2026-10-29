#!/usr/bin/env python3
"""Fake ESP32 for IsoCare: plays the ESP32's side of protocol v0.1, no hardware needed.

It opens a serial port (normally one end of a socat virtual cable), parses the
Pi's command frames with the real codec, and sends back what the real firmware
would: SENSOR_DATA at 50 Hz, STATUS, DRAWER_STATUS, VITALS and ACKs. It also
enforces the safety rules from protocol.md section 7 (0.5 s watchdog, obstacle
slow-down and stop, E-stop, fault latch).

Usage:
    socat -d -d pty,raw,echo=0,link=/tmp/esp_fake pty,raw,echo=0,link=/tmp/esp_pi
    python3 tools/fake_esp32.py /tmp/esp_fake        (the bridge opens /tmp/esp_pi)

Type 'help' at the prompt for the commands that change the fake world and
inject faults (corrupt checksums, dropped frames, delayed ACKs, silence).

All physical constants below are made up. They only need to be consistent.
"""

import argparse
import math
from pathlib import Path
import random
import sys
import threading
import time

try:
    from isocare_serial import codec
except ImportError:  # running from the repo without sourcing the workspace
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'isocare_serial'))
    from isocare_serial import codec

TICKS_PER_MM = 2.0          # fake encoder resolution
WHEELBASE_MM = 150.0        # fake distance between the wheels
SPEED_TAU_S = 0.1           # wheels approach the target speed with this time constant
WATCHDOG_S = 0.5            # no DRIVE_CMD for this long stops the motors
SLOW_RANGE_MM = 400         # front obstacle closer than this: OBST_SLOW
STOP_RANGE_MM = 150         # front obstacle closer than this: OBST_STOP
SLOW_SPEED_MM_S = 100.0     # forward speed limit while OBST_SLOW
SENSOR_PERIOD_S = 0.02      # SENSOR_DATA at 50 Hz
VITALS_PERIOD_S = 1.0       # VITALS at 1 Hz while measuring
DRAWER_PERIOD_S = 1.0       # DRAWER_STATUS at least once per second
STATUS_PERIOD_S = 0.5       # STATUS at least at 2 Hz
GYRO_CAL_S = 2.0            # how long gyro calibration takes
VITALS_WARMUP_S = 2.0       # HR and SpO2 become valid this long after START
FAULT_FLAGS = (codec.StatusFlag.IMU_FAULT | codec.StatusFlag.VITALS_FAULT
               | codec.StatusFlag.RANGE_FAULT)
FAULT_FLAG_BY_NAME = {
    'imu': codec.StatusFlag.IMU_FAULT,
    'vitals': codec.StatusFlag.VITALS_FAULT,
    'range': codec.StatusFlag.RANGE_FAULT,
}
COMMAND_TYPES = (codec.MsgType.DRIVE_CMD, codec.MsgType.DRAWER_CMD,
                 codec.MsgType.MEASURE_CMD, codec.MsgType.SYSTEM_CMD)

HELP = """\
commands:
  estop on|off                    press or release the hardware E-stop
  range front|left|right N        set a range in mm, or 'none' for no reading
  fault imu|vitals|range on|off   start or stop a sensor fault (the flag latches)
  finger on|off                   finger on or off the vitals sensors
  hr N | spo2 N | temp C          set the fake vitals (temp = forehead temperature, degC)
  battery MV                      set the battery voltage in mV (0 = no sensing)
  open N | close N                open or close drawer N (0-2) or the lid (3)
  corrupt                         flip the checksum of the next outgoing frame
  drop [N]                        drop the next N outgoing frames (default 1)
  dropack [N]                     drop the next N ACK frames (default 1)
  ackdelay SECONDS                delay every ACK by this long (0 = off)
  silent SECONDS                  send nothing at all for this long
  status                          print the fake robot's state
  quit                            stop the fake
"""


def _clamp(value, low, high):
    """Return value limited to the range low..high."""
    return max(low, min(high, value))


def _wrap_int32(value):
    """Return value rounded and wrapped into the signed 32-bit range."""
    return (int(round(value)) + 2**31) % 2**32 - 2**31


def _on_off(text):
    """Convert 'on' or 'off' to a bool, raising ValueError otherwise."""
    if text.lower() not in ('on', 'off'):
        raise ValueError(text)
    return text.lower() == 'on'


def describe_flags(flags):
    """Return a readable list of the flags that are set, like 'ESTOP|WATCHDOG'."""
    names = [flag.name for flag in type(flags) if flag in flags]
    return '|'.join(names) if names else 'none'


class FakeEsp32:
    """The simulated ESP32. It owns no serial port and no clock, so tests are exact.

    Feed it received bytes with feed(data, now), call tick(now) regularly, and
    write whatever tick() returns to the port. 'now' is any monotonic time in
    seconds.
    """

    def __init__(self, now=0.0, seed=1, log=print):
        """Create a fake that boots at time now. log receives human-readable lines."""
        self.log = log
        self.verbose = False
        self.quit_requested = False
        self.parser = codec.Parser()
        self._rng = random.Random(seed)
        self._seq = 0
        self._outbox = []
        # Fault injection, changed by the prompt commands.
        self.corrupt_next = False
        self.drop_frames = 0
        self.drop_acks = 0
        self.ack_delay_s = 0.0
        self.silent_until = 0.0
        # The fake world, changed by the prompt commands.
        self.estop = False
        self.cause = {'imu': False, 'vitals': False, 'range': False}
        self.ranges = {'front': 2000, 'left': 800, 'right': 800}
        self.battery_mv = 7400
        self.finger = True
        self.hr_bpm = 72
        self.spo2_pct = 98
        self.temp_c = 36.6
        # Drawers and lid (index 3). They keep their state across a soft reset.
        self.unlocked = [False] * 4
        self.is_open = [False] * 4
        self.relock_s = [0] * 4
        self.relock_deadline = [None] * 4
        self.bad_drive = 0
        self._logged_flags = None
        self._boot(now)

    # ------------------------------------------------------------------
    # Firmware state
    # ------------------------------------------------------------------

    def _boot(self, now):
        """Reset everything a reboot would reset."""
        self.boot_time = now
        self.last_tick = now
        self.target = [0.0, 0.0]    # commanded wheel speeds, mm/s (left, right)
        self.speed = [0.0, 0.0]     # actual wheel speeds, mm/s
        self.ticks = [0.0, 0.0]     # cumulative encoder ticks
        self.last_drive = None      # time of the last valid DRIVE_CMD
        self.latched = codec.StatusFlag(0)
        for name, active in self.cause.items():
            if active:
                self.latched |= FAULT_FLAG_BY_NAME[name]
        self.gyro_cal_until = None
        self.measuring = False
        self.mask = codec.SensorMask(0)
        self.measure_start = now
        self._next = {'sensor': now, 'vitals': now, 'drawer': now, 'status': now}
        self._last_status = None
        self._last_drawer = None

    def _watchdog(self, now):
        """Return True if no DRIVE_CMD has arrived for 0.5 s (counted from boot at first)."""
        reference = self.boot_time if self.last_drive is None else self.last_drive
        return now - reference > WATCHDOG_S

    def _range(self, name):
        """Return a range in mm, or None if that sensor has no valid reading."""
        return None if self.cause['range'] else self.ranges[name]

    def _status_flags(self, now):
        """Return the current STATUS flags."""
        flags = self.latched
        if self.estop:
            flags |= codec.StatusFlag.ESTOP
        if self._watchdog(now):
            flags |= codec.StatusFlag.WATCHDOG
        front = self._range('front')
        if front is not None and front < SLOW_RANGE_MM:
            flags |= codec.StatusFlag.OBST_SLOW
        if front is not None and front < STOP_RANGE_MM:
            flags |= codec.StatusFlag.OBST_STOP
        if self.gyro_cal_until is not None:
            flags |= codec.StatusFlag.GYRO_CAL
        return flags

    def _unsafe(self):
        """Return True if an E-stop or latched fault blocks unlocking and calibration."""
        return self.estop or bool(self.latched & FAULT_FLAGS)

    def _effective_targets(self, now):
        """Return the wheel speeds the motors will really chase after safety limits."""
        if self.estop or self._watchdog(now):
            return [0.0, 0.0]
        left, right = self.target
        front = self._range('front')
        if front is not None and front < STOP_RANGE_MM:
            return [0.0, 0.0]
        if front is not None and front < SLOW_RANGE_MM:
            left, right = min(left, SLOW_SPEED_MM_S), min(right, SLOW_SPEED_MM_S)
        return [left, right]

    # ------------------------------------------------------------------
    # Receiving
    # ------------------------------------------------------------------

    def feed(self, data, now):
        """Process bytes received from the Pi."""
        for msg_type, payload in self.parser.feed(data, now):
            self._handle(msg_type, payload, now)

    def _handle(self, msg_type, payload, now):
        """React to one valid frame from the Pi."""
        if msg_type == codec.MsgType.DRIVE_CMD:
            try:
                cmd = codec.DriveCmd.decode(payload)
            except codec.ProtocolError as error:
                self.bad_drive += 1
                self.log(f'RX bad DRIVE_CMD ignored (never ACKed): {error}')
                return
            self.last_drive = now
            self.target = [float(cmd.left_mm_s), float(cmd.right_mm_s)]
            if self.verbose:
                self.log(f'RX {cmd}')
            return
        if msg_type not in COMMAND_TYPES:
            self.log(f'RX unknown type 0x{msg_type:02X} -> UNKNOWN_TYPE')
            self._ack(msg_type, codec.AckStatus.UNKNOWN_TYPE, now)
            return
        try:
            cmd = codec.decode_frame(msg_type, payload)
        except codec.ProtocolError as error:
            self.log(f'RX bad payload for type 0x{msg_type:02X} -> BAD_PAYLOAD: {error}')
            self._ack(msg_type, codec.AckStatus.BAD_PAYLOAD, now)
            return
        handler = {
            codec.MsgType.DRAWER_CMD: self._on_drawer,
            codec.MsgType.MEASURE_CMD: self._on_measure,
            codec.MsgType.SYSTEM_CMD: self._on_system,
        }[msg_type]
        status = handler(cmd, now)
        self.log(f'RX {cmd} -> {status.name}')
        self._ack(msg_type, status, now)
        if (msg_type == codec.MsgType.SYSTEM_CMD
                and cmd.command == codec.SystemCommand.SOFT_RESET):
            self._boot(now)

    def _on_drawer(self, cmd, now):
        """Handle DRAWER_CMD and return the ACK status. Locking is always allowed."""
        index = cmd.drawer_id
        if cmd.action == codec.DrawerAction.UNLOCK:
            if self._unsafe():
                return codec.AckStatus.REFUSED_SAFETY
            self.unlocked[index] = True
            self.relock_s[index] = cmd.relock_s
            self._arm_relock(index, now)
        else:
            self.unlocked[index] = False
            self.relock_deadline[index] = None
        return codec.AckStatus.OK

    def _arm_relock(self, index, now):
        """Start the auto-relock timer if the drawer is unlocked, closed and has a delay."""
        if self.unlocked[index] and not self.is_open[index] and self.relock_s[index] > 0:
            self.relock_deadline[index] = now + self.relock_s[index]
        else:
            self.relock_deadline[index] = None

    def _on_measure(self, cmd, now):
        """Handle MEASURE_CMD and return the ACK status."""
        if cmd.action == codec.MeasureAction.STOP:
            self.measuring = False
            return codec.AckStatus.OK
        if not cmd.sensor_mask:
            return codec.AckStatus.BAD_PAYLOAD
        if self.latched & codec.StatusFlag.VITALS_FAULT:
            return codec.AckStatus.REFUSED_SAFETY
        if not self.measuring:
            self.measure_start = now
            self._next['vitals'] = now
        self.measuring = True
        self.mask = cmd.sensor_mask
        return codec.AckStatus.OK

    def _on_system(self, cmd, now):
        """Handle SYSTEM_CMD and return the ACK status."""
        if cmd.command == codec.SystemCommand.RECALIBRATE_GYRO:
            if self.latched & codec.StatusFlag.IMU_FAULT:
                return codec.AckStatus.REFUSED_SAFETY
            moving = any(abs(speed) > 1.0 for speed in self.speed)
            if self.gyro_cal_until is not None or moving:
                return codec.AckStatus.BUSY
            self.gyro_cal_until = now + GYRO_CAL_S
        elif cmd.command == codec.SystemCommand.CLEAR_FAULT:
            if self.estop or any(self.cause.values()):
                return codec.AckStatus.REFUSED_SAFETY
            self.latched = codec.StatusFlag(0)
        return codec.AckStatus.OK  # SOFT_RESET reboots after the ACK is queued

    # ------------------------------------------------------------------
    # Simulation and sending
    # ------------------------------------------------------------------

    def tick(self, now):
        """Advance the simulation to now and return the bytes to write to the port."""
        self._advance(now)
        flags = self._status_flags(now)
        if flags != self._logged_flags:
            self._logged_flags = flags
            self.log(f'STATUS flags: {describe_flags(flags)}')
        self._send_periodic(now, flags)
        return self._flush(now)

    def _advance(self, now):
        """Move the wheels, ticks, timers and drawers forward to now."""
        dt = min(now - self.last_tick, 0.25)
        self.last_tick = now
        if self.gyro_cal_until is not None and now >= self.gyro_cal_until:
            self.gyro_cal_until = None
        for index in range(4):
            deadline = self.relock_deadline[index]
            if deadline is not None and now >= deadline:
                self.unlocked[index] = False
                self.relock_deadline[index] = None
        if dt <= 0:
            return
        targets = self._effective_targets(now)
        alpha = 1.0 - math.exp(-dt / SPEED_TAU_S)
        for side in (0, 1):
            if self.estop:
                self.speed[side] = 0.0
            else:
                self.speed[side] += (targets[side] - self.speed[side]) * alpha
            self.ticks[side] += self.speed[side] * dt * TICKS_PER_MM

    def _due(self, name, now, period):
        """Return True if periodic message name is due, and schedule the next one."""
        if now < self._next[name]:
            return False
        following = self._next[name] + period
        self._next[name] = following if following > now else now + period
        return True

    def _send_periodic(self, now, flags):
        """Queue SENSOR_DATA, STATUS, DRAWER_STATUS and VITALS when they are due."""
        if self._due('sensor', now, SENSOR_PERIOD_S):
            self._send(self._sensor_data(now), now)
        status_signature = (int(flags), self.battery_mv)
        if status_signature != self._last_status:
            self._last_status = status_signature
            self._next['status'] = now + STATUS_PERIOD_S
            self._send(codec.Status(flags, self.battery_mv), now)
        elif self._due('status', now, STATUS_PERIOD_S):
            self._send(codec.Status(flags, self.battery_mv), now)
        drawer_signature = (tuple(self.unlocked), tuple(self.is_open))
        if drawer_signature != self._last_drawer:
            self._last_drawer = drawer_signature
            self._next['drawer'] = now + DRAWER_PERIOD_S
            self._send(self._drawer_status(), now)
        elif self._due('drawer', now, DRAWER_PERIOD_S):
            self._send(self._drawer_status(), now)
        if self.measuring and self._due('vitals', now, VITALS_PERIOD_S):
            self._send(self._vitals_message(now), now)

    def _wire_range(self, name):
        """Return a range as the codec wants it: int mm, or None for no reading."""
        value = self._range(name)
        return None if value is None else _clamp(int(value), 0, codec.NO_RANGE - 1)

    def _sensor_data(self, now):
        """Build the current SENSOR_DATA message."""
        t_ms = int((now - self.boot_time) * 1000) & 0xFFFFFFFF
        yaw_deg_s = math.degrees((self.speed[1] - self.speed[0]) / WHEELBASE_MM)
        gyro = 0 if self.cause['imu'] else _clamp(round(yaw_deg_s * 100.0), -32768, 32767)
        return codec.SensorData(
            _wrap_int32(self.ticks[0]), _wrap_int32(self.ticks[1]), t_ms, gyro,
            self._wire_range('front'), self._wire_range('left'), self._wire_range('right'))

    def _drawer_status(self):
        """Build the current DRAWER_STATUS message."""
        unlocked_mask = sum(1 << i for i in range(4) if self.unlocked[i])
        open_mask = sum(1 << i for i in range(4) if self.is_open[i])
        return codec.DrawerStatus(unlocked_mask, open_mask)

    def _vitals_message(self, now):
        """Build a plausible VITALS message for the selected sensors."""
        flags = codec.VitalsFlag(0)
        heart_rate = spo2 = temp_ir = temp_contact = 0
        if not self.cause['vitals']:
            warm = now - self.measure_start >= VITALS_WARMUP_S
            if codec.SensorMask.MAX30102 in self.mask and self.finger:
                flags |= codec.VitalsFlag.FINGER_DETECTED
                if warm:
                    flags |= codec.VitalsFlag.HR_VALID | codec.VitalsFlag.SPO2_VALID
                    heart_rate = _clamp(self.hr_bpm + self._rng.randint(-1, 1), 0, 255)
                    spo2 = _clamp(self.spo2_pct + self._rng.randint(-1, 0), 0, 255)
            if codec.SensorMask.MLX90614 in self.mask:
                flags |= codec.VitalsFlag.IR_TEMP_VALID
                temp_ir = round((self.temp_c + self._rng.uniform(-0.05, 0.05)) * 100)
            if codec.SensorMask.MAX30205 in self.mask and self.finger:
                flags |= codec.VitalsFlag.CONTACT_TEMP_VALID
                temp_contact = round((self.temp_c + 0.2 + self._rng.uniform(-0.05, 0.05)) * 100)
        return codec.Vitals(flags, heart_rate, spo2, temp_ir, temp_contact)

    def _ack(self, acked_type, status, now):
        """Queue an ACK, unless fault injection says to drop or delay it."""
        if self.drop_acks > 0:
            self.drop_acks -= 1
            self.log(f'(injected) dropped ACK for type 0x{acked_type:02X}')
            return
        self._send(codec.Ack(acked_type, status), now, delay=self.ack_delay_s)

    def _send(self, message, now, delay=0.0):
        """Queue a message to be written once its due time has passed."""
        self._seq += 1
        self._outbox.append((now + delay, self._seq, message.to_frame()))

    def _flush(self, now):
        """Return the bytes of all due frames, applying silence, drops and corruption."""
        ready = sorted(item for item in self._outbox if item[0] <= now)
        self._outbox = [item for item in self._outbox if item[0] > now]
        out = bytearray()
        for _, _, frame in ready:
            if now < self.silent_until:
                continue
            if self.drop_frames > 0:
                self.drop_frames -= 1
                self.log('(injected) dropped an outgoing frame')
                continue
            if self.corrupt_next:
                self.corrupt_next = False
                frame = frame[:-1] + bytes([frame[-1] ^ 0xFF])
                self.log('(injected) corrupted the checksum of an outgoing frame')
            out += frame
        return bytes(out)

    # ------------------------------------------------------------------
    # Prompt commands
    # ------------------------------------------------------------------

    def command(self, line, now):
        """Run one prompt command and return the text to show the user."""
        parts = line.split()
        if not parts:
            return ''
        name, args = parts[0].lower(), parts[1:]
        method = getattr(self, f'_cmd_{name}', None)
        if method is None:
            return f'unknown command {name!r}, type help'
        try:
            return method(args, now) or 'ok'
        except (ValueError, IndexError, KeyError):
            return f'bad arguments for {name!r}, type help'

    def _cmd_help(self, args, now):
        """Return the help text."""
        return HELP

    def _cmd_quit(self, args, now):
        """Stop the fake."""
        self.quit_requested = True
        return 'bye'

    def _cmd_estop(self, args, now):
        """Press or release the E-stop."""
        self.estop = _on_off(args[0])

    def _cmd_range(self, args, now):
        """Set one range in mm, or 'none'."""
        side = args[0]
        value = None if args[1].lower() == 'none' else int(args[1])
        if side not in self.ranges or (value is not None and not 0 <= value < codec.NO_RANGE):
            raise ValueError(side)
        self.ranges[side] = value

    def _cmd_fault(self, args, now):
        """Start or stop a sensor fault. The flag latches until CLEAR_FAULT."""
        name = args[0]
        active = _on_off(args[1])
        flag = FAULT_FLAG_BY_NAME[name]
        self.cause[name] = active
        if active:
            self.latched |= flag

    def _cmd_finger(self, args, now):
        """Put the finger on or take it off."""
        self.finger = _on_off(args[0])

    def _cmd_hr(self, args, now):
        """Set the fake heart rate in bpm."""
        self.hr_bpm = _clamp(int(args[0]), 0, 255)

    def _cmd_spo2(self, args, now):
        """Set the fake SpO2 in percent."""
        self.spo2_pct = _clamp(int(args[0]), 0, 255)

    def _cmd_temp(self, args, now):
        """Set the fake forehead temperature in degC."""
        self.temp_c = float(args[0])

    def _cmd_battery(self, args, now):
        """Set the fake battery voltage in mV."""
        self.battery_mv = _clamp(int(args[0]), 0, 0xFFFF)

    def _cmd_open(self, args, now):
        """Open a drawer, which is only possible if it is unlocked."""
        index = int(args[0])
        if not 0 <= index <= 3:
            raise ValueError(index)
        if not self.unlocked[index]:
            return f'drawer {index} is locked, it cannot be opened'
        self.is_open[index] = True
        self.relock_deadline[index] = None

    def _cmd_close(self, args, now):
        """Close a drawer. The auto-relock timer starts now."""
        index = int(args[0])
        if not 0 <= index <= 3:
            raise ValueError(index)
        self.is_open[index] = False
        self._arm_relock(index, now)

    def _cmd_corrupt(self, args, now):
        """Corrupt the checksum of the next outgoing frame."""
        self.corrupt_next = True

    def _cmd_drop(self, args, now):
        """Drop the next N outgoing frames."""
        self.drop_frames = int(args[0]) if args else 1

    def _cmd_dropack(self, args, now):
        """Drop the next N ACK frames."""
        self.drop_acks = int(args[0]) if args else 1

    def _cmd_ackdelay(self, args, now):
        """Delay every ACK by this many seconds."""
        self.ack_delay_s = max(0.0, float(args[0]))

    def _cmd_silent(self, args, now):
        """Send nothing at all for this many seconds."""
        self.silent_until = now + max(0.0, float(args[0]))

    def _cmd_status(self, args, now):
        """Return a summary of the fake robot's state."""
        flags = self._status_flags(now)
        return '\n'.join([
            f'uptime {now - self.boot_time:.1f} s, flags: {describe_flags(flags)}',
            f'wheel speeds {self.speed[0]:.0f} / {self.speed[1]:.0f} mm/s, '
            f'ticks {self.ticks[0]:.0f} / {self.ticks[1]:.0f}',
            f'ranges {self.ranges}, battery {self.battery_mv} mV',
            f'unlocked {self.unlocked}, open {self.is_open}',
            f'measuring {self.measuring} ({describe_flags(self.mask)}), finger {self.finger}',
            f'injection: corrupt={self.corrupt_next} drop={self.drop_frames} '
            f'dropack={self.drop_acks} ackdelay={self.ack_delay_s}s',
        ])


# --------------------------------------------------------------------------
# Real-time runner
# --------------------------------------------------------------------------

def _prompt_loop(esp, lock, stop):
    """Read prompt commands from stdin until it closes or the user types quit."""
    for line in sys.stdin:
        with lock:
            reply = esp.command(line, time.monotonic())
            quit_now = esp.quit_requested
        if reply:
            print(reply)
        if quit_now or stop.is_set():
            break


def run(port, baud=115200, verbose=False, prompt=True, stop_event=None):
    """Run the fake on a serial port until quit, Ctrl-C or stop_event is set."""
    import serial  # imported here so the simulator itself does not need pyserial
    link = serial.Serial(port, baud, timeout=0)
    lock = threading.Lock()
    stop = stop_event if stop_event is not None else threading.Event()
    esp = FakeEsp32(now=time.monotonic())
    esp.verbose = verbose
    if prompt:
        threading.Thread(target=_prompt_loop, args=(esp, lock, stop), daemon=True).start()
    try:
        while not stop.is_set() and not esp.quit_requested:
            now = time.monotonic()
            data = link.read(4096)
            with lock:
                if data:
                    esp.feed(data, now)
                out = esp.tick(now)
            if out:
                link.write(out)
            time.sleep(0.002)
    finally:
        link.close()


def main(argv=None):
    """Parse the command line and run the fake."""
    parser = argparse.ArgumentParser(description='Fake ESP32 for the IsoCare serial protocol.')
    parser.add_argument('port', nargs='?', default='/tmp/esp_fake',
                        help='serial port to open (default: /tmp/esp_fake)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--verbose', action='store_true', help='also log every DRIVE_CMD')
    parser.add_argument('--no-prompt', action='store_true', help='do not read commands from stdin')
    args = parser.parse_args(argv)
    print(f'fake ESP32 on {args.port} at {args.baud} baud. Type help for commands.')
    try:
        run(args.port, args.baud, args.verbose, not args.no_prompt)
    except KeyboardInterrupt:
        print('\nstopped')


if __name__ == '__main__':
    main()
