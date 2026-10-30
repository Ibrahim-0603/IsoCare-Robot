"""ROS-free brain of serial_bridge.

The node in serial_bridge.py only moves data between ROS and the serial port.
Every decision lives here, so it can be tested with plain pytest and against
tools/fake_esp32.py in simulated time. Nothing in this module reads a clock or
touches a port: callers pass 'now' (monotonic seconds) and exchange bytes.

Pieces:
    twist_to_wheels  convert a Twist into left/right wheel speeds
    CmdVelHold       remember the last command, send zeros if it goes stale
    LinkMonitor      is the ESP32 still talking? how jittery is the link?
    AckTracker       commands waiting for an ACK, with resends and timeout
    BridgeCore       ties them together with the codec
    *_fields()       convert decoded messages to ROS message field values
"""

from collections import deque
from dataclasses import dataclass
import math
import threading

from isocare_serial import codec

STATUS_TIMEOUT = 255  # made up by the bridge: no ACK arrived after all resends


@dataclass
class BridgeConfig:
    """Tunable settings. The node fills these from ROS parameters."""

    wheelbase_mm: float = 150.0      # placeholder until the chassis is measured
    max_speed_mm_s: float = 500.0    # wheel speed limit; both wheels scale together
    cmd_vel_stale_s: float = 0.3     # no /cmd_vel for this long: send zeros
    link_lost_s: float = 0.2         # no SENSOR_DATA for this long: link lost
    ack_timeout_s: float = 0.2       # wait this long for an ACK before resending
    max_resends: int = 3             # resends after the first send
    drive_period_s: float = 0.05     # DRIVE_CMD at 20 Hz


def twist_to_wheels(linear_m_s, angular_rad_s, wheelbase_mm, max_speed_mm_s):
    """Convert a Twist to (left, right) wheel speeds in whole mm/s.

    Positive angular speed is counter-clockwise, so the right wheel runs faster.
    If either wheel would exceed max_speed_mm_s, both are scaled down together
    so the robot keeps the same curve. Non-finite input gives (0, 0).
    """
    if not (math.isfinite(linear_m_s) and math.isfinite(angular_rad_s)):
        return 0, 0
    forward = linear_m_s * 1000.0
    half_turn = angular_rad_s * wheelbase_mm / 2.0
    left, right = forward - half_turn, forward + half_turn
    limit = min(max_speed_mm_s, 32767.0)
    fastest = max(abs(left), abs(right))
    if fastest > limit:
        scale = limit / fastest
        left, right = left * scale, right * scale
    return int(round(left)), int(round(right))


class CmdVelHold:
    """Remember the latest wheel command and report zeros once it is too old."""

    def __init__(self, stale_s):
        """Create an empty hold; nothing has been commanded yet."""
        self.stale_s = stale_s
        self._wheels = (0, 0)
        self._time = None

    def update(self, left, right, now):
        """Store a new wheel command received at time now."""
        self._wheels = (left, right)
        self._time = now

    def is_stale(self, now):
        """Return True if no command has arrived within stale_s."""
        return self._time is None or now - self._time > self.stale_s

    def current(self, now):
        """Return the wheel speeds to send now: the last command, or (0, 0) if stale."""
        return (0, 0) if self.is_stale(now) else self._wheels


class LinkMonitor:
    """Watch SENSOR_DATA to decide if the link is alive and how much it jitters.

    The ESP32 clock and the Pi clock started at different moments, so their
    absolute difference means nothing. What does mean something is how that
    difference changes: if a frame arrives later than the best recent case,
    the excess is delay on the link.
    """

    WINDOW = 250     # samples kept, about 5 s at 50 Hz
    MIN_SAMPLES = 10

    def __init__(self, link_lost_s):
        """Create a monitor that has not heard from the ESP32 yet."""
        self.link_lost_s = link_lost_s
        self.reboots = 0
        self._last_rx = None
        self._last_t_ms = None
        self._offsets = deque(maxlen=self.WINDOW)
        self.jitter_ms = math.nan

    def on_sensor_data(self, data, now):
        """Record one SENSOR_DATA arrival."""
        self._last_rx = now
        if self._last_t_ms is not None and data.t_ms < self._last_t_ms:
            self.reboots += 1            # the ESP32 clock went backwards: it rebooted
            self._offsets.clear()
            self.jitter_ms = math.nan
        self._last_t_ms = data.t_ms
        offset = now - data.t_ms / 1000.0
        self._offsets.append(offset)
        if len(self._offsets) >= self.MIN_SAMPLES:
            excess_ms = (offset - min(self._offsets)) * 1000.0
            if math.isnan(self.jitter_ms):
                self.jitter_ms = excess_ms
            else:
                self.jitter_ms = 0.9 * self.jitter_ms + 0.1 * excess_ms

    def link_ok(self, now):
        """Return True if SENSOR_DATA arrived within the last link_lost_s."""
        return self._last_rx is not None and now - self._last_rx <= self.link_lost_s


class RequestInFlight(Exception):
    """Raise when a command of the same type is still waiting for its ACK."""


class PendingRequest:
    """One command waiting for its ACK. The submitting thread waits on it."""

    def __init__(self, msg_type, frame):
        """Create a request that has not been sent yet."""
        self.msg_type = int(msg_type)
        self.frame = frame
        self.sent_at = None    # time of the latest send; None until first written
        self.sends = 0
        self.status = None     # an AckStatus value, or STATUS_TIMEOUT
        self._done = threading.Event()

    def complete(self, status):
        """Store the final status and wake whoever is waiting."""
        self.status = status
        self._done.set()

    def wait(self, timeout=None):
        """Block until the request completes. Return False if timeout passed first."""
        return self._done.wait(timeout)

    @property
    def done(self):
        """Return True once the request has a final status."""
        return self._done.is_set()

    @property
    def success(self):
        """Return True only if the ESP32 answered OK."""
        return self.status == codec.AckStatus.OK


class AckTracker:
    """Track commands waiting for an ACK: send, resend on silence, give up.

    An ACK carries only the command type, so only one request per type can be
    in flight; submit() refuses a second one. Resending is safe because every
    command sets a state instead of toggling it.
    """

    def __init__(self, ack_timeout_s=0.2, max_resends=3):
        """Create an empty tracker."""
        self.ack_timeout_s = ack_timeout_s
        self.max_resends = max_resends
        self.last_rtt_s = None
        self.unmatched_acks = 0
        self._pending = {}
        self._lock = threading.Lock()

    def submit(self, msg_type, frame):
        """Register a command. Its frame is written by the next poll()."""
        with self._lock:
            if int(msg_type) in self._pending:
                raise RequestInFlight(f'type 0x{int(msg_type):02X} is still waiting for an ACK')
            request = PendingRequest(msg_type, frame)
            self._pending[request.msg_type] = request
            return request

    def poll(self, now):
        """Return the frames to write now: first sends and resends. Finish timed-out requests."""
        frames = []
        with self._lock:
            for request in list(self._pending.values()):
                if request.sent_at is None:
                    request.sent_at = now
                    request.sends = 1
                    frames.append(request.frame)
                elif now - request.sent_at >= self.ack_timeout_s - 1e-9:   # tolerate float error
                    if request.sends > self.max_resends:
                        self._finish(request, STATUS_TIMEOUT)
                    else:
                        request.sent_at = now
                        request.sends += 1
                        frames.append(request.frame)
        return frames

    def on_ack(self, ack, now):
        """Match an ACK to its request. Return False if nothing was waiting for it."""
        with self._lock:
            request = self._pending.get(ack.acked_type)
            if request is None:
                self.unmatched_acks += 1
                return False
            if request.sends == 1 and request.sent_at is not None:
                self.last_rtt_s = now - request.sent_at   # ambiguous after a resend: skip
            self._finish(request, ack.status)
            return True

    def cancel(self, request):
        """Abandon a request, for example when the waiting service gave up."""
        with self._lock:
            if self._pending.get(request.msg_type) is request:
                self._finish(request, STATUS_TIMEOUT)

    def _finish(self, request, status):
        """Remove a request from the table and wake its waiter. Caller holds the lock."""
        del self._pending[request.msg_type]
        request.complete(status)


@dataclass
class LinkSnapshot:
    """Link health numbers for the LinkStats message."""

    link_ok: bool
    frames_ok: int
    bad_checksum: int
    bad_length: int
    timeouts: int
    junk_bytes: int
    ack_rtt_ms: float
    esp_clock_jitter_ms: float


class BridgeCore:
    """All of serial_bridge's decisions, with no ROS, port or clock.

    Threads: set_cmd_vel() and submit() may be called from ROS callbacks, while
    feed() and poll() belong to the single I/O loop that owns the serial port.
    """

    def __init__(self, config=None):
        """Create a core with the given BridgeConfig (defaults if None)."""
        self.config = config if config is not None else BridgeConfig()
        self.parser = codec.Parser()
        self.hold = CmdVelHold(self.config.cmd_vel_stale_s)
        self.monitor = LinkMonitor(self.config.link_lost_s)
        self.tracker = AckTracker(self.config.ack_timeout_s, self.config.max_resends)
        self.decode_errors = 0
        self.unexpected_frames = 0
        self._lock = threading.Lock()
        self._next_drive = None

    def set_cmd_vel(self, linear_m_s, angular_rad_s, now):
        """Store the newest /cmd_vel as wheel speeds."""
        left, right = twist_to_wheels(linear_m_s, angular_rad_s,
                                      self.config.wheelbase_mm, self.config.max_speed_mm_s)
        with self._lock:
            self.hold.update(left, right, now)

    def submit(self, message):
        """Queue a DrawerCmd, MeasureCmd or SystemCmd. Wait on the returned request."""
        return self.tracker.submit(message.TYPE, message.to_frame())

    def cancel(self, request):
        """Abandon a submitted request."""
        self.tracker.cancel(request)

    def link_ok(self, now):
        """Return True if the ESP32 is still sending SENSOR_DATA."""
        return self.monitor.link_ok(now)

    def feed(self, data, now):
        """Process bytes read from the port. Return the ESP32 messages to publish."""
        messages = []
        for msg_type, payload in self.parser.feed(data, now):
            if msg_type < 0x80:
                self.unexpected_frames += 1   # a command type coming from the ESP32
                continue
            try:
                message = codec.decode_frame(msg_type, payload)
            except codec.ProtocolError:
                self.decode_errors += 1
                continue
            if isinstance(message, codec.Ack):
                self.tracker.on_ack(message, now)
                continue
            if isinstance(message, codec.SensorData):
                self.monitor.on_sensor_data(message, now)
            messages.append(message)
        return messages

    def poll(self, now):
        """Return the bytes to write to the port now."""
        out = bytearray()
        for frame in self.tracker.poll(now):
            out += frame
        if self.monitor.link_ok(now) and self._drive_due(now):
            with self._lock:
                left, right = self.hold.current(now)
            out += codec.DriveCmd(left, right).to_frame()
        return bytes(out)

    def _drive_due(self, now):
        """Return True if a DRIVE_CMD is due, and schedule the next one at 20 Hz."""
        period = self.config.drive_period_s
        if self._next_drive is not None and now < self._next_drive:
            return False
        following = (self._next_drive if self._next_drive is not None else now) + period
        self._next_drive = following if following > now else now + period
        return True

    def snapshot(self, now):
        """Return the current link health numbers."""
        rtt = self.tracker.last_rtt_s
        return LinkSnapshot(
            link_ok=self.link_ok(now),
            frames_ok=self.parser.frames_ok,
            bad_checksum=self.parser.bad_checksum,
            bad_length=self.parser.bad_length,
            timeouts=self.parser.timeouts,
            junk_bytes=self.parser.junk_bytes,
            ack_rtt_ms=math.nan if rtt is None else rtt * 1000.0,
            esp_clock_jitter_ms=self.monitor.jitter_ms)


# --------------------------------------------------------------------------
# Codec messages -> ROS message field values
# --------------------------------------------------------------------------

def _mm(value):
    """Return a range as float mm, with NaN for no reading."""
    return math.nan if value is None else float(value)


def sensor_data_fields(message):
    """Return the isocare_interfaces/SensorData field values for a codec SensorData."""
    return {
        'esp_time_ms': message.t_ms,
        'left_ticks': message.left_ticks,
        'right_ticks': message.right_ticks,
        'gyro_z_centideg_s': message.gyro_z,
        'range_front_mm': _mm(message.range_front_mm),
        'range_left_mm': _mm(message.range_left_mm),
        'range_right_mm': _mm(message.range_right_mm),
    }


def vitals_fields(message):
    """Return the isocare_interfaces/Vitals field values for a codec Vitals."""
    flag = codec.VitalsFlag
    return {
        'hr_valid': flag.HR_VALID in message.flags,
        'spo2_valid': flag.SPO2_VALID in message.flags,
        'ir_temp_valid': flag.IR_TEMP_VALID in message.flags,
        'contact_temp_valid': flag.CONTACT_TEMP_VALID in message.flags,
        'finger_detected': flag.FINGER_DETECTED in message.flags,
        'heart_rate_bpm': message.heart_rate,
        'spo2_percent': message.spo2,
        'temp_ir_centi_c': message.temp_ir,
        'temp_contact_centi_c': message.temp_contact,
    }


def drawer_status_fields(message):
    """Return the isocare_interfaces/DrawerStatus field values for a codec DrawerStatus."""
    return {
        'unlocked': [message.is_unlocked(i) for i in range(4)],
        'open': [message.is_open(i) for i in range(4)],
    }


def esp_status_fields(message):
    """Return the isocare_interfaces/EspStatus field values for a codec Status."""
    flag = codec.StatusFlag
    return {
        'estop': flag.ESTOP in message.flags,
        'watchdog': flag.WATCHDOG in message.flags,
        'obstacle_slow': flag.OBST_SLOW in message.flags,
        'obstacle_stop': flag.OBST_STOP in message.flags,
        'imu_fault': flag.IMU_FAULT in message.flags,
        'vitals_fault': flag.VITALS_FAULT in message.flags,
        'range_fault': flag.RANGE_FAULT in message.flags,
        'gyro_cal': flag.GYRO_CAL in message.flags,
        'battery_mv': message.battery_mv,
    }


def link_stats_fields(snapshot):
    """Return the isocare_interfaces/LinkStats field values for a LinkSnapshot."""
    return {
        'link_ok': snapshot.link_ok,
        'frames_ok': snapshot.frames_ok,
        'bad_checksum': snapshot.bad_checksum,
        'bad_length': snapshot.bad_length,
        'timeouts': snapshot.timeouts,
        'junk_bytes': snapshot.junk_bytes,
        'ack_rtt_ms': snapshot.ack_rtt_ms,
        'esp_clock_jitter_ms': snapshot.esp_clock_jitter_ms,
    }


def message_fields(message):
    """Return (kind, fields) for a message to publish, or (None, {}) if it has no topic.

    kind is 'sensor_data', 'vitals', 'drawer_status' or 'esp_status'.
    """
    if isinstance(message, codec.SensorData):
        return 'sensor_data', sensor_data_fields(message)
    if isinstance(message, codec.Vitals):
        return 'vitals', vitals_fields(message)
    if isinstance(message, codec.DrawerStatus):
        return 'drawer_status', drawer_status_fields(message)
    if isinstance(message, codec.Status):
        return 'esp_status', esp_status_fields(message)
    return None, {}
