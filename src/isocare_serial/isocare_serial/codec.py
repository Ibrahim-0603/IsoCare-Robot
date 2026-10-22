"""IsoCare serial protocol codec (draft v0.1, see docs/protocol.md).

This module is plain Python and imports nothing from ROS. That keeps it
testable with pytest alone, and lets both the serial_bridge node and
tools/fake_esp32.py use the exact same code.

Three layers, each one built on the one before:

    1. Framing:  checksum(), build_frame(), Parser
       Knows about bytes only (SYNC, TYPE, LEN, PAYLOAD, CHK), not meanings.
    2. Messages: one frozen dataclass per message, each with encode() and
       decode(). Knows what the payload bytes mean, not how frames are built.
    3. Dispatch: decode_frame() turns a (type, payload) pair from the Parser
       into the right message object.

If a byte layout changes, change docs/protocol.md, this file and the
firmware parser in the same commit.
"""

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import struct
import time
from typing import ClassVar

SYNC = 0xAA
MAX_PAYLOAD = 32
INTER_BYTE_TIMEOUT_S = 0.050
NO_RANGE = 0xFFFF  # SENSOR_DATA range value meaning "no valid reading"


class ProtocolError(ValueError):
    """Raise when a frame or payload breaks the protocol rules."""


class UnknownTypeError(ProtocolError):
    """Raise when a frame carries a message type this codec does not know."""


class MsgType(IntEnum):
    """Message type IDs: 0x01-0x7F are Pi to ESP32, 0x80-0xFF are ESP32 to Pi."""

    DRIVE_CMD = 0x01
    DRAWER_CMD = 0x02
    MEASURE_CMD = 0x03
    SYSTEM_CMD = 0x04
    ACK = 0x81
    SENSOR_DATA = 0x82
    VITALS = 0x83
    DRAWER_STATUS = 0x84
    STATUS = 0x85


class DrawerAction(IntEnum):
    """DRAWER_CMD action field."""

    LOCK = 0
    UNLOCK = 1


class MeasureAction(IntEnum):
    """MEASURE_CMD action field."""

    STOP = 0
    START = 1


class SensorMask(IntFlag):
    """MEASURE_CMD sensor_mask bits."""

    MAX30102 = 1 << 0  # heart rate and SpO2
    MLX90614 = 1 << 1  # non-contact (IR) temperature
    MAX30205 = 1 << 2  # contact temperature


class SystemCommand(IntEnum):
    """SYSTEM_CMD command values."""

    RECALIBRATE_GYRO = 0x01
    CLEAR_FAULT = 0x02
    SOFT_RESET = 0x03


class AckStatus(IntEnum):
    """ACK status values."""

    OK = 0x00
    BAD_PAYLOAD = 0x01
    UNKNOWN_TYPE = 0x02
    REFUSED_SAFETY = 0x03
    BUSY = 0x04


class VitalsFlag(IntFlag):
    """VITALS flags bits."""

    HR_VALID = 1 << 0
    SPO2_VALID = 1 << 1
    IR_TEMP_VALID = 1 << 2
    CONTACT_TEMP_VALID = 1 << 3
    FINGER_DETECTED = 1 << 4


class StatusFlag(IntFlag):
    """STATUS flags bits (bits 8 to 15 are reserved and sent as 0)."""

    ESTOP = 1 << 0
    WATCHDOG = 1 << 1
    OBST_SLOW = 1 << 2
    OBST_STOP = 1 << 3
    IMU_FAULT = 1 << 4
    VITALS_FAULT = 1 << 5
    RANGE_FAULT = 1 << 6
    GYRO_CAL = 1 << 7


# --------------------------------------------------------------------------
# Layer 1: framing
# --------------------------------------------------------------------------

def checksum(data):
    """Return the XOR of every byte in data."""
    result = 0
    for byte in data:
        result ^= byte
    return result


def build_frame(msg_type, payload=b''):
    """Wrap a payload into a complete wire frame: SYNC TYPE LEN PAYLOAD CHK.

    The checksum covers TYPE, LEN and the payload, but not SYNC.
    """
    if not 0 <= msg_type <= 0xFF:
        raise ProtocolError(f'message type {msg_type!r} does not fit in one byte')
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f'payload is {len(payload)} bytes, maximum is {MAX_PAYLOAD}')
    body = bytes([msg_type, len(payload)]) + bytes(payload)
    return bytes([SYNC]) + body + bytes([checksum(body)])


class Parser:
    """Turn a stream of received bytes into complete, checksum-valid frames.

    Feed it whatever the serial port returns, in any chunk size. It keeps the
    bytes of an unfinished frame in an internal buffer between calls.

    Recovery rule: when a candidate frame fails (bad LEN or bad checksum) only
    its first byte, the sync byte, is thrown away. The remaining bytes stay in
    the buffer and are scanned again, because a real frame may have started
    inside the failed candidate (0xAA can also legitimately appear in a payload).

    The counters are for diagnostics: a healthy link keeps them near zero.
    """

    def __init__(self, timeout_s=INTER_BYTE_TIMEOUT_S):
        """Create an empty parser; timeout_s is the longest gap allowed inside a frame."""
        self.timeout_s = timeout_s
        self._buffer = bytearray()
        self._last_rx = 0.0
        self.frames_ok = 0
        self.bad_checksum = 0
        self.bad_length = 0
        self.timeouts = 0
        self.junk_bytes = 0

    @property
    def pending_bytes(self):
        """Return how many bytes of an unfinished frame are being held."""
        return len(self._buffer)

    def feed(self, data, now=None):
        """Add newly received bytes and return every complete frame now available.

        Args:
            data: bytes just read from the port. May be empty, and may hold
                part of a frame, one frame, or several frames.
            now: current time in seconds. Defaults to time.monotonic(). Tests
                pass fake values so timeout behaviour is exact and instant.
                All bytes in one call count as arriving at time now.

        Returns:
            A list of (msg_type, payload) tuples, possibly empty. The payload
            is raw bytes: the Parser does not know what messages mean.

        Calling feed(b'', now) is allowed and expires a stale partial frame.
        """
        if now is None:
            now = time.monotonic()
        # Rule 5: a partial frame that has been silent too long is dead.
        if self._buffer and now - self._last_rx > self.timeout_s:
            self.timeouts += 1
            self._buffer.clear()
        if data:
            self._buffer.extend(data)
            self._last_rx = now
        return self._extract()

    def _extract(self):
        """Pull every complete valid frame out of the buffer."""
        frames = []
        buf = self._buffer
        while True:
            # Rule 1: throw away everything before the next sync byte.
            start = buf.find(SYNC)
            if start == -1:
                self.junk_bytes += len(buf)
                buf.clear()
                return frames
            if start > 0:
                self.junk_bytes += start
                del buf[:start]
            # buf[0] is SYNC. TYPE and LEN are needed to know the frame size.
            if len(buf) < 3:
                return frames
            length = buf[2]
            if length > MAX_PAYLOAD:
                # Rule 2: corrupt LEN. Drop only the sync byte and rescan.
                self.bad_length += 1
                del buf[0]
                continue
            total = length + 4
            if len(buf) < total:
                return frames  # frame not complete yet, wait for more bytes
            if checksum(buf[1:total - 1]) != buf[total - 1]:
                # Rule 4: drop only the sync byte. The rest stays in the
                # buffer and is rescanned on the next pass of this loop.
                self.bad_checksum += 1
                del buf[0]
                continue
            frames.append((buf[1], bytes(buf[3:3 + length])))
            self.frames_ok += 1
            del buf[:total]


# --------------------------------------------------------------------------
# Layer 2: messages
# --------------------------------------------------------------------------

def _check_range(name, value, low, high):
    """Raise ProtocolError unless value is an int within low..high."""
    if not isinstance(value, int) or not low <= value <= high:
        raise ProtocolError(f'{name}={value!r} must be an int in {low}..{high}')


def _to_enum(enum_cls, value, name):
    """Return value converted to enum_cls, or raise ProtocolError."""
    try:
        return enum_cls(value)
    except ValueError:
        raise ProtocolError(f'{name}={value!r} is not a valid {enum_cls.__name__}') from None


def _unpack(fmt, payload, name):
    """Unpack a payload with a struct format, checking its exact length first."""
    expected = struct.calcsize(fmt)
    if len(payload) != expected:
        raise ProtocolError(f'{name} payload is {len(payload)} bytes, expected {expected}')
    return struct.unpack(fmt, payload)


class _Message:
    """Base class: every message has a TYPE and can wrap itself in a frame."""

    TYPE: ClassVar[int]

    def to_frame(self):
        """Return this message as a complete wire frame."""
        return build_frame(self.TYPE, self.encode())


@dataclass(frozen=True)
class DriveCmd(_Message):
    """Pi to ESP32 (0x01): wheel speeds in mm/s, positive rolls forward."""

    TYPE: ClassVar[int] = MsgType.DRIVE_CMD

    left_mm_s: int
    right_mm_s: int

    def __post_init__(self):
        """Validate the ranges. Round float speeds to int before building."""
        _check_range('left_mm_s', self.left_mm_s, -32768, 32767)
        _check_range('right_mm_s', self.right_mm_s, -32768, 32767)

    def encode(self):
        """Return the 4-byte payload."""
        return struct.pack('>hh', self.left_mm_s, self.right_mm_s)

    @classmethod
    def decode(cls, payload):
        """Build a DriveCmd from a 4-byte payload."""
        left, right = _unpack('>hh', payload, cls.__name__)
        return cls(left, right)


@dataclass(frozen=True)
class DrawerCmd(_Message):
    """Pi to ESP32 (0x02): lock or unlock a drawer (0 to 2) or the lid (3)."""

    TYPE: ClassVar[int] = MsgType.DRAWER_CMD

    drawer_id: int
    action: DrawerAction
    relock_s: int  # auto-relock delay once closed; 0 = stay unlocked

    def __post_init__(self):
        """Validate the ranges and convert action to a DrawerAction."""
        _check_range('drawer_id', self.drawer_id, 0, 3)
        object.__setattr__(self, 'action', _to_enum(DrawerAction, self.action, 'action'))
        _check_range('relock_s', self.relock_s, 0, 255)

    def encode(self):
        """Return the 3-byte payload."""
        return struct.pack('>BBB', self.drawer_id, self.action, self.relock_s)

    @classmethod
    def decode(cls, payload):
        """Build a DrawerCmd from a 3-byte payload."""
        drawer_id, action, relock_s = _unpack('>BBB', payload, cls.__name__)
        return cls(drawer_id, action, relock_s)


@dataclass(frozen=True)
class MeasureCmd(_Message):
    """Pi to ESP32 (0x03): start or stop vitals measurement on chosen sensors."""

    TYPE: ClassVar[int] = MsgType.MEASURE_CMD

    action: MeasureAction
    sensor_mask: SensorMask

    def __post_init__(self):
        """Validate the fields and convert them to their enum types."""
        object.__setattr__(self, 'action', _to_enum(MeasureAction, self.action, 'action'))
        _check_range('sensor_mask', self.sensor_mask, 0, 0x07)
        object.__setattr__(self, 'sensor_mask', SensorMask(self.sensor_mask))

    def encode(self):
        """Return the 2-byte payload."""
        return struct.pack('>BB', self.action, self.sensor_mask)

    @classmethod
    def decode(cls, payload):
        """Build a MeasureCmd from a 2-byte payload."""
        action, sensor_mask = _unpack('>BB', payload, cls.__name__)
        return cls(action, sensor_mask)


@dataclass(frozen=True)
class SystemCmd(_Message):
    """Pi to ESP32 (0x04): recalibrate gyro, clear fault latch, or soft reset."""

    TYPE: ClassVar[int] = MsgType.SYSTEM_CMD

    command: SystemCommand

    def __post_init__(self):
        """Convert command to a SystemCommand, rejecting unknown values."""
        object.__setattr__(self, 'command', _to_enum(SystemCommand, self.command, 'command'))

    def encode(self):
        """Return the 1-byte payload."""
        return struct.pack('>B', self.command)

    @classmethod
    def decode(cls, payload):
        """Build a SystemCmd from a 1-byte payload."""
        (command,) = _unpack('>B', payload, cls.__name__)
        return cls(command)


@dataclass(frozen=True)
class Ack(_Message):
    """ESP32 to Pi (0x81): reply to an ACKed command (0x02, 0x03, 0x04)."""

    TYPE: ClassVar[int] = MsgType.ACK

    acked_type: int
    status: AckStatus

    def __post_init__(self):
        """Validate acked_type and convert status to an AckStatus."""
        _check_range('acked_type', self.acked_type, 0, 0xFF)
        object.__setattr__(self, 'status', _to_enum(AckStatus, self.status, 'status'))

    def encode(self):
        """Return the 2-byte payload."""
        return struct.pack('>BB', self.acked_type, self.status)

    @classmethod
    def decode(cls, payload):
        """Build an Ack from a 2-byte payload."""
        acked_type, status = _unpack('>BB', payload, cls.__name__)
        return cls(acked_type, status)


@dataclass(frozen=True)
class SensorData(_Message):
    """ESP32 to Pi (0x82): raw navigation data at 50 Hz.

    Ticks are cumulative since boot. gyro_z is in 0.01 deg/s, positive
    counter-clockwise. A range of None means "no valid reading" and travels
    on the wire as 0xFFFF.
    """

    TYPE: ClassVar[int] = MsgType.SENSOR_DATA

    left_ticks: int
    right_ticks: int
    t_ms: int
    gyro_z: int
    range_front_mm: int | None
    range_left_mm: int | None
    range_right_mm: int | None

    _FMT: ClassVar[str] = '>iiIhHHH'

    def __post_init__(self):
        """Validate every field. A real range may not equal the 0xFFFF marker."""
        _check_range('left_ticks', self.left_ticks, -2**31, 2**31 - 1)
        _check_range('right_ticks', self.right_ticks, -2**31, 2**31 - 1)
        _check_range('t_ms', self.t_ms, 0, 2**32 - 1)
        _check_range('gyro_z', self.gyro_z, -32768, 32767)
        for name in ('range_front_mm', 'range_left_mm', 'range_right_mm'):
            value = getattr(self, name)
            if value is not None:
                _check_range(name, value, 0, NO_RANGE - 1)

    @property
    def gyro_z_deg_s(self):
        """Return the yaw rate in degrees per second."""
        return self.gyro_z / 100.0

    def encode(self):
        """Return the 20-byte payload."""
        return struct.pack(
            self._FMT, self.left_ticks, self.right_ticks, self.t_ms, self.gyro_z,
            _range_to_wire(self.range_front_mm),
            _range_to_wire(self.range_left_mm),
            _range_to_wire(self.range_right_mm))

    @classmethod
    def decode(cls, payload):
        """Build a SensorData from a 20-byte payload."""
        fields = _unpack(cls._FMT, payload, cls.__name__)
        left_ticks, right_ticks, t_ms, gyro_z, front, left, right = fields
        return cls(left_ticks, right_ticks, t_ms, gyro_z,
                   _range_from_wire(front), _range_from_wire(left), _range_from_wire(right))


def _range_to_wire(value):
    """Return the wire value for a range: None becomes 0xFFFF."""
    return NO_RANGE if value is None else value


def _range_from_wire(value):
    """Return the range in mm, or None for the 0xFFFF marker."""
    return None if value == NO_RANGE else value


@dataclass(frozen=True)
class Vitals(_Message):
    """ESP32 to Pi (0x83): vitals sample at 1 Hz while measuring.

    The fields hold the raw wire values. A value is only usable if its valid
    flag is set, so use the *_bpm / *_percent / *_c properties, which return
    None for anything not flagged valid. Zero never means "measured zero".
    """

    TYPE: ClassVar[int] = MsgType.VITALS

    flags: VitalsFlag
    heart_rate: int  # bpm
    spo2: int  # percent
    temp_ir: int  # 0.01 degC, MLX90614 forehead (primary)
    temp_contact: int  # 0.01 degC, MAX30205 (cross-check)

    def __post_init__(self):
        """Validate the ranges and convert flags to a VitalsFlag."""
        _check_range('flags', self.flags, 0, 0xFF)
        object.__setattr__(self, 'flags', VitalsFlag(self.flags))
        _check_range('heart_rate', self.heart_rate, 0, 255)
        _check_range('spo2', self.spo2, 0, 255)
        _check_range('temp_ir', self.temp_ir, -32768, 32767)
        _check_range('temp_contact', self.temp_contact, -32768, 32767)

    @property
    def heart_rate_bpm(self):
        """Return the heart rate in bpm, or None if HR_VALID is not set."""
        return self.heart_rate if VitalsFlag.HR_VALID in self.flags else None

    @property
    def spo2_percent(self):
        """Return SpO2 in percent, or None if SPO2_VALID is not set."""
        return self.spo2 if VitalsFlag.SPO2_VALID in self.flags else None

    @property
    def temp_ir_c(self):
        """Return the IR temperature in degC, or None if IR_TEMP_VALID is not set."""
        return self.temp_ir / 100.0 if VitalsFlag.IR_TEMP_VALID in self.flags else None

    @property
    def temp_contact_c(self):
        """Return the contact temperature in degC, or None if it is not valid."""
        if VitalsFlag.CONTACT_TEMP_VALID in self.flags:
            return self.temp_contact / 100.0
        return None

    def encode(self):
        """Return the 7-byte payload."""
        return struct.pack('>BBBhh', self.flags, self.heart_rate, self.spo2,
                           self.temp_ir, self.temp_contact)

    @classmethod
    def decode(cls, payload):
        """Build a Vitals from a 7-byte payload."""
        flags, heart_rate, spo2, temp_ir, temp_contact = _unpack('>BBBhh', payload, cls.__name__)
        return cls(flags, heart_rate, spo2, temp_ir, temp_contact)


@dataclass(frozen=True)
class DrawerStatus(_Message):
    """ESP32 to Pi (0x84): which drawers are unlocked and which are open.

    Bits 0 to 2 are the drawers, bit 3 is the lid.
    """

    TYPE: ClassVar[int] = MsgType.DRAWER_STATUS

    unlocked_mask: int
    open_mask: int

    def __post_init__(self):
        """Validate that both masks fit in one byte."""
        _check_range('unlocked_mask', self.unlocked_mask, 0, 0xFF)
        _check_range('open_mask', self.open_mask, 0, 0xFF)

    def is_unlocked(self, drawer_id):
        """Return True if the drawer (0 to 2) or lid (3) is unlocked."""
        return bool((self.unlocked_mask >> drawer_id) & 1)

    def is_open(self, drawer_id):
        """Return True if the drawer (0 to 2) or lid (3) limit switch reads open."""
        return bool((self.open_mask >> drawer_id) & 1)

    def encode(self):
        """Return the 2-byte payload."""
        return struct.pack('>BB', self.unlocked_mask, self.open_mask)

    @classmethod
    def decode(cls, payload):
        """Build a DrawerStatus from a 2-byte payload."""
        unlocked_mask, open_mask = _unpack('>BB', payload, cls.__name__)
        return cls(unlocked_mask, open_mask)


@dataclass(frozen=True)
class Status(_Message):
    """ESP32 to Pi (0x85): fault and safety flags plus battery voltage in mV."""

    TYPE: ClassVar[int] = MsgType.STATUS

    flags: StatusFlag
    battery_mv: int  # 0 if no voltage sensing is fitted

    def __post_init__(self):
        """Validate the ranges and convert flags to a StatusFlag."""
        _check_range('flags', self.flags, 0, 0xFFFF)
        object.__setattr__(self, 'flags', StatusFlag(self.flags))
        _check_range('battery_mv', self.battery_mv, 0, 0xFFFF)

    def encode(self):
        """Return the 4-byte payload."""
        return struct.pack('>HH', self.flags, self.battery_mv)

    @classmethod
    def decode(cls, payload):
        """Build a Status from a 4-byte payload."""
        flags, battery_mv = _unpack('>HH', payload, cls.__name__)
        return cls(flags, battery_mv)


# --------------------------------------------------------------------------
# Layer 3: dispatch
# --------------------------------------------------------------------------

_MESSAGE_CLASSES = {
    cls.TYPE: cls
    for cls in (DriveCmd, DrawerCmd, MeasureCmd, SystemCmd,
                Ack, SensorData, Vitals, DrawerStatus, Status)
}


def decode_frame(msg_type, payload):
    """Turn a (type, payload) pair from the Parser into a message object.

    Raises UnknownTypeError if the type is not in the protocol, and
    ProtocolError if the payload does not fit the type.
    """
    try:
        message_cls = _MESSAGE_CLASSES[msg_type]
    except KeyError:
        raise UnknownTypeError(f'unknown message type 0x{msg_type:02X}') from None
    return message_cls.decode(payload)
