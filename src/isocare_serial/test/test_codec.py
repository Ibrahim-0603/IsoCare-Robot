"""Tests for isocare_serial.codec. Plain pytest, no ROS needed."""

import random

from isocare_serial import codec
import pytest

DRIVE_FRAME = bytes.fromhex('AA 01 04 00 C8 00 C8 05')
DRAWER_FRAME = bytes.fromhex('AA 02 03 01 01 1E 1F')
ACK_FRAME = bytes.fromhex('AA 81 02 02 00 81')


# --------------------------------------------------------------------------
# Layer 1: framing
# --------------------------------------------------------------------------

def test_checksum_matches_protocol_section_9():
    """The three XOR checksums quoted in protocol.md section 9."""
    assert codec.checksum(bytes.fromhex('01 04 00 C8 00 C8')) == 0x05
    assert codec.checksum(bytes.fromhex('02 03 01 01 1E')) == 0x1F
    assert codec.checksum(bytes.fromhex('81 02 02 00')) == 0x81


def test_build_frame_matches_vectors():
    """build_frame reproduces the hand-built frames from protocol.md."""
    assert codec.build_frame(0x01, bytes.fromhex('00 C8 00 C8')) == DRIVE_FRAME
    assert codec.build_frame(0x02, bytes.fromhex('01 01 1E')) == DRAWER_FRAME
    assert codec.build_frame(0x81, bytes.fromhex('02 00')) == ACK_FRAME


def test_build_frame_rejects_bad_input():
    """Oversize payloads and out-of-range types raise ProtocolError."""
    with pytest.raises(codec.ProtocolError):
        codec.build_frame(0x01, bytes(33))
    with pytest.raises(codec.ProtocolError):
        codec.build_frame(256, b'')
    assert len(codec.build_frame(0x01, bytes(32))) == 36


@pytest.mark.parametrize('frame, msg_type, payload', [
    (DRIVE_FRAME, 0x01, bytes.fromhex('00 C8 00 C8')),
    (DRAWER_FRAME, 0x02, bytes.fromhex('01 01 1E')),
    (ACK_FRAME, 0x81, bytes.fromhex('02 00')),
])
def test_parser_reads_vectors(frame, msg_type, payload):
    """The Parser extracts each protocol vector as (type, payload)."""
    parser = codec.Parser()
    assert parser.feed(frame, now=0.0) == [(msg_type, payload)]
    assert parser.frames_ok == 1
    assert parser.pending_bytes == 0


def test_bad_checksum_gives_nothing():
    """A corrupted checksum produces no frame (the ESP32 would not reply either)."""
    corrupted = bytearray(DRIVE_FRAME)
    corrupted[-1] ^= 0xFF
    parser = codec.Parser()
    assert parser.feed(bytes(corrupted), now=0.0) == []
    assert parser.bad_checksum == 1
    assert parser.frames_ok == 0


def test_aa_inside_payload_is_not_a_new_frame():
    """0xAA bytes inside a payload must not confuse the Parser."""
    payload = bytes([0xAA, 0x01, 0xAA, 0xAA])
    frame = codec.build_frame(0x82, payload)
    parser = codec.Parser()
    assert parser.feed(frame, now=0.0) == [(0x82, payload)]
    assert parser.bad_checksum == 0


def test_resync_after_false_start():
    """Noise that looks like a frame start must not swallow the real frame behind it."""
    noise = bytes.fromhex('AA 05 03')  # claims LEN=3, so it would eat the next frame's head
    parser = codec.Parser()
    assert parser.feed(noise + ACK_FRAME, now=0.0) == [(0x81, bytes.fromhex('02 00'))]
    assert parser.bad_checksum == 1


def test_resync_after_corrupt_frame_with_aa_in_payload():
    """A corrupt frame whose payload holds 0xAA is skipped; the next good frame survives."""
    bad = bytearray(codec.build_frame(0x02, bytes([0xAA, 0x00, 0x00])))
    bad[-1] ^= 0xFF
    parser = codec.Parser()
    assert parser.feed(bytes(bad) + DRIVE_FRAME, now=0.0) == [(0x01, bytes.fromhex('00 C8 00 C8'))]


def test_truncated_frame_times_out():
    """Half a frame, then a gap longer than 50 ms, is dropped."""
    parser = codec.Parser()
    assert parser.feed(DRIVE_FRAME[:5], now=0.000) == []
    assert parser.pending_bytes == 5
    assert parser.feed(DRIVE_FRAME[5:], now=0.060) == []
    assert parser.timeouts == 1
    assert parser.frames_ok == 0


def test_split_frame_within_timeout_is_delivered():
    """The same split with a short gap is delivered normally."""
    parser = codec.Parser()
    assert parser.feed(DRIVE_FRAME[:5], now=0.000) == []
    assert parser.feed(DRIVE_FRAME[5:], now=0.010) == [(0x01, bytes.fromhex('00 C8 00 C8'))]
    assert parser.timeouts == 0


def test_stale_partial_does_not_block_next_frame():
    """After a timeout the Parser starts clean on the next frame."""
    parser = codec.Parser()
    parser.feed(DRIVE_FRAME[:5], now=0.0)
    assert parser.feed(ACK_FRAME, now=0.2) == [(0x81, bytes.fromhex('02 00'))]
    assert parser.timeouts == 1


def test_empty_feed_expires_partial_frame():
    """feed(b'', now) is a way to expire a stale partial frame."""
    parser = codec.Parser()
    parser.feed(DRIVE_FRAME[:5], now=0.0)
    assert parser.feed(b'', now=1.0) == []
    assert parser.pending_bytes == 0
    assert parser.timeouts == 1


def test_len_33_is_rejected_and_rescanned():
    """LEN above 32 means a corrupt frame: drop the sync byte and rescan."""
    parser = codec.Parser()
    assert parser.feed(bytes([0xAA, 0x01, 33]), now=0.0) == []
    assert parser.bad_length == 1
    assert parser.pending_bytes == 0
    # The same corrupt header directly in front of a good frame: the good one survives.
    bad_header = bytes([0xAA, 0x01, 33])
    assert parser.feed(bad_header + DRIVE_FRAME, now=1.0) == [(0x01, bytes.fromhex('00 C8 00 C8'))]
    assert parser.bad_length == 2


@pytest.mark.parametrize('frame', [DRIVE_FRAME, DRAWER_FRAME, ACK_FRAME])
def test_one_byte_at_a_time_equals_whole_frame(frame):
    """Chunk boundaries must not matter."""
    whole = codec.Parser().feed(frame, now=0.0)
    parser = codec.Parser()
    pieces = []
    for index, byte in enumerate(frame):
        pieces += parser.feed(bytes([byte]), now=index * 0.001)
    assert pieces == whole
    assert len(pieces) == 1


def test_two_frames_in_one_feed():
    """One read can contain several frames."""
    parser = codec.Parser()
    frames = parser.feed(DRIVE_FRAME + ACK_FRAME, now=0.0)
    assert frames == [(0x01, bytes.fromhex('00 C8 00 C8')), (0x81, bytes.fromhex('02 00'))]


def test_junk_before_sync_is_counted_and_dropped():
    """Bytes before the first 0xAA are discarded."""
    parser = codec.Parser()
    assert parser.feed(bytes([1, 2, 3]) + ACK_FRAME, now=0.0) == [(0x81, bytes.fromhex('02 00'))]
    assert parser.junk_bytes == 3


def test_parser_survives_random_noise():
    """100 000 random bytes in random chunks: no exception, bounded buffer."""
    rng = random.Random(1234)
    parser = codec.Parser()
    now = 0.0
    sent = 0
    while sent < 100_000:
        size = rng.randint(1, 64)
        chunk = bytes(rng.getrandbits(8) for _ in range(size))
        now += rng.choice([0.0, 0.001, 0.1])
        parser.feed(chunk, now=now)
        sent += size
        assert parser.pending_bytes <= codec.MAX_PAYLOAD + 3


def test_noise_without_sync_between_frames_loses_nothing():
    """Valid frames separated by noise that has no 0xAA are all recovered."""
    rng = random.Random(99)
    expected = []
    stream = bytearray()
    for _ in range(500):
        payload = bytes(rng.randrange(256) for _ in range(rng.randint(0, 32)))
        msg_type = rng.randrange(256)
        stream += bytes(rng.randrange(0xAA) for _ in range(rng.randint(0, 5)))
        stream += codec.build_frame(msg_type, payload)
        expected.append((msg_type, payload))
    parser = codec.Parser()
    received = []
    position = 0
    now = 0.0
    while position < len(stream):
        size = rng.randint(1, 40)
        received += parser.feed(bytes(stream[position:position + size]), now=now)
        position += size
        now += 0.001
    assert received == expected


# --------------------------------------------------------------------------
# Layer 2 and 3: messages and dispatch
# --------------------------------------------------------------------------

MESSAGES = [
    codec.DriveCmd(0, 0),
    codec.DriveCmd(-32768, 32767),
    codec.DriveCmd(200, -200),
    codec.DrawerCmd(0, codec.DrawerAction.LOCK, 0),
    codec.DrawerCmd(3, codec.DrawerAction.UNLOCK, 255),
    codec.MeasureCmd(codec.MeasureAction.START,
                     codec.SensorMask.MAX30102 | codec.SensorMask.MAX30205),
    codec.MeasureCmd(codec.MeasureAction.STOP, codec.SensorMask(0)),
    codec.SystemCmd(codec.SystemCommand.SOFT_RESET),
    codec.Ack(0x02, codec.AckStatus.REFUSED_SAFETY),
    codec.SensorData(-2**31, 2**31 - 1, 2**32 - 1, -32768, None, 0, 65534),
    codec.Vitals(codec.VitalsFlag.HR_VALID | codec.VitalsFlag.IR_TEMP_VALID, 72, 98, 3650, 3712),
    codec.DrawerStatus(0b0101, 0b0001),
    codec.Status(codec.StatusFlag.ESTOP | codec.StatusFlag.OBST_STOP, 7400),
]

EXPECTED_LEN = {
    codec.DriveCmd: 4, codec.DrawerCmd: 3, codec.MeasureCmd: 2, codec.SystemCmd: 1,
    codec.Ack: 2, codec.SensorData: 20, codec.Vitals: 7, codec.DrawerStatus: 2,
    codec.Status: 4,
}


@pytest.mark.parametrize('message', MESSAGES)
def test_round_trip_through_frame_and_parser(message):
    """encode, frame, parse and decode gives back an equal message."""
    frames = codec.Parser().feed(message.to_frame(), now=0.0)
    assert len(frames) == 1
    msg_type, payload = frames[0]
    assert msg_type == message.TYPE
    assert len(payload) == EXPECTED_LEN[type(message)]
    assert codec.decode_frame(msg_type, payload) == message


def test_to_frame_matches_protocol_vectors():
    """Message objects produce exactly the frames in protocol.md section 9."""
    assert codec.DriveCmd(200, 200).to_frame() == DRIVE_FRAME
    assert codec.DrawerCmd(1, codec.DrawerAction.UNLOCK, 30).to_frame() == DRAWER_FRAME
    assert codec.Ack(0x02, codec.AckStatus.OK).to_frame() == ACK_FRAME


def test_sensor_data_decodes_independent_bytes():
    """Hand-written bytes check big-endian order, two's complement and the None marker."""
    payload = bytes.fromhex('000003E8 FFFFFC18 0001E240 FF6A FFFF 012C 01C2')
    data = codec.SensorData.decode(payload)
    assert data.left_ticks == 1000
    assert data.right_ticks == -1000
    assert data.t_ms == 123456
    assert data.gyro_z == -150
    assert data.gyro_z_deg_s == -1.5
    assert data.range_front_mm is None
    assert data.range_left_mm == 300
    assert data.range_right_mm == 450
    assert data.encode() == payload


def test_vitals_properties_respect_valid_flags():
    """Values are only exposed when their valid flag is set."""
    payload = bytes.fromhex('17 48 62 0E42 0000')  # HR, SpO2, IR temp, finger; contact invalid
    vitals = codec.Vitals.decode(payload)
    assert vitals.heart_rate_bpm == 72
    assert vitals.spo2_percent == 98
    assert vitals.temp_ir_c == 36.5
    assert vitals.temp_contact_c is None
    assert codec.VitalsFlag.FINGER_DETECTED in vitals.flags
    empty = codec.Vitals(0, 0, 0, 0, 0)
    assert empty.heart_rate_bpm is None
    assert empty.spo2_percent is None
    assert empty.temp_ir_c is None
    assert empty.temp_contact_c is None


def test_drawer_status_helpers():
    """is_unlocked and is_open read the right bits."""
    status = codec.DrawerStatus(unlocked_mask=0b0010, open_mask=0b1000)
    assert [status.is_unlocked(i) for i in range(4)] == [False, True, False, False]
    assert [status.is_open(i) for i in range(4)] == [False, False, False, True]


def test_flag_bit_layouts_match_protocol():
    """Flag values are the bit positions written in protocol.md."""
    assert codec.StatusFlag.ESTOP == 1
    assert codec.StatusFlag.OBST_STOP == 8
    assert codec.StatusFlag.GYRO_CAL == 128
    assert codec.VitalsFlag.FINGER_DETECTED == 16
    assert codec.SensorMask.MAX30205 == 4


def test_status_keeps_reserved_bits_instead_of_failing():
    """Reserved bits set by newer firmware must not break decoding."""
    status = codec.Status.decode(bytes.fromhex('8001 1CE8'))
    assert codec.StatusFlag.ESTOP in status.flags
    assert status.battery_mv == 7400


LEN_BY_TYPE = {cls.TYPE: length for cls, length in EXPECTED_LEN.items()}


@pytest.mark.parametrize('msg_type', sorted(LEN_BY_TYPE))
def test_wrong_payload_length_is_a_protocol_error(msg_type):
    """A valid frame with the wrong payload length fails in decode, not in the Parser."""
    length = LEN_BY_TYPE[msg_type]
    with pytest.raises(codec.ProtocolError):
        codec.decode_frame(msg_type, bytes(length + 1))
    with pytest.raises(codec.ProtocolError):
        codec.decode_frame(msg_type, bytes(length - 1))


def test_out_of_range_values_raise():
    """Bad values are rejected both when building and when decoding."""
    with pytest.raises(codec.ProtocolError):
        codec.DriveCmd(40000, 0)
    with pytest.raises(codec.ProtocolError):
        codec.DriveCmd(1.5, 0)
    with pytest.raises(codec.ProtocolError):
        codec.DrawerCmd(4, codec.DrawerAction.UNLOCK, 0)
    with pytest.raises(codec.ProtocolError):
        codec.SensorData(0, 0, 0, 0, codec.NO_RANGE, None, None)
    with pytest.raises(codec.ProtocolError):
        codec.decode_frame(0x02, bytes([1, 2, 0]))  # action 2 does not exist
    with pytest.raises(codec.ProtocolError):
        codec.decode_frame(0x04, bytes([9]))  # unknown system command
    with pytest.raises(codec.ProtocolError):
        codec.decode_frame(0x81, bytes([0x02, 9]))  # unknown ACK status
    with pytest.raises(codec.ProtocolError):
        codec.decode_frame(0x03, bytes([1, 0x08]))  # sensor_mask bit 3 does not exist


def test_unknown_type_raises_unknown_type_error():
    """An unknown type is a different error from a bad payload."""
    with pytest.raises(codec.UnknownTypeError):
        codec.decode_frame(0x7F, b'')
    assert issubclass(codec.UnknownTypeError, codec.ProtocolError)
