"""Tests for the ROS-free patient identification core."""

from pathlib import Path
import sys

try:
    from isocare_patient_id.identify_core import IdentifyCore, QrDecodeError
except ImportError:  # running from the repo without sourcing the workspace
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isocare_patient_id.identify_core import IdentifyCore, QrDecodeError


def test_non_empty_qr_payload_is_trimmed_and_returned():
    """A QR payload with surrounding whitespace becomes the patient ID."""
    core = IdentifyCore(decoder=lambda frame: ' P001\n', confirm_frames=1)
    core.start(10.0)

    result = core.observe_frame(object(), 11.0)

    assert result.success
    assert result.patient_id == 'P001'
    assert result.reason == ''


def test_empty_or_absent_qr_payload_keeps_searching_until_deadline():
    """Frames without a decoded payload do not identify a patient."""
    core = IdentifyCore(timeout_s=2.0, decoder=lambda frame: '')
    core.start(10.0)

    assert core.observe_frame(object(), 11.0) is None
    assert core.poll(11.9) is None
    result = core.poll(12.0)
    assert not result.success
    assert result.patient_id == ''
    assert result.reason == 'no QR found before timeout'


def test_decoder_failure_returns_a_normal_failed_result():
    """A known QR decoder failure is reported as an ordinary action failure."""
    def fail(frame):
        raise QrDecodeError('QR decoding failed: invalid frame')

    core = IdentifyCore(decoder=fail, confirm_frames=1)
    core.start(1.0)

    result = core.observe_frame(object(), 2.0)

    assert not result.success
    assert result.reason == 'QR decoding failed: invalid frame'


def test_timeout_must_be_positive_and_finite():
    """Invalid timeout parameters are rejected instead of creating an endless goal."""
    for timeout in (0.0, -1.0, float('inf')):
        try:
            IdentifyCore(timeout_s=timeout)
        except ValueError as error:
            assert str(error) == 'timeout_s must be a positive finite number'
        else:
            raise AssertionError(f'accepted invalid timeout {timeout}')


def reads(texts):
    """Return a decoder that gives the texts one by one."""
    items = iter(texts)
    return lambda frame: next(items)


def test_a_payload_needs_three_identical_reads_by_default():
    """One or two reads are not enough to accept a patient id."""
    core = IdentifyCore(decoder=reads(['P001', 'P001', 'P001']))
    core.start(0.0)

    assert core.observe_frame(object(), 1.0) is None
    assert core.observe_frame(object(), 2.0) is None
    result = core.observe_frame(object(), 3.0)

    assert result.success and result.patient_id == 'P001'


def test_a_garbled_read_in_between_restarts_the_count():
    """A different payload restarts the confirmation, so a stray decode never wins."""
    core = IdentifyCore(decoder=reads(['P001', 'P0O1', 'P001', 'P001', 'P001']))
    core.start(0.0)

    outcomes = [core.observe_frame(object(), float(t)) for t in range(1, 6)]

    assert outcomes[:4] == [None, None, None, None]
    assert outcomes[4].success and outcomes[4].patient_id == 'P001'


def test_frames_without_a_qr_do_not_reset_the_count():
    """A frame with no QR between two good reads keeps the count."""
    core = IdentifyCore(decoder=reads(['P001', '', 'P001', None, 'P001']))
    core.start(0.0)

    outcomes = [core.observe_frame(object(), float(t)) for t in range(1, 6)]

    assert outcomes[4].success


def test_start_clears_the_count_of_the_previous_goal():
    """A new goal never inherits reads from the last one."""
    core = IdentifyCore(decoder=reads(['P001', 'P001', 'P001', 'P001']), confirm_frames=3)
    core.start(0.0)
    core.observe_frame(object(), 1.0)
    core.observe_frame(object(), 2.0)
    core.start(10.0)

    assert core.streak == 0
    assert core.observe_frame(object(), 11.0) is None


def test_confirm_frames_must_be_at_least_one():
    """A zero or negative confirmation count is rejected."""
    for value in (0, -2):
        try:
            IdentifyCore(confirm_frames=value)
        except ValueError:
            continue
        raise AssertionError(f'accepted confirm_frames {value}')
