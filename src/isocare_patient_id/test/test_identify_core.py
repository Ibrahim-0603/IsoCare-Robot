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
    core = IdentifyCore(decoder=lambda frame: ' P001\n')
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

    core = IdentifyCore(decoder=fail)
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
