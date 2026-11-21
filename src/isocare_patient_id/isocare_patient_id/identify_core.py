"""ROS-free QR decoding and identification deadline logic."""

from dataclasses import dataclass
import math
from typing import Callable, Optional


class QrDecodeError(Exception):
    """A frame could not be decoded by the QR decoder."""


@dataclass(frozen=True)
class IdentifyResult:
    """Result of one patient identification attempt."""

    success: bool
    patient_id: str = ''
    reason: str = ''


def decode_qr(frame: object) -> str:
    """Return decoded QR text from an OpenCV image, or an empty string if none is present."""
    try:
        import cv2
    except ImportError as error:
        raise QrDecodeError(f'OpenCV is unavailable: {error}') from error
    try:
        decoded, _, _ = cv2.QRCodeDetector().detectAndDecode(frame)
    except cv2.error as error:
        raise QrDecodeError(f'QR decoding failed: {error}') from error
    return decoded


class IdentifyCore:
    """Find a QR payload that was read the same way several times before the deadline.

    A payload is accepted only after `confirm_frames` identical reads in a row. Frames with no QR
    do not reset the count, a different payload does. This keeps one garbled decode from becoming
    a patient id.
    """

    def __init__(self, timeout_s: float = 55.0,
                 decoder: Optional[Callable[[object], Optional[str]]] = None,
                 confirm_frames: int = 3):
        """Create an identification core with an optional injected decoder for tests."""
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError('timeout_s must be a positive finite number')
        if int(confirm_frames) < 1:
            raise ValueError('confirm_frames must be at least 1')
        self.timeout_s = timeout_s
        self.confirm_frames = int(confirm_frames)
        self._decoder = decoder if decoder is not None else decode_qr
        self._started_at: Optional[float] = None
        self._last_text = ''
        self.streak = 0                 # identical reads in a row so far

    def start(self, now: float) -> None:
        """Start a new attempt at the supplied monotonic time."""
        self._started_at = float(now)
        self._last_text = ''
        self.streak = 0

    def remaining_s(self, now: float) -> float:
        """Return the time left in this attempt, or the full budget before it starts."""
        if self._started_at is None:
            return self.timeout_s
        return max(0.0, self.timeout_s - (float(now) - self._started_at))

    def poll(self, now: float) -> Optional[IdentifyResult]:
        """Return a timeout failure once the attempt reaches its deadline."""
        if self._started_at is not None and self.remaining_s(now) <= 0.0:
            return IdentifyResult(False, reason='no QR found before timeout')
        return None

    def observe_frame(self, frame: object, now: float) -> Optional[IdentifyResult]:
        """Inspect one frame and return an outcome if it identifies the patient or fails."""
        timeout = self.poll(now)
        if timeout is not None:
            return timeout
        try:
            decoded = self._decoder(frame)
        except QrDecodeError as error:
            return IdentifyResult(False, reason=str(error))
        if decoded is None:
            return None
        patient_id = str(decoded).strip()
        if not patient_id:
            return None
        if patient_id == self._last_text:
            self.streak += 1
        else:
            self._last_text, self.streak = patient_id, 1
        if self.streak >= self.confirm_frames:
            return IdentifyResult(True, patient_id=patient_id)
        return None
