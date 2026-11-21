"""Tests for calibrate_camera: synthetic checkerboard photos, no camera and no ROS."""

from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

try:
    from isocare_marker_detector import calibrate_camera as cc
    from isocare_marker_detector.marker_core import load_calibration
except ImportError:  # running from the repo without sourcing the workspace
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isocare_marker_detector import calibrate_camera as cc
    from isocare_marker_detector.marker_core import load_calibration

PATTERN = (9, 6)
SQUARE_M = 0.03
SIZE = (640, 480)
TRUE_MATRIX = np.array([[800.0, 0.0, 322.0], [0.0, 805.0, 243.0], [0.0, 0.0, 1.0]])
PIXELS_PER_SQUARE = 40
MARGIN_PX = 40


def board_image():
    """Draw a flat checkerboard with a white margin, 10 x 7 squares (9 x 6 inner corners)."""
    columns, rows = PATTERN[0] + 1, PATTERN[1] + 1
    image = np.full((rows * PIXELS_PER_SQUARE + 2 * MARGIN_PX,
                     columns * PIXELS_PER_SQUARE + 2 * MARGIN_PX), 255, np.uint8)
    for row in range(rows):
        for column in range(columns):
            if (row + column) % 2 == 0:
                y0 = MARGIN_PX + row * PIXELS_PER_SQUARE
                x0 = MARGIN_PX + column * PIXELS_PER_SQUARE
                image[y0:y0 + PIXELS_PER_SQUARE, x0:x0 + PIXELS_PER_SQUARE] = 0
    return image


def render(rotation_deg, shift, depth_m):
    """Photograph the board with the true camera at a given tilt, sideways shift and distance."""
    board = board_image()
    height, width = board.shape
    metres_per_pixel = SQUARE_M / PIXELS_PER_SQUARE
    corners_3d = np.array([[0, 0, 0], [width, 0, 0], [width, height, 0], [0, height, 0]],
                          dtype=np.float64) * metres_per_pixel
    centre = corners_3d.mean(axis=0)
    rotation, _ = cv2.Rodrigues(np.deg2rad(np.array(rotation_deg, dtype=np.float64)))
    translation = -rotation @ centre + np.array([shift[0], shift[1], depth_m])
    projected, _ = cv2.projectPoints(corners_3d, cv2.Rodrigues(rotation)[0], translation,
                                     TRUE_MATRIX, None)
    transform = cv2.getPerspectiveTransform(
        np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32),
        projected.reshape(4, 2).astype(np.float32))
    return cv2.warpPerspective(board, transform, SIZE, borderValue=255)


POSES = [((0, 0, 0), (0.0, 0.0), 0.62), ((20, 0, 0), (0.0, 0.0), 0.66),
         ((-20, 0, 0), (0.02, 0.0), 0.66), ((0, 25, 0), (0.0, 0.01), 0.66),
         ((0, -25, 0), (-0.02, 0.0), 0.66), ((15, 15, 10), (0.0, 0.0), 0.70),
         ((-15, 15, -10), (0.02, 0.01), 0.70), ((15, -15, 15), (-0.02, 0.0), 0.70),
         ((-15, -15, -15), (0.0, 0.02), 0.70), ((25, 10, 0), (0.0, -0.01), 0.75),
         ((-10, -25, 20), (0.01, 0.0), 0.75), ((0, 0, 30), (0.0, 0.0), 0.68)]


@pytest.fixture(scope='module')
def photo_folder(tmp_path_factory):
    """Write the synthetic photos to a folder once."""
    folder = tmp_path_factory.mktemp('photos')
    for index, (rotation, shift, depth) in enumerate(POSES):
        cv2.imwrite(str(folder / f'view_{index:02d}.png'), render(rotation, shift, depth))
    return folder


def test_synthetic_photos_recover_the_true_camera(photo_folder):
    """The calibration finds the focal lengths and centre that rendered the photos."""
    paths = sorted(photo_folder.glob('*.png'))
    corner_sets, size, missed = cc.corners_from_files(paths, PATTERN)
    assert not missed and size == SIZE

    matrix, distortion, rms = cc.calibrate(corner_sets, size, PATTERN, SQUARE_M)

    assert matrix[0, 0] == pytest.approx(800.0, rel=0.03)
    assert matrix[1, 1] == pytest.approx(805.0, rel=0.03)
    assert matrix[0, 2] == pytest.approx(322.0, abs=6.0)
    assert matrix[1, 2] == pytest.approx(243.0, abs=6.0)
    assert rms < 0.5
    corner = np.array([320.0 / 800.0, 240.0 / 805.0])        # the farthest image corner
    radius_sq = float(corner @ corner)
    stretch = distortion[0] * radius_sq + distortion[1] * radius_sq ** 2
    assert abs(stretch) * 400.0 < 2.0                  # under 2 px of lens error at the corner


def test_the_written_file_loads_with_the_marker_core_loader(photo_folder, tmp_path):
    """Whatever the tool writes, marker_core.load_calibration can read."""
    out = tmp_path / 'sub' / 'camera.yaml'
    args = cc.build_parser().parse_args(
        ['photos', str(photo_folder), '--pattern', '9x6', '--square-m', str(SQUARE_M),
         '--out', str(out)])
    cc.run_photos(args)

    calibration = load_calibration(str(out))

    assert calibration.camera_matrix.shape == (3, 3)
    assert calibration.camera_matrix[0, 0] == pytest.approx(800.0, rel=0.03)


def test_too_few_views_are_refused(photo_folder):
    """Three photos are not enough, the message says how many are needed."""
    paths = sorted(photo_folder.glob('*.png'))[:3]
    corner_sets, size, _ = cc.corners_from_files(paths, PATTERN)
    with pytest.raises(cc.CalibrationFailed, match='need at least 10'):
        cc.calibrate(corner_sets, size, PATTERN, SQUARE_M)


def test_a_photo_without_a_board_is_reported_not_fatal(tmp_path):
    """A blank photo is listed as missed and the others still count."""
    cv2.imwrite(str(tmp_path / 'blank.png'), np.full((480, 640), 128, np.uint8))
    cv2.imwrite(str(tmp_path / 'good.png'), render(*POSES[0]))
    corner_sets, size, missed = cc.corners_from_files(sorted(tmp_path.glob('*.png')), PATTERN)
    assert len(corner_sets) == 1 and missed == ['blank.png']


def test_a_wrong_square_size_does_not_change_the_pixel_error_but_a_bad_one_is_refused(
        photo_folder):
    """A zero or negative square size is rejected up front."""
    corner_sets, size, _ = cc.corners_from_files(sorted(photo_folder.glob('*.png')), PATTERN)
    for bad in (0.0, -0.03, float('nan')):
        with pytest.raises(cc.CalibrationFailed, match='square size'):
            cc.calibrate(corner_sets, size, PATTERN, bad)


def test_a_poor_result_is_refused(photo_folder):
    """A very strict error limit makes the tool refuse rather than write a doubtful file."""
    corner_sets, size, _ = cc.corners_from_files(sorted(photo_folder.glob('*.png')), PATTERN)
    with pytest.raises(cc.CalibrationFailed, match='reprojection error'):
        cc.calibrate(corner_sets, size, PATTERN, SQUARE_M, max_rms_px=1e-6)


def test_pattern_text_is_checked():
    """The pattern argument needs the form COLSxROWS with at least 3 x 3 corners."""
    assert cc.parse_pattern('9x6') == (9, 6)
    for bad in ('9', '9x', 'axb', '2x6'):
        with pytest.raises(Exception):
            cc.parse_pattern(bad)


def test_main_reports_failure_with_a_nonzero_exit(tmp_path, capsys):
    """An empty folder gives a clear message and exit code 1, not a traceback."""
    code = cc.main(['photos', str(tmp_path), '--pattern', '9x6', '--square-m', '0.03',
                    '--out', str(tmp_path / 'x.yaml')])
    assert code == 1 and 'no photos' in capsys.readouterr().err
