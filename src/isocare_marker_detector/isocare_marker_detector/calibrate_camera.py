"""Calibrate the camera with a printed checkerboard and write the YAML marker_detector reads.

Print a checkerboard, stick it on something flat and measure one square with a ruler. Then either
    calibrate_camera photos DIR --pattern 9x6 --square-m 0.025      (a folder of photos), or
    calibrate_camera capture --pattern 9x6 --square-m 0.025         (live camera, needs a display)
`--pattern` is the number of INNER corners (columns x rows), not squares: a board of 10 x 7
squares has 9 x 6 inner corners. Take 15 to 25 views with the board tilted in different
directions, near and far, covering the corners of the image. The file is written to
~/.isocare/camera_calibration.yaml unless --out says otherwise, and is refused when the
reprojection error is too high.

The maths is ROS-free and tested with synthetic images (test_calibrate_camera.py). The live
capture mode needs a camera and a display and is not covered by tests.
"""

import argparse
import math
import os
from pathlib import Path
import sys

import cv2
import numpy as np

DEFAULT_OUT = '~/.isocare/camera_calibration.yaml'
MIN_VIEWS = 10
MAX_RMS_PX = 1.5
IMAGE_SUFFIXES = ('.png', '.jpg', '.jpeg', '.bmp')


class CalibrationFailed(Exception):
    """Not enough good views, or the result is too poor to trust."""


def parse_pattern(text):
    """Turn '9x6' into (9, 6) inner corners."""
    try:
        columns, rows = (int(part) for part in str(text).lower().split('x'))
    except ValueError as error:
        raise argparse.ArgumentTypeError(f'pattern must look like 9x6, got {text!r}') from error
    if columns < 3 or rows < 3:
        raise argparse.ArgumentTypeError('pattern needs at least 3 x 3 inner corners')
    return columns, rows


def board_points(pattern, square_m):
    """Return the 3D inner-corner positions of the board, in metres, on the plane z = 0."""
    columns, rows = pattern
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * float(square_m)
    return points


def find_corners(gray, pattern):
    """Return the refined inner-corner positions in a grayscale image, or None if not found."""
    found, corners = cv2.findChessboardCorners(
        gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def calibrate(corner_sets, image_size, pattern, square_m, min_views=MIN_VIEWS,
              max_rms_px=MAX_RMS_PX):
    """Return (camera_matrix, distortion, rms_px) from the corners found in several views."""
    if len(corner_sets) < min_views:
        raise CalibrationFailed(
            f'only {len(corner_sets)} usable views, need at least {min_views}')
    if not (float(square_m) > 0.0 and math.isfinite(float(square_m))):
        raise CalibrationFailed('square size must be a positive number of metres')
    points = board_points(pattern, square_m)
    # A webcam calibrated from 10 to 25 views overfits the extra lens terms, so only the two
    # radial terms k1 and k2 are fitted (k3 and the tangential terms stay zero).
    flags = cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        [points] * len(corner_sets), corner_sets, image_size, None, None, flags=flags)
    if not (math.isfinite(rms) and rms <= max_rms_px):
        raise CalibrationFailed(
            f'reprojection error {rms:.2f} px is above {max_rms_px:g} px: retake the photos '
            f'(sharp, board flat, more varied angles) and check the square size')
    return matrix, distortion.reshape(-1), float(rms)


def corners_from_files(paths, pattern):
    """Return (corner sets, image size, names of the files without a board)."""
    corner_sets, size, missed = [], None, []
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            missed.append(Path(path).name)
            continue
        if size is not None and (image.shape[1], image.shape[0]) != size:
            raise CalibrationFailed(f'{Path(path).name} has a different size than the others')
        size = (image.shape[1], image.shape[0])
        corners = find_corners(image, pattern)
        if corners is None:
            missed.append(Path(path).name)
        else:
            corner_sets.append(corners)
    return corner_sets, size, missed


def save_calibration(path, matrix, distortion, size, rms):
    """Write the YAML that marker_core.load_calibration reads."""
    target = Path(os.path.expanduser(str(path)))
    target.parent.mkdir(parents=True, exist_ok=True)
    storage = cv2.FileStorage(str(target), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise CalibrationFailed(f'cannot write {target}')
    try:
        storage.write('image_width', int(size[0]))
        storage.write('image_height', int(size[1]))
        storage.write('camera_matrix', np.asarray(matrix, dtype=np.float64))
        storage.write('distortion_coefficients',
                      np.asarray(distortion, dtype=np.float64).reshape(-1, 1))
        storage.write('reprojection_rms_px', float(rms))
    finally:
        storage.release()
    return target


def run_photos(args):
    """Calibrate from a folder of photos."""
    folder = Path(args.folder)
    paths = sorted(p for p in folder.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    if not paths:
        raise CalibrationFailed(f'no photos (png, jpg, bmp) in {folder}')
    corner_sets, size, missed = corners_from_files(paths, args.pattern)
    if missed:
        print(f'board not found in {len(missed)} photo(s): {", ".join(missed)}')
    return finish(corner_sets, size, args)


def run_capture(args):
    """Live capture: space saves a view when the board is found, q finishes."""
    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        raise CalibrationFailed(f'cannot open camera {args.camera}')
    corner_sets, size = [], None
    print('space = keep this view (board must be detected), q = finish and calibrate')
    while True:
        ok, frame = camera.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        corners = find_corners(gray, args.pattern)
        shown = frame.copy()
        if corners is not None:
            cv2.drawChessboardCorners(shown, args.pattern, corners, True)
        cv2.putText(shown, f'views kept: {len(corner_sets)}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow('calibrate_camera', shown)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' ') and corners is not None:
            corner_sets.append(corners)
            print(f'kept view {len(corner_sets)}')
    camera.release()
    cv2.destroyAllWindows()
    return finish(corner_sets, size, args)


def finish(corner_sets, size, args):
    """Calibrate, save and report."""
    matrix, distortion, rms = calibrate(corner_sets, size, args.pattern, args.square_m,
                                        args.min_views, args.max_rms)
    target = save_calibration(args.out, matrix, distortion, size, rms)
    print(f'{len(corner_sets)} views, reprojection error {rms:.2f} px, '
          f'fx {matrix[0, 0]:.1f} fy {matrix[1, 1]:.1f} cx {matrix[0, 2]:.1f} '
          f'cy {matrix[1, 2]:.1f}')
    print(f'written to {target}')
    return target


def build_parser():
    """Return the command-line parser."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--pattern', type=parse_pattern, required=True,
                        help='inner corners, columns x rows, for example 9x6')
    common.add_argument('--square-m', type=float, required=True,
                        help='measured side of one printed square, in metres')
    common.add_argument('--out', default=DEFAULT_OUT, help='output file (default %(default)s)')
    common.add_argument('--min-views', type=int, default=MIN_VIEWS)
    common.add_argument('--max-rms', type=float, default=MAX_RMS_PX,
                        help='refuse a result with a larger reprojection error (pixels)')
    modes = parser.add_subparsers(dest='mode', required=True)
    photos = modes.add_parser('photos', parents=[common], help='calibrate from a folder')
    photos.add_argument('folder')
    photos.set_defaults(run=run_photos)
    capture = modes.add_parser('capture', parents=[common], help='live camera, needs a display')
    capture.add_argument('--camera', type=int, default=0, help='camera index (default 0)')
    capture.set_defaults(run=run_capture)
    return parser


def main(argv=None):
    """Run the command line tool."""
    args = build_parser().parse_args(argv)
    try:
        args.run(args)
    except CalibrationFailed as error:
        print(f'calibration failed: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
