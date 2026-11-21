"""Tests for the ROS-free ArUco detector and pose conversion."""

from pathlib import Path
import sys

import cv2
import numpy as np
import pytest


def _core_types():
    """Import the core from this package when tests run from any workspace directory."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isocare_marker_detector.marker_core import (
        CalibrationError, CameraCalibration, MarkerCore, load_calibration)
    return CalibrationError, CameraCalibration, MarkerCore, load_calibration


CalibrationError, CameraCalibration, MarkerCore, load_calibration = _core_types()


IMAGE_SIZE = (640, 480)
CAMERA_MATRIX = np.array([
    [500.0, 0.0, 320.0],
    [0.0, 500.0, 240.0],
    [0.0, 0.0, 1.0],
])
DISTORTION = np.zeros((5, 1), dtype=np.float64)
MARKER_SIZE_M = 0.15


def calibration_file(path):
    """Write a synthetic OpenCV YAML calibration and load it."""
    path.write_text(
        '%YAML:1.0\n'
        'camera_matrix: !!opencv-matrix\n'
        '   rows: 3\n'
        '   cols: 3\n'
        '   dt: d\n'
        '   data: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]\n'
        'distortion_coefficients: !!opencv-matrix\n'
        '   rows: 1\n'
        '   cols: 5\n'
        '   dt: d\n'
        '   data: [0.0, 0.0, 0.0, 0.0, 0.0]\n',
        encoding='utf-8')
    return load_calibration(str(path))


def projected_marker(marker_id, translation, rotation=None):
    """Render a generated marker at a known camera-relative pose."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    source = cv2.aruco.generateImageMarker(dictionary, marker_id, 400)
    if rotation is None:
        rotation = np.diag([1.0, -1.0, -1.0])
    rotation_vector, _ = cv2.Rodrigues(rotation)
    half_size = MARKER_SIZE_M / 2.0
    object_points = np.array([
        [-half_size, half_size, 0.0],
        [half_size, half_size, 0.0],
        [half_size, -half_size, 0.0],
        [-half_size, -half_size, 0.0],
    ], dtype=np.float64)
    image_points, _ = cv2.projectPoints(
        object_points, rotation_vector, np.asarray(translation, dtype=np.float64),
        CAMERA_MATRIX, DISTORTION)
    source_corners = np.array(
        [[0.0, 0.0], [399.0, 0.0], [399.0, 399.0], [0.0, 399.0]],
        dtype=np.float32)
    transform = cv2.getPerspectiveTransform(source_corners, image_points.reshape(4, 2).astype(
        np.float32))
    return cv2.warpPerspective(
        source, transform, IMAGE_SIZE, borderValue=255, flags=cv2.INTER_NEAREST)


def composite_images(*images):
    """Place marker renderings on one white image."""
    combined = np.full((IMAGE_SIZE[1], IMAGE_SIZE[0]), 255, dtype=np.uint8)
    for image in images:
        combined = np.minimum(combined, image)
    return combined


def make_core(calibration, **parameters):
    """Create a marker core using the synthetic camera intrinsics."""
    return MarkerCore(
        calibration=calibration, marker_size_m=MARKER_SIZE_M,
        cam_x_m=0.0, cam_y_m=0.0, cam_z_m=0.0, **parameters)


def test_front_facing_marker_has_expected_distance_bearing_and_facing(tmp_path):
    """A marker one metre ahead reports zero bearing and faces back toward the robot."""
    core = make_core(calibration_file(tmp_path / 'camera.yaml'))
    image = projected_marker(2, (0.0, 0.0, 1.0))

    sightings = core.detect(image)

    assert len(sightings) == 1
    sighting = sightings[0]
    assert sighting.marker_id == 2
    assert sighting.distance_m == pytest.approx(1.0, abs=0.02)
    assert sighting.bearing_rad == pytest.approx(0.0, abs=0.02)
    assert abs(sighting.facing_rel_rad) == pytest.approx(np.pi, abs=0.02)
    assert sighting.side_px == pytest.approx(75.0, abs=2.0)


def test_marker_to_robot_left_has_positive_bearing(tmp_path):
    """A marker offset to the robot left reports positive bearing."""
    core = make_core(calibration_file(tmp_path / 'camera.yaml'))
    image = projected_marker(2, (-0.2, 0.0, 1.0))

    sighting = core.detect(image)[0]

    assert sighting.bearing_rad > 0.0
    assert sighting.distance_m == pytest.approx(np.hypot(1.0, 0.2), abs=0.03)


def test_marker_front_orientation_is_reported_in_robot_axes(tmp_path):
    """A marker turned toward the robot left shifts its reported facing angle."""
    calibration = calibration_file(tmp_path / 'camera.yaml')
    core = make_core(calibration)
    desired_facing = np.pi - 0.25
    robot_marker_rotation = np.column_stack((
        [-np.sin(desired_facing), np.cos(desired_facing), 0.0],
        [0.0, 0.0, 1.0],
        [np.cos(desired_facing), np.sin(desired_facing), 0.0]))
    robot_camera_rotation = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])
    camera_marker_rotation = robot_camera_rotation.T @ robot_marker_rotation
    image = projected_marker(
        2, (0.0, 0.0, 1.0), rotation=camera_marker_rotation)

    sighting = core.detect(image)[0]

    assert sighting.facing_rel_rad == pytest.approx(desired_facing, abs=0.03)


def test_camera_forward_offset_is_added_to_robot_relative_marker_position(tmp_path):
    """Camera offset translates the observed point into the robot-center frame."""
    calibration = calibration_file(tmp_path / 'camera.yaml')
    image = projected_marker(2, (0.0, 0.0, 1.0))

    sighting = MarkerCore(
        calibration=calibration, marker_size_m=MARKER_SIZE_M,
        cam_x_m=0.1, cam_y_m=0.0, cam_z_m=0.0).detect(image)[0]

    assert sighting.distance_m == pytest.approx(1.1, abs=0.02)


def test_camera_pitch_and_mounting_offset_are_applied_before_planar_distance(tmp_path):
    """A pitched camera still reports range from the robot center, not the lens."""
    calibration = calibration_file(tmp_path / 'camera.yaml')
    pitch = np.deg2rad(10.0)
    robot_camera_rotation = np.array([
        [0.0, -np.sin(pitch), np.cos(pitch)],
        [-1.0, 0.0, 0.0],
        [0.0, -np.cos(pitch), -np.sin(pitch)],
    ])
    desired_facing = np.pi
    robot_marker_rotation = np.column_stack((
        [-np.sin(desired_facing), np.cos(desired_facing), 0.0],
        [0.0, 0.0, 1.0],
        [np.cos(desired_facing), np.sin(desired_facing), 0.0]))
    camera_marker_rotation = robot_camera_rotation.T @ robot_marker_rotation
    camera_position = np.array([0.1, 0.0, 0.4])
    marker_position = np.array([1.0, 0.0, 0.4])
    camera_marker_position = robot_camera_rotation.T @ (marker_position - camera_position)
    image = projected_marker(
        2, camera_marker_position, rotation=camera_marker_rotation)
    core = MarkerCore(
        calibration=calibration, marker_size_m=MARKER_SIZE_M,
        cam_x_m=0.1, cam_y_m=0.0, cam_z_m=0.4, cam_pitch_deg=10.0)

    sighting = core.detect(image)[0]

    assert sighting.distance_m == pytest.approx(1.0, abs=0.03)
    assert sighting.bearing_rad == pytest.approx(0.0, abs=0.02)
    assert abs(sighting.facing_rel_rad) == pytest.approx(np.pi, abs=0.03)


def test_two_markers_produce_one_sighting_each(tmp_path):
    """Every detected marker in a frame is returned independently."""
    core = make_core(calibration_file(tmp_path / 'camera.yaml'))
    image = composite_images(
        projected_marker(1, (-0.2, 0.0, 1.0)),
        projected_marker(2, (0.2, 0.0, 1.0)))

    sightings = core.detect(image)

    assert {sighting.marker_id for sighting in sightings} == {1, 2}


def test_marker_id_outside_the_navigation_table_is_still_returned(tmp_path):
    """The detector reports valid IDs without consulting a marker map."""
    core = make_core(calibration_file(tmp_path / 'camera.yaml'))

    sightings = core.detect(projected_marker(6, (0.0, 0.0, 1.0)))

    assert [sighting.marker_id for sighting in sightings] == [6]


def test_blank_image_and_frame_without_markers_return_no_sightings(tmp_path):
    """Blank frames and frames without markers produce an empty tuple."""
    core = make_core(calibration_file(tmp_path / 'camera.yaml'))

    assert core.detect(np.full((480, 640), 255, dtype=np.uint8)) == ()
    assert core.detect(np.zeros((480, 640, 3), dtype=np.uint8)) == ()


def test_missing_calibration_file_is_reported(tmp_path):
    """A missing calibration is an explicit startup error, never a guessed fallback."""
    with pytest.raises(CalibrationError, match='does not exist'):
        load_calibration(str(tmp_path / 'missing.yaml'))


def test_invalid_calibration_is_reported(tmp_path):
    """Calibration with missing matrices is rejected."""
    calibration_path = tmp_path / 'invalid.yaml'
    calibration_path.write_text('%YAML:1.0\nunrelated: 1\n', encoding='utf-8')

    with pytest.raises(CalibrationError, match='must define'):
        load_calibration(str(calibration_path))


def test_marker_core_requires_loaded_calibration():
    """Core construction refuses to run without actual calibration."""
    with pytest.raises(CalibrationError, match='required'):
        MarkerCore(calibration=None)


def test_pitch_parameter_must_be_finite():
    """Invalid camera pitch is rejected at construction."""
    calibration = CameraCalibration(CAMERA_MATRIX, DISTORTION)

    with pytest.raises(ValueError, match='cam_pitch_deg must be finite'):
        make_core(calibration, cam_pitch_deg=float('nan'))
