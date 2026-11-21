"""ROS-free ArUco detection and conversion into robot-relative sightings."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


class CalibrationError(ValueError):
    """Camera calibration is missing or unusable."""


@dataclass(frozen=True)
class CameraCalibration:
    """Camera intrinsics and distortion coefficients."""

    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray


@dataclass(frozen=True)
class MarkerSighting:
    """One marker detected in an image, relative to the robot."""

    marker_id: int
    distance_m: float
    bearing_rad: float
    facing_rel_rad: float
    side_px: float


def load_calibration(path: str) -> CameraCalibration:
    """Load camera intrinsics from an OpenCV YAML file, or raise CalibrationError."""
    calibration_path = Path(str(path)).expanduser()
    if not calibration_path.is_file():
        raise CalibrationError(f'camera calibration file does not exist: {calibration_path}')

    storage = cv2.FileStorage(str(calibration_path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise CalibrationError(f'cannot open camera calibration file: {calibration_path}')
    try:
        camera_matrix = storage.getNode('camera_matrix').mat()
        distortion = storage.getNode('distortion_coefficients').mat()
    finally:
        storage.release()

    if camera_matrix is None or distortion is None:
        raise CalibrationError(
            f'{calibration_path} must define camera_matrix and distortion_coefficients')
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
    distortion = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
    if camera_matrix.shape != (3, 3) or distortion.size < 4:
        raise CalibrationError(
            f'{calibration_path} has invalid camera_matrix or distortion_coefficients')
    if not np.all(np.isfinite(camera_matrix)) or not np.all(np.isfinite(distortion)):
        raise CalibrationError(f'{calibration_path} contains non-finite calibration values')
    if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
        raise CalibrationError(f'{calibration_path} must have positive focal lengths')
    return CameraCalibration(camera_matrix, distortion)


def _finite_number(value, name):
    """Convert a number to float and reject non-finite values."""
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


def _wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _camera_to_robot_rotation(pitch_rad):
    """Return the rotation from camera optical axes to robot axes."""
    cosine = math.cos(pitch_rad)
    sine = math.sin(pitch_rad)
    return np.array([
        [0.0, -sine, cosine],
        [-1.0, 0.0, 0.0],
        [0.0, -cosine, -sine],
    ], dtype=np.float64)


class MarkerCore:
    """Detect ArUco markers and estimate their poses relative to the robot."""

    _IPPE_SQUARE_CORNERS = (
        (-0.5, 0.5, 0.0),
        (0.5, 0.5, 0.0),
        (0.5, -0.5, 0.0),
        (-0.5, -0.5, 0.0),
    )

    def __init__(self, calibration: CameraCalibration, marker_size_m: float = 0.15,
                 aruco_dictionary: str = 'DICT_4X4_50', cam_x_m: float = 0.10,
                 cam_y_m: float = 0.0, cam_z_m: float = 0.40,
                 cam_pitch_deg: float = 0.0):
        """Create a detector with explicit camera calibration and mounting parameters."""
        if not isinstance(calibration, CameraCalibration):
            raise CalibrationError('a loaded camera calibration is required')
        self.marker_size_m = _finite_number(marker_size_m, 'marker_size_m')
        if self.marker_size_m <= 0.0:
            raise ValueError('marker_size_m must be positive')
        self.cam_x_m = _finite_number(cam_x_m, 'cam_x_m')
        self.cam_y_m = _finite_number(cam_y_m, 'cam_y_m')
        self.cam_z_m = _finite_number(cam_z_m, 'cam_z_m')
        pitch_deg = _finite_number(cam_pitch_deg, 'cam_pitch_deg')
        self._pitch_rad = math.radians(pitch_deg)
        self._camera_to_robot = _camera_to_robot_rotation(self._pitch_rad)

        dictionary_id = getattr(cv2.aruco, str(aruco_dictionary), None)
        if not isinstance(dictionary_id, int):
            raise ValueError(f'unknown ArUco dictionary: {aruco_dictionary}')
        self._dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            parameters = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(self._dictionary, parameters)
        else:
            parameters_factory = getattr(cv2.aruco, 'DetectorParameters_create', None)
            parameters = (parameters_factory() if parameters_factory is not None
                          else cv2.aruco.DetectorParameters())
            self._detector = None
        self._detector_parameters = parameters
        self._calibration = calibration
        self._object_points = np.asarray(self._IPPE_SQUARE_CORNERS, dtype=np.float64)
        self._object_points *= self.marker_size_m

    def detect(self, image) -> tuple:
        """Return all valid marker sightings in one image; return an empty tuple if none."""
        if not isinstance(image, np.ndarray) or image.size == 0:
            raise ValueError('image must be a non-empty NumPy array')
        if image.ndim == 2:
            gray = image
        elif image.ndim == 3 and image.shape[2] == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            raise ValueError('image must be grayscale or a BGR image')

        if self._detector is None:
            corners, identifiers, _ = cv2.aruco.detectMarkers(
                gray, self._dictionary, parameters=self._detector_parameters)
        else:
            corners, identifiers, _ = self._detector.detectMarkers(gray)
        if identifiers is None:
            return ()

        sightings = []
        for marker_corners, identifier in zip(corners, identifiers.reshape(-1)):
            sighting = self._estimate_sighting(marker_corners, int(identifier))
            if sighting is not None:
                sightings.append(sighting)
        return tuple(sightings)

    def _estimate_sighting(self, corners, marker_id):
        """Estimate one detected marker pose and express it relative to the robot."""
        image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
        solved, rotation_vector, translation = cv2.solvePnP(
            self._object_points, image_points, self._calibration.camera_matrix,
            self._calibration.distortion_coefficients, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not solved:
            return None

        marker_to_camera, _ = cv2.Rodrigues(rotation_vector)
        camera_marker = np.asarray(translation, dtype=np.float64).reshape(3)
        front_toward_camera = float(np.dot(marker_to_camera[:, 2], camera_marker)) < 0.0
        if not front_toward_camera:
            solved, rotation_vector, translation = cv2.solvePnP(
                self._object_points, image_points, self._calibration.camera_matrix,
                self._calibration.distortion_coefficients, flags=cv2.SOLVEPNP_ITERATIVE)
            if not solved:
                return None
            marker_to_camera, _ = cv2.Rodrigues(rotation_vector)
            camera_marker = np.asarray(translation, dtype=np.float64).reshape(3)
            front_toward_camera = float(np.dot(marker_to_camera[:, 2], camera_marker)) < 0.0
        if camera_marker[2] <= 0.0 or not front_toward_camera:
            return None

        marker_to_robot = self._camera_to_robot @ marker_to_camera
        marker_position = (self._camera_to_robot @ camera_marker
                           + np.array([self.cam_x_m, self.cam_y_m, self.cam_z_m]))

        distance = math.hypot(float(marker_position[0]), float(marker_position[1]))
        bearing = math.atan2(float(marker_position[1]), float(marker_position[0]))
        marker_front = marker_to_robot[:, 2]
        facing = math.atan2(float(marker_front[1]), float(marker_front[0]))
        side_lengths = np.linalg.norm(np.roll(image_points, -1, axis=0) - image_points, axis=1)
        side_px = float(np.mean(side_lengths))
        return MarkerSighting(
            marker_id=int(marker_id),
            distance_m=float(distance),
            bearing_rad=float(_wrap_angle(bearing)),
            facing_rel_rad=float(_wrap_angle(facing)),
            side_px=float(side_px))
