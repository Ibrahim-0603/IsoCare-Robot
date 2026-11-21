"""marker_detector: publish every ArUco marker seen as a MarkerSighting relative to the robot.

ROS side:
    subscribes   image_topic (default /camera/image_raw)   sensor_msgs/Image
    publishes    /markers                                  isocare_interfaces/MarkerSighting

One message per detected marker per frame. This node only reports what it sees: the navigator
decides whether a sighting may correct the pose, and uses it for steering.
Parameters: image_topic, marker_size_m, aruco_dictionary, cam_x_m, cam_y_m, cam_z_m,
cam_pitch_deg, calibration_file (OpenCV YAML written by calibrate_camera). The node refuses to
start without a usable calibration file, it never guesses a lens.
"""

import time

from cv_bridge import CvBridge, CvBridgeError
from isocare_interfaces.msg import MarkerSighting
from isocare_marker_detector.marker_core import (
    CalibrationError, load_calibration, MarkerCore)
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

DEFAULT_CALIBRATION = '~/.isocare/camera_calibration.yaml'
ROBOT_FRAME = 'base_link'
WARN_EVERY_S = 5.0


def fill_sighting(msg, stamp, sighting):
    """Copy a core sighting into a MarkerSighting message with the exact wire types."""
    msg.header.stamp = stamp
    msg.header.frame_id = ROBOT_FRAME
    msg.marker_id = int(sighting.marker_id)
    msg.distance_m = float(sighting.distance_m)
    msg.bearing_rad = float(sighting.bearing_rad)
    msg.facing_rel_rad = float(sighting.facing_rel_rad)
    msg.side_px = float(sighting.side_px)
    return msg


class MarkerDetector(Node):
    """The ROS node. See the module docstring."""

    def __init__(self):
        """Load the calibration (or fail clearly), then subscribe and publish."""
        super().__init__('marker_detector')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('marker_size_m', 0.15)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('cam_x_m', 0.10)
        self.declare_parameter('cam_y_m', 0.0)
        self.declare_parameter('cam_z_m', 0.40)
        self.declare_parameter('cam_pitch_deg', 0.0)
        self.declare_parameter('calibration_file', DEFAULT_CALIBRATION)
        param = self.get_parameter
        calibration_file = str(param('calibration_file').value)
        try:
            calibration = load_calibration(calibration_file)
            self.core = MarkerCore(
                calibration, marker_size_m=float(param('marker_size_m').value),
                aruco_dictionary=str(param('aruco_dictionary').value),
                cam_x_m=float(param('cam_x_m').value), cam_y_m=float(param('cam_y_m').value),
                cam_z_m=float(param('cam_z_m').value),
                cam_pitch_deg=float(param('cam_pitch_deg').value))
        except (CalibrationError, ValueError) as error:
            self.get_logger().error(
                f'marker_detector cannot start: {error}. Run calibrate_camera first, or pass '
                f'-p calibration_file:=PATH')
            raise
        self.bridge = CvBridge()
        self._last_warn = {}
        self.publisher = self.create_publisher(MarkerSighting, '/markers', 10)
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, str(param('image_topic').value), self._on_image, qos)
        self.get_logger().info(
            f'marker_detector ready: marker {float(param("marker_size_m").value):g} m, '
            f'calibration {calibration_file}')

    def _warn_limited(self, key, text):
        """Warn at most once every few seconds per kind of problem."""
        now = time.monotonic()
        if now - self._last_warn.get(key, -WARN_EVERY_S) >= WARN_EVERY_S:
            self._last_warn[key] = now
            self.get_logger().warn(text)

    def _on_image(self, msg):
        """Detect the markers in one image and publish one sighting per marker."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as error:
            self._warn_limited('convert', f'cannot convert camera image: {error}')
            return
        try:
            sightings = self.core.detect(frame)
        except ValueError as error:
            self._warn_limited('detect', f'cannot detect markers: {error}')
            return
        for sighting in sightings:
            self.publisher.publish(fill_sighting(MarkerSighting(), msg.header.stamp, sighting))


def main():
    """Start the node."""
    rclpy.init()
    node = MarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
