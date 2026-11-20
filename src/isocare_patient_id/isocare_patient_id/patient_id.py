"""Action server that identifies a patient from a wristband QR code."""

import threading
import time

from cv_bridge import CvBridge, CvBridgeError
from isocare_interfaces.action import Identify
from isocare_patient_id.identify_core import IdentifyCore, IdentifyResult
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image


class PatientId(Node):
    """Serve `/patient/identify` using QR codes received from a camera topic."""

    def __init__(self):
        """Declare camera settings and create the image subscription and action server."""
        super().__init__('patient_id')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('identify_timeout_s', 55.0)
        image_topic = str(self.get_parameter('image_topic').value)
        timeout_s = float(self.get_parameter('identify_timeout_s').value)
        self.core = IdentifyCore(timeout_s=timeout_s)
        self.bridge = CvBridge()
        self.condition = threading.Condition()
        self.active_goal = False
        self.latest_frame = None
        self.frame_version = 0
        self.camera_error = ''

        group = ReentrantCallbackGroup()
        self.image_sub = self.create_subscription(
            Image, image_topic, self._on_image, 10, callback_group=group)
        self.action_server = ActionServer(
            self, Identify, '/patient/identify',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=group)
        self.get_logger().info(
            f'patient_id ready: /patient/identify, image topic {image_topic}, '
            f'timeout {timeout_s:.1f} s')

    def _on_goal(self, goal_request):
        """Accept only one identification goal at a time."""
        with self.condition:
            if self.active_goal:
                return GoalResponse.REJECT
            self.active_goal = True
            return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        """Allow cancellation and wake a goal waiting for the next image."""
        with self.condition:
            self.condition.notify_all()
        return CancelResponse.ACCEPT

    def _on_image(self, msg):
        """Convert each camera message and make the newest frame available to the goal."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as error:
            with self.condition:
                if self.active_goal:
                    self.camera_error = f'camera image conversion failed: {error}'
                    self.condition.notify_all()
            return
        with self.condition:
            self.latest_frame = frame
            self.frame_version += 1
            self.condition.notify_all()

    @staticmethod
    def _fill_result(result, outcome):
        """Copy a core result into the exact action wire types."""
        result.success = bool(outcome.success)
        result.patient_id = str(outcome.patient_id)
        result.reason = str(outcome.reason)
        return result

    def _execute(self, goal_handle):
        """Wait for a QR result, timeout, camera error, or cancellation."""
        result = Identify.Result()
        feedback = Identify.Feedback()
        outcome = None
        last_status = 'waiting for camera image'
        try:
            with self.condition:
                self.camera_error = ''
                seen_version = self.frame_version
                self.core.start(time.monotonic())
            self._publish_feedback(goal_handle, feedback, last_status)

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._fill_result(result, IdentifyResult(False, reason='canceled'))
                    goal_handle.canceled()
                    return result

                now = time.monotonic()
                outcome = self.core.poll(now)
                if outcome is not None:
                    if goal_handle.is_cancel_requested:
                        self._fill_result(result, IdentifyResult(False, reason='canceled'))
                        goal_handle.canceled()
                        return result
                    goal_handle.abort()
                    return self._fill_result(result, outcome)

                with self.condition:
                    if self.camera_error:
                        outcome = IdentifyResult(False, reason=self.camera_error)
                    elif self.frame_version == seen_version:
                        self.condition.wait(timeout=self.core.remaining_s(now))
                    if outcome is None and self.frame_version != seen_version:
                        frame = self.latest_frame
                        seen_version = self.frame_version
                    else:
                        frame = None

                if outcome is not None:
                    if goal_handle.is_cancel_requested:
                        self._fill_result(result, IdentifyResult(False, reason='canceled'))
                        goal_handle.canceled()
                        return result
                    goal_handle.abort()
                    return self._fill_result(result, outcome)
                if frame is None:
                    continue

                if last_status != 'scanning for QR':
                    last_status = 'scanning for QR'
                    self._publish_feedback(goal_handle, feedback, last_status)
                outcome = self.core.observe_frame(frame, time.monotonic())
                if outcome is None:
                    continue
                if goal_handle.is_cancel_requested:
                    self._fill_result(result, IdentifyResult(False, reason='canceled'))
                    goal_handle.canceled()
                    return result
                if outcome.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return self._fill_result(result, outcome)

            outcome = IdentifyResult(False, reason='ROS is shutting down')
            goal_handle.abort()
            return self._fill_result(result, outcome)
        finally:
            with self.condition:
                self.active_goal = False
                self.condition.notify_all()

    @staticmethod
    def _publish_feedback(goal_handle, feedback, status):
        """Publish the current identification status."""
        feedback.status = str(status)
        goal_handle.publish_feedback(feedback)


def main(args=None):
    """Run the patient identification action server."""
    rclpy.init(args=args)
    node = PatientId()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
