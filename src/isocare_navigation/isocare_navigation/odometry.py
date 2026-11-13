"""odometry: turns the ESP32's raw ticks and gyro into the robot's pose.

It moves data between ROS and odometry_core.py and nothing else. Every decision
(time from the ESP32 clock, reboot and gap handling, heading source) lives in the core.

ROS side:
    subscribes  /sensor_data     isocare_interfaces/SensorData
                /esp_status      isocare_interfaces/EspStatus   (latched; imu_fault)
    publishes   /odom            nav_msgs/Odometry
                TF               odom -> base_link
"""

from geometry_msgs.msg import TransformStamped
from isocare_interfaces.msg import EspStatus, SensorData
from isocare_navigation import odometry_core
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class OdometryNode(Node):
    """The ROS node. See the module docstring."""

    def __init__(self):
        """Declare parameters, create the core and the ROS interfaces."""
        super().__init__('odometry')
        defaults = odometry_core.OdometryConfig()
        self.declare_parameter('ticks_per_mm', defaults.ticks_per_mm)
        self.declare_parameter('wheelbase_mm', defaults.wheelbase_mm)
        self.declare_parameter('max_gap_s', defaults.max_gap_s)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.core = odometry_core.OdometryCore(odometry_core.OdometryConfig(
            ticks_per_mm=self.get_parameter('ticks_per_mm').value,
            wheelbase_mm=self.get_parameter('wheelbase_mm').value,
            max_gap_s=self.get_parameter('max_gap_s').value))
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._imu_ok = True

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(SensorData, '/sensor_data', self._on_sensor_data, 50)
        self.create_subscription(EspStatus, '/esp_status', self._on_status, LATCHED)

    def _on_status(self, msg):
        """Remember whether the gyro works; log changes."""
        imu_ok = not msg.imu_fault
        if imu_ok != self._imu_ok:
            if imu_ok:
                self.get_logger().info('gyro is back: heading from the gyro again')
            else:
                self.get_logger().warn('gyro fault: heading from the wheel difference')
        self._imu_ok = imu_ok

    def _on_sensor_data(self, msg):
        """Feed one sample to the core and publish the new pose."""
        result = self.core.update(msg.esp_time_ms, msg.left_ticks, msg.right_ticks,
                                  msg.gyro_z_centideg_s, self._imu_ok)
        if result == odometry_core.DUPLICATE:
            return
        if result == odometry_core.REBOOT:
            self.get_logger().warn('ESP32 rebooted (its clock went backwards): '
                                   'pose kept, new baseline')
        elif result == odometry_core.GAP:
            self.get_logger().warn('gap in /sensor_data: pose kept, new baseline')
        self._publish(msg.header.stamp)

    def _publish(self, stamp):
        """Publish /odom and the odom -> base_link transform for the current pose."""
        pose = self.core.pose
        qx, qy, qz, qw = odometry_core.yaw_to_quaternion(pose.heading_rad)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = pose.x_m
        odom.pose.pose.position.y = pose.y_m
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = pose.v_m_s
        odom.twist.twist.angular.z = pose.omega_rad_s
        self._odom_pub.publish(odom)

        if self._publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._odom_frame
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = pose.x_m
            transform.transform.translation.y = pose.y_m
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self._tf.sendTransform(transform)


def main(args=None):
    """Run the odometry node until Ctrl-C."""
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
