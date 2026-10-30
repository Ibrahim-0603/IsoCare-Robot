"""serial_bridge: the only node that talks to the ESP32.

It moves data between ROS and the serial port and nothing else. Every decision
(wheel speeds, stale /cmd_vel, ACK resends, link health) lives in bridge_core.py.

ROS side:
    subscribes  /cmd_vel                 geometry_msgs/Twist
    publishes   /sensor_data             isocare_interfaces/SensorData
                /vitals                  isocare_interfaces/Vitals
                /drawer/status           isocare_interfaces/DrawerStatus   (latched)
                /esp_status              isocare_interfaces/EspStatus      (latched)
                /link_stats              isocare_interfaces/LinkStats      (1 Hz)
    services    /drawer/unlock           isocare_interfaces/DrawerUnlock
                /vitals/measure          isocare_interfaces/Measure
                /system/cmd              isocare_interfaces/SystemCmd

One I/O thread owns the serial port: it reads bytes, publishes what they decode
to, and writes whatever the core says is due (the 20 Hz heartbeat, command
sends and resends). Service callbacks only submit a request and wait for it.
"""

import threading
import time

from geometry_msgs.msg import Twist
from isocare_interfaces.msg import DrawerStatus, EspStatus, LinkStats, SensorData, Vitals
from isocare_interfaces.srv import DrawerUnlock, Measure, SystemCmd
from isocare_serial import bridge_core, codec
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import serial

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
SERVICE_WAIT_S = 3.0   # safety net: a request normally finishes within about 0.85 s


class SerialBridge(Node):
    """The ROS node. See the module docstring."""

    def __init__(self):
        """Declare parameters, create the ROS interfaces and start the I/O thread."""
        super().__init__('serial_bridge')
        defaults = bridge_core.BridgeConfig()
        self.declare_parameter('port', '/tmp/esp_pi')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheelbase_mm', defaults.wheelbase_mm)
        self.declare_parameter('max_speed_mm_s', defaults.max_speed_mm_s)
        self.declare_parameter('cmd_vel_stale_s', defaults.cmd_vel_stale_s)
        self.declare_parameter('link_lost_s', defaults.link_lost_s)
        self.declare_parameter('ack_timeout_s', defaults.ack_timeout_s)
        self.declare_parameter('max_resends', defaults.max_resends)

        self._port = self.get_parameter('port').value
        self._baud = self.get_parameter('baud').value
        self.core = bridge_core.BridgeCore(bridge_core.BridgeConfig(
            wheelbase_mm=self.get_parameter('wheelbase_mm').value,
            max_speed_mm_s=self.get_parameter('max_speed_mm_s').value,
            cmd_vel_stale_s=self.get_parameter('cmd_vel_stale_s').value,
            link_lost_s=self.get_parameter('link_lost_s').value,
            ack_timeout_s=self.get_parameter('ack_timeout_s').value,
            max_resends=self.get_parameter('max_resends').value))

        self._types = {'sensor_data': SensorData, 'vitals': Vitals,
                       'drawer_status': DrawerStatus, 'esp_status': EspStatus}
        self._pubs = {
            'sensor_data': self.create_publisher(SensorData, '/sensor_data', 50),
            'vitals': self.create_publisher(Vitals, '/vitals', 10),
            'drawer_status': self.create_publisher(DrawerStatus, '/drawer/status', LATCHED),
            'esp_status': self.create_publisher(EspStatus, '/esp_status', LATCHED),
        }
        self._link_stats_pub = self.create_publisher(LinkStats, '/link_stats', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        # Services block while waiting for an ACK, so they get their own callback
        # group. With a multi-threaded executor, /cmd_vel is still delivered meanwhile.
        group = ReentrantCallbackGroup()
        self.create_service(DrawerUnlock, '/drawer/unlock', self._on_drawer, callback_group=group)
        self.create_service(Measure, '/vitals/measure', self._on_measure, callback_group=group)
        self.create_service(SystemCmd, '/system/cmd', self._on_system, callback_group=group)
        # An ACK carries no sequence number, so only one request per type may be in flight.
        self._type_locks = {codec.MsgType.DRAWER_CMD: threading.Lock(),
                            codec.MsgType.MEASURE_CMD: threading.Lock(),
                            codec.MsgType.SYSTEM_CMD: threading.Lock()}

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._io_loop, name='serial_io', daemon=True)
        self._thread.start()

    def shutdown(self):
        """Stop the I/O thread and close the port."""
        self._stop_event.set()
        self._thread.join(timeout=2.0)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_cmd_vel(self, msg):
        """Store the newest velocity command; the I/O thread sends it at 20 Hz."""
        self.core.set_cmd_vel(msg.linear.x, msg.angular.z, time.monotonic())

    def _call(self, message):
        """Submit a command, wait for its ACK or timeout, and return the final status."""
        with self._type_locks[message.TYPE]:
            request = self.core.submit(message)
            if not request.wait(timeout=SERVICE_WAIT_S):
                self.core.cancel(request)    # the I/O thread is stuck; do not leave it pending
                return bridge_core.STATUS_TIMEOUT
            return request.status

    def _answer(self, response, build):
        """Fill a service response: build() makes the codec message, or raises ProtocolError."""
        try:
            message = build()
        except codec.ProtocolError as error:
            self.get_logger().warn(f'rejected request: {error}')
            response.success = False
            response.status = int(codec.AckStatus.BAD_PAYLOAD)
            return response
        status = self._call(message)
        response.success = status == codec.AckStatus.OK
        response.status = int(status)
        return response

    def _on_drawer(self, request, response):
        """Handle /drawer/unlock (also locks, depending on request.action)."""
        return self._answer(response, lambda: codec.DrawerCmd(
            request.drawer_id, request.action, request.relock_s))

    def _on_measure(self, request, response):
        """Handle /vitals/measure."""
        return self._answer(response, lambda: codec.MeasureCmd(
            request.action, request.sensor_mask))

    def _on_system(self, request, response):
        """Handle /system/cmd."""
        return self._answer(response, lambda: codec.SystemCmd(request.command))

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish(self, message):
        """Publish one decoded ESP32 message on its topic."""
        kind, fields = bridge_core.message_fields(message)
        if kind is None:
            return
        ros_msg = self._types[kind]()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        for name, value in fields.items():
            setattr(ros_msg, name, value)
        self._pubs[kind].publish(ros_msg)

    def _publish_link_stats(self, now):
        """Publish the link health numbers."""
        ros_msg = LinkStats()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        for name, value in bridge_core.link_stats_fields(self.core.snapshot(now)).items():
            setattr(ros_msg, name, value)
        self._link_stats_pub.publish(ros_msg)

    # ------------------------------------------------------------------
    # The I/O thread
    # ------------------------------------------------------------------

    def _open_port(self):
        """Open the serial port without blocking. Return None if it is not available."""
        try:
            link = serial.Serial(self._port, self._baud, timeout=0)
        except (serial.SerialException, OSError) as error:
            self.get_logger().warn(f'cannot open {self._port}: {error}',
                                   throttle_duration_sec=10.0)
            return None
        self.get_logger().info(f'opened {self._port} at {self._baud} baud')
        return link

    def _drop_port(self, link, error):
        """Log a port failure, close the port and return None so it is reopened later."""
        self.get_logger().error(f'serial port failed: {error}; will retry')
        try:
            link.close()
        except (serial.SerialException, OSError):
            pass
        return None

    def _io_loop(self):
        """Read, publish, write, repeat. Owns the serial port and survives unplugging."""
        link = None
        next_open = 0.0
        next_stats = 0.0
        was_ok = None
        reboots = 0
        while not self._stop_event.is_set():
            now = time.monotonic()
            if link is None and now >= next_open:
                link = self._open_port()
                next_open = now + 1.0
            data = b''
            if link is not None:
                try:
                    data = link.read(4096)
                except (serial.SerialException, OSError) as error:
                    link = self._drop_port(link, error)
            for message in self.core.feed(data, now):
                self._publish(message)
            out = self.core.poll(now)
            if out and link is not None:
                try:
                    link.write(out)
                except (serial.SerialException, OSError) as error:
                    link = self._drop_port(link, error)
            self._report_changes(now, was_ok, reboots)
            was_ok = self.core.link_ok(now)
            reboots = self.core.monitor.reboots
            if now >= next_stats:
                self._publish_link_stats(now)
                next_stats = now + 1.0
            time.sleep(0.002)
        if link is not None:
            link.close()

    def _report_changes(self, now, was_ok, reboots):
        """Log link up/down transitions and ESP32 reboots."""
        link_ok = self.core.link_ok(now)
        if link_ok != was_ok:
            if link_ok:
                self.get_logger().info('ESP32 link up')
            elif was_ok is None:
                self.get_logger().info('waiting for ESP32 data')
            else:
                self.get_logger().warn(
                    'ESP32 link lost: heartbeat stopped, ESP32 will stop motors')
        if self.core.monitor.reboots != reboots:
            self.get_logger().warn('ESP32 rebooted (its clock went backwards)')


def main(args=None):
    """Run the serial_bridge node until Ctrl-C."""
    rclpy.init(args=args)
    node = SerialBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
