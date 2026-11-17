"""navigator: drives the robot to a bed or to the dock.

It moves data between ROS and nav_core.py and nothing else. Every decision (steering, turning
on the spot, blocked paths, no-progress, the final approach) lives in the core.

ROS side:
    action server  /nav/go_to       isocare_interfaces/GoTo   (target_id: a bed name or "dock")
    action client  none; it calls two services:
                   /plan_route      isocare_interfaces/PlanRoute
                   /odom/set_pose   isocare_interfaces/SetPose   (once at startup)
    subscribes     /odom            nav_msgs/Odometry
                   /sensor_data     isocare_interfaces/SensorData   (front range)
                   /esp_status      isocare_interfaces/EspStatus    (E-stop, obstacle stop)
                   /link_stats      isocare_interfaces/LinkStats
    publishes      /cmd_vel         geometry_msgs/Twist   (20 Hz while a goal is active only)

At startup it assumes the robot is parked on the dock: it asks the planner where the dock marker
is, computes the dock pose (dock_standoff_m in front of it, facing it) and gives it to odometry.
Parameters are the fields of nav_core.NavConfig plus set_pose_on_start and plan_timeout_s.
Floats must be written with a decimal point in launch and YAML files.
"""

import dataclasses
import math
import threading
import time

from geometry_msgs.msg import Twist
from isocare_interfaces.action import GoTo
from isocare_interfaces.msg import EspStatus, LinkStats, SensorData
from isocare_interfaces.srv import PlanRoute, SetPose
from isocare_navigation import nav_core
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
DATA_STALE_S = 0.5      # no /odom or /sensor_data for this long counts as a lost link


def yaw_from_quaternion(z, w):
    """Return the yaw of a rotation about the vertical axis given its z and w."""
    return 2.0 * math.atan2(z, w)


def plan_result(response):
    """Turn a PlanRoute response into a nav_core.PlanResult."""
    if not response.success or not len(response.names):
        return nav_core.PlanResult(False, response.reason or 'empty route')
    return nav_core.PlanResult(True, '', tuple(response.names), tuple(response.marker_x_m),
                               tuple(response.marker_y_m), tuple(response.marker_heading_rad))


class Navigator(Node):
    """The ROS node. See the module docstring."""

    def __init__(self):
        """Declare parameters, create the core and the ROS interfaces."""
        super().__init__('navigator')
        defaults = nav_core.NavConfig()
        values = {}
        for field in dataclasses.fields(nav_core.NavConfig):
            default = getattr(defaults, field.name)
            self.declare_parameter(field.name, default)
            values[field.name] = type(default)(self.get_parameter(field.name).value)
        self.declare_parameter('set_pose_on_start', True)
        self.declare_parameter('plan_timeout_s', 3.0)
        self.config = nav_core.NavConfig(**values)
        self.core = nav_core.NavCore(self.config, self.config.dock_name)
        self._set_pose_on_start = bool(self.get_parameter('set_pose_on_start').value)
        self._plan_timeout_s = float(self.get_parameter('plan_timeout_s').value)

        # Shared between the executor's threads: everything below is guarded by self._lock.
        self._lock = threading.RLock()
        self._pose = None                   # (x, y, yaw), None until /odom arrives
        self._odom_at = 0.0
        self._sensor_at = 0.0
        self._front_mm = None
        self._estop = False
        self._obstacle_stop = False
        self._link_ok = True
        self._ready = not self._set_pose_on_start
        self._startup_busy = False
        self._startup_failures = 0
        self._busy = False                  # a goal is being served
        self._done = None                   # nav_core.Done when the goal has ended
        self._plan_sent_at = None

        group = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._plan_client = self.create_client(PlanRoute, '/plan_route', callback_group=group)
        self._pose_client = self.create_client(SetPose, '/odom/set_pose', callback_group=group)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10, callback_group=group)
        self.create_subscription(SensorData, '/sensor_data', self._on_sensor, 50,
                                 callback_group=group)
        self.create_subscription(EspStatus, '/esp_status', self._on_status, LATCHED,
                                 callback_group=group)
        self.create_subscription(LinkStats, '/link_stats', self._on_link, 10,
                                 callback_group=group)
        self._server = ActionServer(
            self, GoTo, '/nav/go_to', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=group)
        self.create_timer(0.05, self._on_timer, callback_group=group)
        self.create_timer(1.0, self._startup, callback_group=group)

    # ------------------------------------------------------------------
    # Data in
    # ------------------------------------------------------------------

    def _on_odom(self, msg):
        """Keep the latest pose."""
        orientation = msg.pose.pose.orientation
        with self._lock:
            self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                          yaw_from_quaternion(orientation.z, orientation.w))
            self._odom_at = time.monotonic()

    def _on_sensor(self, msg):
        """Keep the latest front range (NaN means no valid reading)."""
        with self._lock:
            self._front_mm = None if math.isnan(msg.range_front_mm) else float(msg.range_front_mm)
            self._sensor_at = time.monotonic()

    def _on_status(self, msg):
        """Keep the E-stop and obstacle-stop flags."""
        with self._lock:
            self._estop = bool(msg.estop)
            self._obstacle_stop = bool(msg.obstacle_stop)

    def _on_link(self, msg):
        """Keep the serial link state."""
        with self._lock:
            self._link_ok = bool(msg.link_ok)

    # ------------------------------------------------------------------
    # The control cycle
    # ------------------------------------------------------------------

    def _on_timer(self):
        """Run one control cycle (20 Hz) while a goal is active."""
        now = time.monotonic()
        with self._lock:
            if self.core.state == nav_core.IDLE:
                return
            if (self.core.state in (nav_core.PLANNING, nav_core.REPLANNING)
                    and self._plan_sent_at is not None
                    and now - self._plan_sent_at > self._plan_timeout_s):
                self._plan_sent_at = None
                self._run(self.core.on_plan(nav_core.PlanResult(False, 'planner timeout'), now))
            fresh = (self._pose is not None and now - self._odom_at < DATA_STALE_S
                     and now - self._sensor_at < DATA_STALE_S)
            x, y, yaw = self._pose if self._pose is not None else (0.0, 0.0, 0.0)
            sample = nav_core.Sample(x, y, yaw, self._front_mm, self._estop,
                                     self._obstacle_stop, self._link_ok and fresh)
            self._run(self.core.update(sample, now))

    def _run(self, commands):
        """Carry out the core's commands (called with the lock held)."""
        for command in commands:
            if isinstance(command, nav_core.CmdVel):
                msg = Twist()
                msg.linear.x = float(command.linear)
                msg.angular.z = float(command.angular)
                self._cmd_pub.publish(msg)
            elif isinstance(command, nav_core.PlanRequest):
                self._request_plan(command)
            elif isinstance(command, nav_core.Done):
                self._done = command
            elif isinstance(command, nav_core.Log):
                self.get_logger().info(command.text)

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _request_plan(self, command):
        """Ask the route planner (non-blocking); the answer goes to the core."""
        if not self._plan_client.service_is_ready():
            self._run(self.core.on_plan(nav_core.PlanResult(False, 'planner unavailable'),
                                        time.monotonic()))
            return
        request = PlanRoute.Request()
        request.from_name = command.from_name
        request.to_name = command.to_name
        request.blocked_from = [a for a, _ in command.blocked]
        request.blocked_to = [b for _, b in command.blocked]
        self._plan_sent_at = time.monotonic()
        self._plan_client.call_async(request).add_done_callback(self._on_plan_answer)

    def _on_plan_answer(self, future):
        """Give the planner's answer to the core."""
        try:
            result = plan_result(future.result())
        except Exception as error:          # noqa: B902 - any failure is just "no route"
            result = nav_core.PlanResult(False, f'planner call failed: {error}')
        with self._lock:
            if self._plan_sent_at is None:
                return                      # it already timed out
            self._plan_sent_at = None
            self._run(self.core.on_plan(result, time.monotonic()))

    # ------------------------------------------------------------------
    # The action server
    # ------------------------------------------------------------------

    def _on_goal(self, goal_request):
        """Accept one goal at a time, and only once the start pose is set."""
        with self._lock:
            if self._busy or not self._ready:
                return GoalResponse.REJECT
            self._busy = True
            self._done = None
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        """Serve one goal: start it, give feedback, watch for a cancel, return the result."""
        with self._lock:
            self._run(self.core.start(goal_handle.request.target_id, time.monotonic()))
        cancelled = False
        while True:
            with self._lock:
                if self._done is not None:
                    done = self._done
                    break
                if goal_handle.is_cancel_requested and not cancelled:
                    cancelled = True
                    self._run(self.core.cancel(time.monotonic()))
                    continue
                status = self.core.status
            feedback = GoTo.Feedback()
            feedback.status = status
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
        with self._lock:
            self._busy = False
        result = GoTo.Result()
        result.success = bool(done.success)
        result.reason = done.reason
        if cancelled:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Startup: tell odometry where the dock is
    # ------------------------------------------------------------------

    def _startup(self):
        """Once the planner and odometry are up, set the pose to the dock pose."""
        with self._lock:
            if self._ready or self._startup_busy:
                return
            if not (self._plan_client.service_is_ready() and self._pose_client.service_is_ready()):
                return
            self._startup_busy = True
        request = PlanRoute.Request()
        request.from_name = request.to_name = self.config.dock_name
        self._plan_client.call_async(request).add_done_callback(self._on_dock_marker)

    def _on_dock_marker(self, future):
        """Send the dock pose to odometry, now that the planner has said where the marker is."""
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.reason)
            x, y, heading = nav_core.dock_pose(response.marker_x_m[0], response.marker_y_m[0],
                                               response.marker_heading_rad[0],
                                               self.config.dock_standoff_m)
        except Exception as error:          # noqa: B902 - report and retry
            self._startup_failed(f'cannot get the dock marker: {error}')
            return
        request = SetPose.Request()
        request.x_m, request.y_m, request.heading_rad = x, y, heading
        self._pose_client.call_async(request).add_done_callback(self._on_pose_set)

    def _on_pose_set(self, future):
        """Odometry answered: the navigator is ready."""
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
        except Exception as error:          # noqa: B902 - report and retry
            self._startup_failed(f'odometry refused the dock pose: {error}')
            return
        with self._lock:
            self._ready = True
            self._startup_busy = False
        self.get_logger().info(f'dock pose set: ready ({self.config.dock_name} assumed)')

    def _startup_failed(self, text):
        """Log (the first few times) and let the next timer tick retry."""
        with self._lock:
            self._startup_failures += 1
            self._startup_busy = False
            if self._startup_failures <= 3:
                self.get_logger().warn(f'{text}; will retry')


def main(args=None):
    """Run the navigator until Ctrl-C."""
    rclpy.init(args=args)
    node = Navigator()
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
