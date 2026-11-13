"""mission_manager: runs one visit to one bed, step by step.

It moves data between ROS and mission_core.py and nothing else. Every decision (step order,
timeouts, what counts as dispensed, when to alert) lives in the core.

ROS side:
    service      /mission/start      isocare_interfaces/StartVisit
    action clients (one per step):
                 /nav/go_to          GoTo          drive to a bed or the dock
                 /patient/identify   Identify      only in identify_mode 'qr'
                 /decision/evaluate  Evaluate      stable vitals + the doctor's rules
                 /nurse/approval     Approval      wait for the nurse's OK
                 /nurse/alert        Alert         tell the nurse, wait for the acknowledgment
    service clients (called one at a time, in order):
                 /vitals/measure     Measure       start and stop the sensors
                 /drawer/unlock      DrawerUnlock  unlock (open) or lock (close) a drawer
    subscribes   /drawer/status, /esp_status, /link_stats
    publishes    /mission/state      std_msgs/String (latched)
"""

from collections import deque
import time

from action_msgs.msg import GoalStatus
from isocare_interfaces.action import Alert, Approval, Evaluate, GoTo, Identify
from isocare_interfaces.msg import DrawerStatus, EspStatus, LinkStats
from isocare_interfaces.srv import DrawerUnlock, Measure, StartVisit
from isocare_mission import mission_core as mc
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

ACTIONS = {
    mc.NAV: (GoTo, '/nav/go_to'),
    mc.IDENTIFY_ACTION: (Identify, '/patient/identify'),
    mc.EVALUATE: (Evaluate, '/decision/evaluate'),
    mc.APPROVAL: (Approval, '/nurse/approval'),
    mc.ALERT_ACTION: (Alert, '/nurse/alert'),
}
SERVICE_STUCK_S = 5.0     # a service call with no reply after this long counts as failed


def goal_event(name, token, status, result):
    """Turn a finished action into a core event. Anything but SUCCEEDED counts as failed."""
    if status != GoalStatus.STATUS_SUCCEEDED:
        return mc.GoalResult(name, token, False, f'goal ended with status {status}')
    if name == mc.NAV:
        return mc.GoalResult(name, token, result.success, result.reason)
    if name == mc.IDENTIFY_ACTION:
        return mc.GoalResult(name, token, result.success, result.reason,
                             {'patient_id': result.patient_id})
    if name == mc.EVALUATE:
        return mc.GoalResult(name, token, result.success, result.reason,
                             {'outcome': result.outcome, 'drawer_id': result.drawer_id,
                              'summary': result.summary})
    if name == mc.APPROVAL:
        return mc.GoalResult(name, token, result.approved, result.reason)
    return mc.GoalResult(name, token, result.acknowledged)


def parse_bed_patients(entries):
    """Turn ['A=P001', 'B=P002', ''] into {'A': 'P001', 'B': 'P002'}. Empty entries are skipped."""
    mapping = {}
    for entry in entries:
        entry = entry.strip()
        if not entry:
            continue
        bed, separator, patient = entry.partition('=')
        if not separator or not bed.strip() or not patient.strip():
            raise ValueError(f"bed_patients entry '{entry}' must look like A=P001")
        mapping[bed.strip()] = patient.strip()
    return mapping


class OneAtATime:
    """Call one service at a time, in order.

    The bridge cannot promise the order of simultaneous requests of one type, and the
    manager sometimes sends several in a row (lock all four drawers, stop then start).
    """

    def __init__(self, client):
        """Wrap a service client."""
        self._client = client
        self._pending = deque()
        self._busy_since = None
        self._current = None

    def submit(self, request, done):
        """Queue a request. done(ok, reason) is called with the result."""
        self._pending.append((request, done))
        self._pump(time.monotonic())

    def check_stuck(self, now):
        """Fail the call in flight if it has had no reply for SERVICE_STUCK_S."""
        if self._busy_since is not None and now - self._busy_since > SERVICE_STUCK_S:
            done = self._current[1]
            self._busy_since = None
            self._current = None
            done(False, 'no reply')
            self._pump(now)

    def _pump(self, now):
        """Send the next queued request if none is in flight."""
        while self._busy_since is None and self._pending:
            request, done = self._pending.popleft()
            if not self._client.service_is_ready():
                done(False, 'service unavailable')
                continue
            entry = (request, done)
            self._current = entry
            self._busy_since = now
            future = self._client.call_async(request)
            future.add_done_callback(lambda f, e=entry: self._finished(f, e))

    def _finished(self, future, entry):
        """Handle a reply (or an error)."""
        if self._current is not entry:
            return                          # already given up on as stuck
        self._busy_since = None
        self._current = None
        try:
            response = future.result()
        except Exception as error:          # noqa: B902 - any failure is just "failed"
            entry[1](False, f'call failed: {error}')
        else:
            entry[1](bool(response.success), f'status {response.status}')
        self._pump(time.monotonic())


class MissionManager(Node):
    """The ROS node. See the module docstring."""

    def __init__(self):
        """Declare parameters, create the core and the ROS interfaces."""
        super().__init__('mission_manager')
        defaults = mc.MissionConfig()
        # Floats must be written with a decimal point in launch and YAML files.
        self.declare_parameter('identify_mode', defaults.identify_mode)
        self.declare_parameter('bed_patients', [''])      # entries like 'A=P001'
        self.declare_parameter('dock_id', defaults.dock_id)
        self.declare_parameter('drawer_hold_s', defaults.drawer_hold_s)
        self.declare_parameter('drawer_unlock_timeout_s', defaults.drawer_unlock_timeout_s)
        self.declare_parameter('drawer_open_timeout_s', defaults.drawer_open_timeout_s)
        self.declare_parameter('drawer_close_margin_s', defaults.drawer_close_margin_s)
        self.declare_parameter('measure_start_timeout_s', defaults.measure_start_timeout_s)
        self.declare_parameter('nav_timeout_s', defaults.nav_timeout_s)
        self.declare_parameter('identify_timeout_s', defaults.identify_timeout_s)
        self.declare_parameter('decide_timeout_s', defaults.decide_timeout_s)
        self.declare_parameter('approval_timeout_s', defaults.approval_timeout_s)

        def get(name):
            return self.get_parameter(name).value

        self.core = mc.MissionCore(mc.MissionConfig(
            identify_mode=get('identify_mode'),
            bed_patients=parse_bed_patients(get('bed_patients')),
            dock_id=get('dock_id'),
            drawer_hold_s=int(get('drawer_hold_s')),
            drawer_unlock_timeout_s=float(get('drawer_unlock_timeout_s')),
            drawer_open_timeout_s=float(get('drawer_open_timeout_s')),
            drawer_close_margin_s=float(get('drawer_close_margin_s')),
            measure_start_timeout_s=float(get('measure_start_timeout_s')),
            nav_timeout_s=float(get('nav_timeout_s')),
            identify_timeout_s=float(get('identify_timeout_s')),
            decide_timeout_s=float(get('decide_timeout_s')),
            approval_timeout_s=float(get('approval_timeout_s'))))

        self._inbox = deque()               # events waiting for the core
        self._draining = False
        self._goals = {}                    # action name -> {'token', 'handle', 'cancel'}
        self._last_state = None

        self._action_clients = {name: ActionClient(self, action_type, topic)
                         for name, (action_type, topic) in ACTIONS.items()}
        self._measure = OneAtATime(self.create_client(Measure, '/vitals/measure'))
        self._drawer = OneAtATime(self.create_client(DrawerUnlock, '/drawer/unlock'))
        self._state_pub = self.create_publisher(String, '/mission/state', LATCHED)
        self.create_service(StartVisit, '/mission/start', self._on_start)
        self.create_subscription(DrawerStatus, '/drawer/status', self._on_drawer_status, LATCHED)
        self.create_subscription(EspStatus, '/esp_status', self._on_esp_status, LATCHED)
        self.create_subscription(LinkStats, '/link_stats', self._on_link_stats, 10)
        self.create_timer(0.1, self._on_timer)
        self._publish_state()

    # ------------------------------------------------------------------
    # Events in
    # ------------------------------------------------------------------

    def _on_start(self, request, response):
        """Handle /mission/start."""
        if not self.core.can_start():
            response.accepted = False
            response.message = f'busy: state is {self.core.state}'
            return response
        self._feed(mc.Start(request.bed_id))
        response.accepted = True
        response.message = f'state is {self.core.state}'
        return response

    def _on_drawer_status(self, msg):
        """Pack the four booleans of each array into bit masks."""
        unlocked = sum(1 << i for i in range(4) if msg.unlocked[i])
        opened = sum(1 << i for i in range(4) if msg.open[i])
        self._feed(mc.DrawerStatus(unlocked, opened))

    def _on_esp_status(self, msg):
        """Pass on the two safety flags the manager cares about."""
        self._feed(mc.EspStatus(bool(msg.estop), bool(msg.vitals_fault)))

    def _on_link_stats(self, msg):
        """Tell the core when the serial link is down."""
        if not msg.link_ok:
            self._feed(mc.LinkLost())

    def _on_timer(self):
        """Check deadlines and service calls that got no reply."""
        now = time.monotonic()
        self._measure.check_stuck(now)
        self._drawer.check_stuck(now)
        self._execute(self.core.tick(now))
        self._feed()

    def _feed(self, event=None):
        """Give events to the core one at a time and carry out what it asks for."""
        if event is not None:
            self._inbox.append(event)
        if self._draining:
            return                          # an outer call is already working through them
        self._draining = True
        try:
            while self._inbox:
                self._execute(self.core.handle(self._inbox.popleft(), time.monotonic()))
        finally:
            self._draining = False
        self._publish_state()

    # ------------------------------------------------------------------
    # Commands out
    # ------------------------------------------------------------------

    def _execute(self, commands):
        """Carry out the core's commands in order."""
        for command in commands:
            if isinstance(command, mc.SendGoal):
                self._send_goal(command)
            elif isinstance(command, mc.CancelGoal):
                self._cancel_goal(command)
            elif isinstance(command, mc.MeasureCmd):
                self._call_measure(command)
            elif isinstance(command, mc.DrawerCmd):
                self._call_drawer(command)
            elif isinstance(command, mc.Log):
                self.get_logger().info(command.text)

    def _send_goal(self, command):
        """Send an action goal; remember it so it can be cancelled."""
        client = self._action_clients[command.name]
        if not client.server_is_ready():
            self._inbox.append(mc.GoalResult(command.name, command.token, False,
                                             'action server unavailable'))
            return
        goal = ACTIONS[command.name][0].Goal()
        for key, value in command.fields.items():
            setattr(goal, key, value)
        entry = {'token': command.token, 'handle': None, 'cancel': False}
        self._goals[command.name] = entry
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f, n=command.name, e=entry: self._on_accepted(n, e, f))

    def _on_accepted(self, name, entry, future):
        """Handle the server accepting or rejecting the goal."""
        handle = future.result()
        if not handle.accepted:
            self._feed(mc.GoalResult(name, entry['token'], False, 'goal rejected'))
            return
        entry['handle'] = handle
        if entry['cancel']:                 # cancel was asked for before the goal was accepted
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f, n=name, e=entry: self._on_goal_done(n, e, f))

    def _on_goal_done(self, name, entry, future):
        """Handle a finished goal (also after a cancel; the core ignores those)."""
        wrapper = future.result()
        self._feed(goal_event(name, entry['token'], wrapper.status, wrapper.result))

    def _cancel_goal(self, command):
        """Cancel a goal, now or as soon as it is accepted."""
        entry = self._goals.get(command.name)
        if entry is None or entry['token'] != command.token:
            return
        entry['cancel'] = True
        if entry['handle'] is not None:
            entry['handle'].cancel_goal_async()

    def _call_measure(self, command):
        """Start or stop the sensors; only the START answer matters to the core."""
        request = Measure.Request()
        request.action = Measure.Request.ACTION_START if command.start \
            else Measure.Request.ACTION_STOP
        request.sensor_mask = command.sensor_mask
        self._measure.submit(
            request, lambda ok, reason, c=command: self._measure_done(c, ok, reason))

    def _measure_done(self, command, ok, reason):
        """Handle the end of a /vitals/measure call."""
        if command.start:
            self._feed(mc.MeasureResult(command.token, ok, reason))
        elif not ok:
            self.get_logger().warn(f'could not stop measuring: {reason}')

    def _call_drawer(self, command):
        """Unlock (open) or lock (close) a drawer."""
        request = DrawerUnlock.Request()
        request.drawer_id = command.drawer_id
        request.action = DrawerUnlock.Request.ACTION_UNLOCK if command.unlock \
            else DrawerUnlock.Request.ACTION_LOCK
        request.relock_s = command.relock_s
        self._drawer.submit(
            request, lambda ok, reason, c=command: self._drawer_done(c, ok, reason))

    def _drawer_done(self, command, ok, reason):
        """Handle the end of a /drawer/unlock call."""
        if command.unlock:
            self._feed(mc.DrawerResult(command.token, ok, reason))
        elif not ok:
            self.get_logger().error(f'could not lock drawer {command.drawer_id}: {reason}')

    def _publish_state(self):
        """Publish the state when it changes and log the change."""
        if self.core.state == self._last_state:
            return
        self._last_state = self.core.state
        msg = String()
        msg.data = self.core.state
        self._state_pub.publish(msg)
        self.get_logger().info(f'state: {self.core.state}')


def main(args=None):
    """Run the mission manager until Ctrl-C."""
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
