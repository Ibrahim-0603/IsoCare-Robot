"""demo_stubs: throw-away action servers so a whole visit can run before the real nodes exist.

Stands in for patient_id (/patient/identify), decision_engine (/decision/evaluate) and
app_bridge (/nurse/approval, /nurse/alert); with stub_nav:=true also for navigator (/nav/go_to).
Each "works" for step_s seconds and then answers. It does not read /vitals or move anything.

Parameters (change them with --ros-args -p name:=value):
    step_s            seconds each stub works (default 2.0)
    stub_nav          also stub /nav/go_to (default false: the real navigator serves it)
    fail              names that fail: nav (to a bed; the trip to the dock still works),
                      dock (the trip to the dock), identify, evaluate, approval, alert
    skip              names NOT to serve, so a real node can take over (nav, identify,
                      evaluate, approval, alert), for example -p skip:="['evaluate']"
    evaluate_outcome  0 = dispense drawer 1, 1 = hold, 2 = unreliable, 3 = none needed (default 0)
    approve           false = the nurse declines (default true)
"""

from functools import partial
import time

from isocare_interfaces.action import Alert, Approval, Evaluate, GoTo, Identify
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

SERVERS = {
    'nav': (GoTo, '/nav/go_to'),
    'identify': (Identify, '/patient/identify'),
    'evaluate': (Evaluate, '/decision/evaluate'),
    'approval': (Approval, '/nurse/approval'),
    'alert': (Alert, '/nurse/alert'),
}


class DemoStubs(Node):
    """All five stub servers in one node."""

    def __init__(self):
        """Declare parameters and create the servers."""
        super().__init__('demo_stubs')
        self.declare_parameter('step_s', 2.0)
        self.declare_parameter('stub_nav', False)
        self.declare_parameter('fail', [''])
        self.declare_parameter('skip', [''])
        self.declare_parameter('evaluate_outcome', 0)
        self.declare_parameter('approve', True)
        group = ReentrantCallbackGroup()     # goals sleep, so they must not block each other
        stub_nav = bool(self.get_parameter('stub_nav').value)
        skipped = set(self.get_parameter('skip').value)
        self._servers = [
            ActionServer(self, action_type, topic, execute_callback=partial(self._execute, name),
                         cancel_callback=lambda goal: CancelResponse.ACCEPT,
                         callback_group=group)
            for name, (action_type, topic) in SERVERS.items()
            if (name != 'nav' or stub_nav) and name not in skipped]

    def _execute(self, name, goal_handle):
        """Work for step_s seconds (watching for a cancel), then answer."""
        step_s = float(self.get_parameter('step_s').value)
        self.get_logger().info(f'{name}: working for {step_s:.1f} s')
        end = time.monotonic() + step_s
        while time.monotonic() < end:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f'{name}: cancelled')
                return SERVERS[name][0].Result()
            time.sleep(0.05)
        result = self._answer(name, goal_handle.request)
        goal_handle.succeed()
        return result

    def _answer(self, name, request):
        """Build the result for one stub, honouring the fail parameter."""
        failing = set(self.get_parameter('fail').value)
        result = SERVERS[name][0].Result()
        if name == 'nav':
            fails = ('dock' in failing) if request.target_id == 'dock' else ('nav' in failing)
            result.success = not fails
            result.reason = 'stub: blocked' if fails else ''
        elif name == 'identify':
            result.success = 'identify' not in failing
            result.patient_id = 'P001' if result.success else ''
            result.reason = '' if result.success else 'stub: no QR in view'
        elif name == 'evaluate':
            result.success = 'evaluate' not in failing
            result.outcome = int(self.get_parameter('evaluate_outcome').value)
            result.drawer_id = 1
            result.summary = 'stub: HR 72, SpO2 98, 36.6 C'
            result.reason = 'stub'
        elif name == 'approval':
            result.approved = bool(self.get_parameter('approve').value) \
                and 'approval' not in failing
            result.reason = '' if result.approved else 'declined'
        else:
            result.acknowledged = 'alert' not in failing
        self.get_logger().info(f'{name}: answered')
        return result


def main(args=None):
    """Run the stub servers until Ctrl-C."""
    rclpy.init(args=args)
    node = DemoStubs()
    executor = MultiThreadedExecutor(num_threads=6)
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
