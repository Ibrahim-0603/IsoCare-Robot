"""decision_engine: judges the vitals and applies the doctor's rules for one bed.

It moves data between ROS and decision_core.py and nothing else. Every decision (stability,
abnormal bands, drug rules, dose intervals) lives in the core.

ROS side:
    action server  /decision/evaluate  isocare_interfaces/Evaluate
    subscribes     /vitals             Vitals          (the manager has started the sensors)
                   /mission/state      std_msgs/String (latched): a dose counts when the visit
                                       goes DISPENSE -> RETURN
Parameters:
    rules_file, bands_file, patients_file   YAML files (default: the ones in this package's config)
    history_db      SQLite file with the last dose per patient and drug
                    (default ~/.isocare/dose_history.db)
    settle_s, window, deadline_s            see DecisionConfig
    hr_tolerance, spo2_tolerance, temp_tolerance   how close the readings of one vital must be
"""

import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from isocare_decision import decision_config, decision_core as dc
from isocare_decision.dose_history import DoseHistory
from isocare_interfaces.action import Evaluate
from isocare_interfaces.msg import Vitals
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
POLL_S = 0.1


def sample_from_msg(msg):
    """Turn a Vitals message into the plain Sample the core uses."""
    return dc.Sample(
        hr_valid=bool(msg.hr_valid), spo2_valid=bool(msg.spo2_valid),
        ir_valid=bool(msg.ir_temp_valid), contact_valid=bool(msg.contact_temp_valid),
        finger=bool(msg.finger_detected), hr=float(msg.heart_rate_bpm),
        spo2=float(msg.spo2_percent), ir_c=msg.temp_ir_centi_c / 100.0,
        contact_c=msg.temp_contact_centi_c / 100.0)


def fill_result(result, core_result):
    """Copy a core Result into the action result with the exact types the .action declares."""
    result.success = bool(core_result.success)
    result.outcome = int(core_result.outcome)
    result.drawer_id = int(core_result.drawer_id)
    result.summary = str(core_result.summary)
    result.reason = str(core_result.reason)
    return result


class DecisionEngine(Node):
    """The ROS node. See the module docstring."""

    def __init__(self):
        """Load and check the config files, then offer the action."""
        super().__init__('decision_engine')
        share = get_package_share_directory('isocare_decision')
        for name, default in (('rules_file', 'rules.yaml'), ('bands_file', 'bands.yaml'),
                              ('patients_file', 'patients.yaml')):
            self.declare_parameter(name, os.path.join(share, 'config', default))
        self.declare_parameter('history_db', os.path.expanduser('~/.isocare/dose_history.db'))
        self.declare_parameter('settle_s', 5.0)
        self.declare_parameter('window', 5)
        self.declare_parameter('deadline_s', 80.0)
        self.declare_parameter('hr_tolerance', 5.0)
        self.declare_parameter('spo2_tolerance', 2.0)
        self.declare_parameter('temp_tolerance', 0.3)
        param = self.get_parameter
        rules = decision_config.load_rules(param('rules_file').value)    # raises, names the file
        bands = decision_config.load_bands(param('bands_file').value)
        patients = decision_config.load_patients(param('patients_file').value)
        config = dc.DecisionConfig(
            settle_s=float(param('settle_s').value), window=int(param('window').value),
            deadline_s=float(param('deadline_s').value),
            tolerance={dc.HR: float(param('hr_tolerance').value),
                       dc.SPO2: float(param('spo2_tolerance').value),
                       dc.TEMP: float(param('temp_tolerance').value)})
        self.history = DoseHistory(param('history_db').value)
        self.core = dc.DecisionCore(config, rules, bands, patients, self.history)
        self.lock = threading.Lock()
        group = ReentrantCallbackGroup()
        self.create_subscription(Vitals, '/vitals', self._on_vitals, 10, callback_group=group)
        self.create_subscription(String, '/mission/state', self._on_state, LATCHED,
                                 callback_group=group)
        self.server = ActionServer(self, Evaluate, '/decision/evaluate',
                                   execute_callback=self._execute,
                                   goal_callback=self._on_goal, cancel_callback=self._on_cancel,
                                   callback_group=group)
        self.get_logger().info(f'decision_engine ready: {len(rules)} rule(s), '
                               f'{len(patients)} patient(s), history {param("history_db").value}')

    def _on_goal(self, goal_request):
        """Accept a goal unless one is already running (the manager sends one at a time)."""
        with self.lock:
            return GoalResponse.REJECT if self.core.active else GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        """Allow every cancel."""
        return CancelResponse.ACCEPT

    def _on_vitals(self, msg):
        """Feed one vitals sample to the core."""
        with self.lock:
            self.core.add_sample(sample_from_msg(msg), time.monotonic())

    def _on_state(self, msg):
        """Tell the core about the visit state so it can record doses that really happened."""
        with self.lock:
            self.core.on_mission_state(str(msg.data), time.time())

    def _execute(self, goal_handle):
        """Run one evaluation: wait for the core to decide, publishing feedback."""
        request = goal_handle.request
        result = Evaluate.Result()
        with self.lock:
            failed = self.core.start(str(request.patient_id), str(request.bed_id),
                                     time.monotonic())
        if failed is not None:
            goal_handle.succeed()           # a normal answer: the manager drops the reason of an
            return fill_result(result, failed)      # aborted goal, so success=False travels here
        feedback = Evaluate.Feedback()
        while rclpy.ok():
            with self.lock:
                if goal_handle.is_cancel_requested:
                    self.core.cancel()
                    cancelled = True
                else:
                    cancelled = False
                    now = time.monotonic()
                    outcome = self.core.poll(now, time.time())
                    status, stable = self.core.feedback(now)
            if cancelled:
                goal_handle.canceled()
                return result
            if outcome is not None:
                self.get_logger().info(f'evaluation done: outcome {outcome.outcome}, '
                                       f'{outcome.summary} {outcome.reason}'.strip())
                goal_handle.succeed()
                return fill_result(result, outcome)
            feedback.status = str(status)
            feedback.stable_readings = int(stable)
            goal_handle.publish_feedback(feedback)
            time.sleep(POLL_S)
        with self.lock:
            self.core.cancel()
        goal_handle.abort()
        return fill_result(result, dc.Result(False, reason='shutting down'))


def main():
    """Start the node."""
    rclpy.init()
    node = DecisionEngine()
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
