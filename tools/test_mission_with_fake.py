"""Run the mission state machine against the real bridge core and the fake ESP32.

Simulated time, no ROS. The harness plays the part of mission_manager: it carries out the
core's commands (measure and drawer commands go through the real bridge core to the fake
ESP32, with its servo drawers), and answers the action goals like simple stub servers.
"""

from pathlib import Path
import sys

import fake_esp32
from isocare_serial import codec
from test_bridge_with_fake import Loop

try:
    from isocare_mission import mission_core as mc
except ImportError:  # running from the repo without sourcing the workspace
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'isocare_mission'))
    from isocare_mission import mission_core as mc

STEP_S = 0.05
GOAL_S = {mc.NAV: 2.0, mc.IDENTIFY_ACTION: 1.0, mc.EVALUATE: 2.0, mc.APPROVAL: 2.0,
          mc.ALERT_ACTION: 2.0}


class Sim:
    """The ESP32 side is real (bridge core + fake); the action servers are scripted."""

    def __init__(self, evaluate_outcome=mc.OUTCOME_DISPENSE, approve=True, nav_fails=False):
        """Wire a MissionCore to a Loop whose fake ESP32 has servo drawers."""
        self.loop = Loop()
        self.loop.esp = fake_esp32.FakeEsp32(now=0.0, log=self.loop.log.append,
                                             servo_drawers=True)
        self.mission = mc.MissionCore(mc.MissionConfig(bed_patients={'A': 'P1'}))
        self.evaluate_outcome = evaluate_outcome
        self.approve = approve
        self.nav_fails = nav_fails
        self.goal_log = []          # (time, name, fields)
        self.states = [mc.IDLE]
        self._due = []              # [time, GoalResult]
        self._queues = {'measure': [], 'drawer': []}
        self._inflight = {'measure': None, 'drawer': None}
        self._seen = {'drawer': 0, 'status': 0}
        self.measure_calls = []     # (time, 'start' or 'stop', ok)

    @property
    def esp(self):
        """Return the fake ESP32."""
        return self.loop.esp

    def start(self, bed='A'):
        """Start a visit."""
        self._apply(self.mission.handle(mc.Start(bed), self.loop.now))

    def run_until(self, condition, limit_s=120.0):
        """Advance simulated time until condition() is true. Fail if it takes too long."""
        end = self.loop.now + limit_s
        while not condition():
            assert self.loop.now < end, f'timed out in state {self.mission.state}'
            self.step()

    def run_for(self, seconds):
        """Advance simulated time by a fixed amount."""
        end = self.loop.now + seconds
        while self.loop.now < end - 1e-9:
            self.step()

    def step(self):
        """Advance 50 ms and deliver everything that became due."""
        self.loop.run(STEP_S)
        now = self.loop.now
        self._deliver_bridge_answers(now)
        self._deliver_status(now)
        for item in [i for i in self._due if i[0] <= now + 1e-9]:
            self._due.remove(item)
            self._apply(self.mission.handle(item[1], now))
        self._apply(self.mission.tick(now))
        if self.states[-1] != self.mission.state:
            self.states.append(self.mission.state)

    # -- carrying out the core's commands ------------------------------------

    def _apply(self, commands):
        """Carry out commands like the node does."""
        for command in commands:
            if isinstance(command, mc.SendGoal):
                self.goal_log.append((self.loop.now, command.name, command.fields))
                self._due.append([self.loop.now + GOAL_S[command.name],
                                  self._scripted_result(command)])
            elif isinstance(command, mc.CancelGoal):
                self._due = [i for i in self._due if i[1].token != command.token]
            elif isinstance(command, mc.MeasureCmd):
                self._queues['measure'].append(command)
            elif isinstance(command, mc.DrawerCmd):
                self._queues['drawer'].append(command)
        self._send_next('measure')
        self._send_next('drawer')

    def _send_next(self, kind):
        """Send the next queued bridge command if none is waiting (one at a time per type)."""
        if self._inflight[kind] is not None or not self._queues[kind]:
            return
        command = self._queues[kind].pop(0)
        if kind == 'measure':
            message = codec.MeasureCmd(int(command.start), command.sensor_mask)
        else:
            message = codec.DrawerCmd(command.drawer_id, int(command.unlock), command.relock_s)
        self._inflight[kind] = (command, self.loop.core.submit(message))

    def _deliver_bridge_answers(self, now):
        """Turn finished bridge requests into events."""
        for kind in ('measure', 'drawer'):
            inflight = self._inflight[kind]
            if inflight is None or not inflight[1].done:
                continue
            command, request = inflight
            self._inflight[kind] = None
            ok, reason = request.success, f'status {request.status}'
            if kind == 'measure':
                self.measure_calls.append((now, 'start' if command.start else 'stop', ok))
                if command.start:
                    self._apply(self.mission.handle(mc.MeasureResult(command.token, ok, reason),
                                                    now))
            elif command.unlock:
                self._apply(self.mission.handle(mc.DrawerResult(command.token, ok, reason),
                                                now))
            self._send_next(kind)

    def _deliver_status(self, now):
        """Turn the ESP32's DRAWER_STATUS and STATUS frames into events."""
        drawers = self.loop.msgs(codec.DrawerStatus)
        for m in drawers[self._seen['drawer']:]:
            self._apply(self.mission.handle(mc.DrawerStatus(m.unlocked_mask, m.open_mask), now))
        self._seen['drawer'] = len(drawers)
        statuses = self.loop.msgs(codec.Status)
        for m in statuses[self._seen['status']:]:
            self._apply(self.mission.handle(mc.EspStatus(
                codec.StatusFlag.ESTOP in m.flags,
                codec.StatusFlag.VITALS_FAULT in m.flags), now))
        self._seen['status'] = len(statuses)

    def _scripted_result(self, command):
        """Return the answer the scripted action server gives for this goal."""
        name, token = command.name, command.token
        if name == mc.NAV:
            fails = self.nav_fails and command.fields['target_id'] != 'dock'
            return mc.GoalResult(name, token, not fails, 'blocked' if fails else '')
        if name == mc.IDENTIFY_ACTION:
            return mc.GoalResult(name, token, True, '', {'patient_id': 'P1'})
        if name == mc.EVALUATE:
            return mc.GoalResult(name, token, True, 'scripted', {
                'outcome': self.evaluate_outcome, 'drawer_id': 1, 'summary': 'HR 72'})
        if name == mc.APPROVAL:
            return mc.GoalResult(name, token, self.approve, '' if self.approve else 'declined')
        return mc.GoalResult(name, token, True)


def all_safe(esp):
    """Return True if every drawer is locked and shut and the sensors are off."""
    return (not any(esp.unlocked) and not any(esp.is_open) and not esp.measuring
            and not any(esp.open_at))


def test_full_visit_against_the_fake():
    """A whole visit: the drawer opens by itself, closes after 20 s, the robot goes home."""
    sim = Sim()
    sim.start()
    sim.run_until(lambda: sim.mission.state == mc.IDLE)
    assert sim.states == [mc.IDLE, mc.GO_TO_BED, mc.MEASURE, mc.DECIDE, mc.WAIT_APPROVAL,
                          mc.DISPENSE, mc.RETURN, mc.IDLE]
    assert sim.mission.outcome == mc.COMPLETED
    assert all_safe(sim.esp)
    starts = [t for t, kind, ok in sim.measure_calls if kind == 'start']
    stops = [t for t, kind, ok in sim.measure_calls if kind == 'stop']
    evaluate_at = [t for t, name, _ in sim.goal_log if name == mc.EVALUATE][0]
    assert starts and starts[0] <= evaluate_at           # sensors on before deciding
    assert stops and all(ok for _, kind, ok in sim.measure_calls)
    assert sim.goal_log[-1][1:] == (mc.NAV, {'target_id': 'dock'})


def test_hold_outcome_leaves_everything_locked_and_goes_home():
    """Abnormal vitals: no unlock ever reaches the ESP32, sensors off, robot goes home."""
    sim = Sim(evaluate_outcome=mc.OUTCOME_HOLD)
    sim.start()
    sim.run_until(lambda: sim.mission.state == mc.IDLE)
    assert sim.mission.outcome == mc.ALERTED
    assert all_safe(sim.esp)
    assert mc.APPROVAL not in [name for _, name, _ in sim.goal_log]
    assert not sim.esp.relock_deadline[1]


def test_declined_approval_never_unlocks_anything():
    """The nurse says no: the fake never saw an unlock, so nothing ever opened."""
    sim = Sim(approve=False)
    opened = []
    sim.start()
    while sim.mission.state != mc.IDLE:
        sim.step()
        opened.append(any(sim.esp.unlocked) or any(sim.esp.is_open) or any(sim.esp.open_at))
    assert sim.mission.outcome == mc.ALERTED
    assert not any(opened)


def test_estop_while_the_drawer_is_open_locks_and_closes_it():
    """E-stop mid-dispense: alert, the servo is told to lock (so it closes), robot goes home."""
    sim = Sim()
    sim.start()
    sim.run_until(lambda: sim.mission.state == mc.DISPENSE)
    sim.run_until(lambda: sim.esp.is_open[1])
    sim.esp.command('estop on', sim.loop.now)
    sim.run_until(lambda: sim.mission.state == mc.ALERT)
    assert 'E-stop' in sim.mission.alert_reason
    sim.run_for(1.0)
    assert not sim.esp.is_open[1] and not sim.esp.unlocked[1]
    assert not sim.esp.measuring


def test_vitals_sensor_fault_before_measuring_stops_the_visit_safely():
    """A faulty vitals sensor refuses START (REFUSED_SAFETY): alert, no drawer ever opens."""
    sim = Sim()
    sim.esp.command('fault vitals on', 0.0)
    sim.start()
    sim.run_until(lambda: sim.mission.state == mc.ALERT)
    assert 'would not start' in sim.mission.alert_reason
    assert mc.EVALUATE not in [name for _, name, _ in sim.goal_log]
    sim.run_until(lambda: sim.mission.state == mc.IDLE)
    assert all_safe(sim.esp)


def test_unlock_refused_during_a_fault_alerts_instead_of_dispensing():
    """A sensor fault appearing between approval and unlock makes the ESP32 refuse the unlock."""
    sim = Sim()
    sim.start()
    sim.run_until(lambda: sim.mission.state == mc.WAIT_APPROVAL)
    sim.esp.command('fault imu on', sim.loop.now)
    sim.run_until(lambda: sim.mission.state == mc.ALERT)
    assert 'refused' in sim.mission.alert_reason
    assert not any(sim.esp.is_open)
    sim.run_until(lambda: sim.mission.state == mc.IDLE)
    assert all_safe(sim.esp)


def test_navigation_failure_goes_home_after_the_alert():
    """Blocked path: alert, then home, with drawers never touched."""
    sim = Sim(nav_fails=True)
    sim.start()
    sim.run_until(lambda: sim.mission.state == mc.IDLE)
    assert sim.mission.outcome == mc.ALERTED
    assert [name for _, name, _ in sim.goal_log] == [mc.NAV, mc.ALERT_ACTION, mc.NAV]
    assert all_safe(sim.esp)
