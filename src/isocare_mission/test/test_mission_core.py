"""Tests for isocare_mission.mission_core. Plain pytest, no ROS, simulated time."""

from pathlib import Path
import sys

try:
    from isocare_mission import mission_core as mc
except ImportError:  # running from the repo without sourcing the workspace
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isocare_mission import mission_core as mc
import pytest


class Visit:
    """Drive a MissionCore by hand: feed events, keep the commands and the last goal tokens."""

    def __init__(self, **config):
        """Create a core for bed A with patient P1 assigned (nurse identity mode)."""
        config.setdefault('bed_patients', {'A': 'P1', 'B': 'P2'})
        self.core = mc.MissionCore(mc.MissionConfig(**config))
        self.now = 0.0
        self.commands = []      # every command so far
        self.last = []          # the commands of the latest step
        self.tokens = {}        # action name -> token of the latest goal sent

    def send(self, event):
        """Feed an event and remember what the core asked for."""
        self.last = self.core.handle(event, self.now)
        self._remember()
        return self.last

    def wait(self, seconds):
        """Advance simulated time and let the core check its deadline."""
        self.now += seconds
        self.last = self.core.tick(self.now)
        self._remember()
        return self.last

    def _remember(self):
        for command in self.last:
            if isinstance(command, mc.SendGoal):
                self.tokens[command.name] = command.token
        self.commands += self.last

    def result(self, name, ok=True, reason='', **data):
        """Answer the latest goal of this name."""
        return self.send(mc.GoalResult(name, self.tokens[name], ok, reason, data))

    def goals(self, name=None):
        """Return the goals sent so far (optionally of one name)."""
        return [c for c in self.commands if isinstance(c, mc.SendGoal)
                and (name is None or c.name == name)]

    def last_of(self, cls):
        """Return the commands of one class in the latest step."""
        return [c for c in self.last if isinstance(c, cls)]

    def measure_answer(self, ok=True, reason=''):
        """Answer the latest START command."""
        start = [c for c in self.commands if isinstance(c, mc.MeasureCmd) and c.start][-1]
        return self.send(mc.MeasureResult(start.token, ok, reason))

    def drawer_answer(self, ok=True, reason=''):
        """Answer the latest unlock command."""
        unlock = [c for c in self.commands if isinstance(c, mc.DrawerCmd) and c.unlock][-1]
        return self.send(mc.DrawerResult(unlock.token, ok, reason))

    def to_decide(self):
        """Run the visit up to the point where decision_engine is working."""
        self.send(mc.Start('A'))
        self.result(mc.NAV)
        self.measure_answer()
        assert self.core.state == mc.DECIDE

    def to_dispense(self, drawer=1):
        """Run the visit up to the point where the drawer was told to unlock."""
        self.to_decide()
        self.result(mc.EVALUATE, outcome=mc.OUTCOME_DISPENSE, drawer_id=drawer,
                    summary='HR 72')
        self.result(mc.APPROVAL)
        assert self.core.state == mc.DISPENSE


def locked_all(commands):
    """Return True if the commands lock all four drawers."""
    locks = {c.drawer_id for c in commands if isinstance(c, mc.DrawerCmd) and not c.unlock}
    return locks == {0, 1, 2, 3}


# --------------------------------------------------------------------------
# Happy path
# --------------------------------------------------------------------------

def test_full_visit_in_nurse_mode():
    """The whole visit, step by step, with the commands each step produces."""
    v = Visit()
    assert v.core.state == mc.IDLE
    v.send(mc.Start('A'))
    assert v.core.state == mc.GO_TO_BED
    assert v.goals(mc.NAV)[0].fields == {'target_id': 'A'}

    v.result(mc.NAV)                                  # nurse mode: no identify goal
    assert v.core.state == mc.MEASURE
    assert not v.goals(mc.IDENTIFY_ACTION)
    start = v.last_of(mc.MeasureCmd)[0]
    assert start.start and start.sensor_mask == 7

    v.measure_answer()
    assert v.core.state == mc.DECIDE
    assert v.goals(mc.EVALUATE)[0].fields == {'patient_id': 'P1', 'bed_id': 'A'}

    v.result(mc.EVALUATE, outcome=mc.OUTCOME_DISPENSE, drawer_id=1, summary='HR 72, 36.6 C')
    assert v.core.state == mc.WAIT_APPROVAL
    stop = v.last_of(mc.MeasureCmd)
    assert len(stop) == 1 and not stop[0].start             # sensors are switched off
    approval = v.goals(mc.APPROVAL)[0].fields
    assert approval == {'patient_id': 'P1', 'bed_id': 'A', 'drawer_id': 1,
                        'identity_source': 'nurse', 'summary': 'HR 72, 36.6 C'}

    v.result(mc.APPROVAL)
    assert v.core.state == mc.DISPENSE
    unlock = v.last_of(mc.DrawerCmd)[0]
    assert (unlock.drawer_id, unlock.unlock, unlock.relock_s) == (1, True, 20)

    v.drawer_answer()
    v.send(mc.DrawerStatus(unlocked_mask=0b010, open_mask=0b010))
    assert v.core.state == mc.DISPENSE
    v.send(mc.DrawerStatus(unlocked_mask=0, open_mask=0))      # closed again
    assert v.core.state == mc.RETURN
    assert v.goals(mc.NAV)[-1].fields == {'target_id': 'dock'}

    v.result(mc.NAV)
    assert v.core.state == mc.IDLE
    assert v.core.outcome == mc.COMPLETED
    assert v.core.last_patient_by_bed == {'A': 'P1'}


def test_qr_mode_uses_the_identify_action():
    """In qr mode the patient id comes from the identify action."""
    v = Visit(identify_mode='qr')
    v.send(mc.Start('A'))
    v.result(mc.NAV)
    assert v.core.state == mc.IDENTIFY
    assert v.goals(mc.IDENTIFY_ACTION)[0].fields == {'bed_id': 'A'}
    v.result(mc.IDENTIFY_ACTION, patient_id='P1')
    assert v.core.state == mc.MEASURE
    assert v.core.patient_id == 'P1' and v.core.identity_source == 'qr'


def test_qr_mode_wristband_of_another_bed_alerts():
    """A wristband that is not the patient assigned to this bed never starts a measurement."""
    v = Visit(identify_mode='qr')
    v.send(mc.Start('A'))
    v.result(mc.NAV)
    v.result(mc.IDENTIFY_ACTION, patient_id='P2')           # P2 belongs to bed B
    assert v.core.state == mc.ALERT
    assert 'P2' in v.core.alert_reason and 'bed A' in v.core.alert_reason
    assert 'expected P1' in v.core.alert_reason
    assert not [c for c in v.commands if isinstance(c, mc.MeasureCmd) and c.start]


def test_qr_mode_bed_without_an_assigned_patient_accepts_the_scanned_id():
    """Where no patient is assigned to the bed there is nothing to compare, the scan decides."""
    v = Visit(identify_mode='qr')
    v.send(mc.Start('Z'))
    v.result(mc.NAV)
    v.result(mc.IDENTIFY_ACTION, patient_id='P9')
    assert v.core.state == mc.MEASURE and v.core.patient_id == 'P9'


@pytest.mark.parametrize('kwargs', [
    {'ok': False, 'reason': 'no QR in view'},
    {'ok': True, 'patient_id': ''},
])
def test_qr_mode_failure_or_empty_id_alerts(kwargs):
    """No patient id from the scan means an alert, never a guess."""
    v = Visit(identify_mode='qr')
    v.send(mc.Start('A'))
    v.result(mc.NAV)
    v.result(mc.IDENTIFY_ACTION, **kwargs)
    assert v.core.state == mc.ALERT
    assert 'not identified' in v.core.alert_reason


def test_nurse_mode_with_no_assigned_patient_alerts():
    """A bed with nobody assigned cannot be visited in nurse mode."""
    v = Visit()
    v.send(mc.Start('Z'))
    v.result(mc.NAV)
    assert v.core.state == mc.ALERT
    assert 'no patient' in v.core.alert_reason


def test_skip_mode_reuses_the_previous_patient_on_a_repeat_visit():
    """Skip falls back to the nurse the first time, then reuses the earlier id."""
    v = Visit(identify_mode='skip')
    v.to_dispense()
    v.drawer_answer()
    v.send(mc.DrawerStatus(0b010, 0b010))
    v.send(mc.DrawerStatus(0, 0))
    v.result(mc.NAV)
    assert v.core.state == mc.IDLE
    first = v.core.identity_source
    assert first == 'nurse'

    v.send(mc.Start('A'))
    v.result(mc.NAV)
    assert v.core.identity_source == 'previous' and v.core.patient_id == 'P1'
    assert v.core.state == mc.MEASURE


def test_identity_is_never_left_unconfirmed():
    """Whatever the mode, the approval goal always says who confirmed the identity."""
    for mode in ('qr', 'nurse', 'skip'):
        v = Visit(identify_mode=mode)
        v.send(mc.Start('A'))
        v.result(mc.NAV)
        if mode == 'qr':
            v.result(mc.IDENTIFY_ACTION, patient_id='P1')
        v.measure_answer()
        v.result(mc.EVALUATE, outcome=mc.OUTCOME_DISPENSE, drawer_id=0, summary='')
        assert v.goals(mc.APPROVAL)[0].fields['identity_source'] in ('qr', 'nurse', 'previous')


# --------------------------------------------------------------------------
# Measuring and deciding
# --------------------------------------------------------------------------

def test_measure_start_refused_alerts():
    """If the ESP32 refuses to start measuring (a sensor fault), the visit alerts."""
    v = Visit()
    v.send(mc.Start('A'))
    v.result(mc.NAV)
    v.measure_answer(ok=False, reason='status 3')
    assert v.core.state == mc.ALERT
    assert 'would not start' in v.core.alert_reason
    assert not v.goals(mc.EVALUATE)


def test_hold_outcome_alerts_with_the_reason_and_stops_the_sensors():
    """Abnormal vitals: hold all drugs, sensors off, drawers locked, nurse told why."""
    v = Visit()
    v.to_decide()
    v.result(mc.EVALUATE, outcome=mc.OUTCOME_HOLD, reason='temperature 39.4 C')
    assert v.core.state == mc.ALERT
    assert 'hold all drugs' in v.core.alert_reason and '39.4' in v.core.alert_reason
    assert any(isinstance(c, mc.MeasureCmd) and not c.start for c in v.last)
    assert locked_all(v.last)
    assert v.goals(mc.ALERT_ACTION)[-1].fields['reason'] == v.core.alert_reason
    assert not v.goals(mc.APPROVAL)


def test_unreliable_once_measures_again_then_proceeds():
    """One unreliable result restarts the sensors (stop then start) and tries again."""
    v = Visit()
    v.to_decide()
    v.result(mc.EVALUATE, outcome=mc.OUTCOME_UNRELIABLE, reason='no stable readings')
    assert v.core.state == mc.MEASURE
    kinds = [c.start for c in v.last if isinstance(c, mc.MeasureCmd)]
    assert kinds == [False, True]
    v.measure_answer()
    assert v.core.state == mc.DECIDE
    v.result(mc.EVALUATE, outcome=mc.OUTCOME_DISPENSE, drawer_id=0, summary='')
    assert v.core.state == mc.WAIT_APPROVAL


def test_unreliable_twice_alerts():
    """After the allowed attempts the nurse is called."""
    v = Visit()
    v.to_decide()
    v.result(mc.EVALUATE, outcome=mc.OUTCOME_UNRELIABLE)
    v.measure_answer()
    v.result(mc.EVALUATE, outcome=mc.OUTCOME_UNRELIABLE)
    assert v.core.state == mc.ALERT
    assert 'unreliable' in v.core.alert_reason


def test_measure_stop_answer_is_ignored():
    """The answer to a STOP command must not be mistaken for the START answer."""
    v = Visit()
    v.to_decide()
    v.result(mc.EVALUATE, outcome=mc.OUTCOME_UNRELIABLE)
    stop = [c for c in v.last if isinstance(c, mc.MeasureCmd) and not c.start][0]
    assert v.send(mc.MeasureResult(stop.token, True)) == []
    assert v.core.state == mc.MEASURE


def test_evaluate_failure_alerts():
    """If decision_engine itself fails, the visit alerts."""
    v = Visit()
    v.to_decide()
    v.result(mc.EVALUATE, ok=False, reason='server died')
    assert v.core.state == mc.ALERT and 'decision failed' in v.core.alert_reason


# --------------------------------------------------------------------------
# Approval and dispensing
# --------------------------------------------------------------------------

def test_declined_approval_alerts_and_never_unlocks():
    """The nurse says no: alert, and no unlock command is ever sent."""
    v = Visit()
    v.to_decide()
    v.result(mc.EVALUATE, outcome=mc.OUTCOME_DISPENSE, drawer_id=1, summary='')
    v.result(mc.APPROVAL, ok=False, reason='declined')
    assert v.core.state == mc.ALERT
    assert not any(isinstance(c, mc.DrawerCmd) and c.unlock for c in v.commands)


def test_unlock_refused_alerts():
    """REFUSED_SAFETY from the ESP32 stops the dispense."""
    v = Visit()
    v.to_dispense()
    v.drawer_answer(ok=False, reason='status 3')
    assert v.core.state == mc.ALERT and 'refused' in v.core.alert_reason
    assert locked_all(v.last)


def test_drawer_that_never_opens_alerts_after_the_open_timeout():
    """The servo must report open within 5 s of the unlock answer."""
    v = Visit()
    v.to_dispense()
    v.drawer_answer()
    v.wait(4.9)
    assert v.core.state == mc.DISPENSE
    v.wait(0.2)
    assert v.core.state == mc.ALERT and 'did not open' in v.core.alert_reason
    assert locked_all(v.last)


def test_drawer_left_open_alerts_after_hold_plus_margin():
    """20 s hold plus 10 s margin, then the drawer is declared left open."""
    v = Visit()
    v.to_dispense()
    v.drawer_answer()
    v.send(mc.DrawerStatus(0b010, 0b010))
    v.wait(29.5)
    assert v.core.state == mc.DISPENSE
    v.wait(1.0)
    assert v.core.state == mc.ALERT and 'left open' in v.core.alert_reason


def test_unlock_with_no_answer_alerts():
    """If the unlock service never answers, give up after 10 s."""
    v = Visit()
    v.to_dispense()
    v.wait(10.5)
    assert v.core.state == mc.ALERT and 'no answer' in v.core.alert_reason


def test_status_may_arrive_before_the_unlock_answer():
    """Open status first, answer second: still counts, and the late answer changes nothing."""
    v = Visit()
    v.to_dispense()
    v.send(mc.DrawerStatus(0b010, 0b010))
    v.drawer_answer()
    v.send(mc.DrawerStatus(0, 0))
    assert v.core.state == mc.RETURN


def test_only_the_dispensed_drawer_is_watched():
    """Another drawer's switch does not count as this dose being given."""
    v = Visit()
    v.to_dispense(drawer=1)
    v.drawer_answer()
    v.send(mc.DrawerStatus(0b001, 0b001))        # drawer 0, not drawer 1
    assert v.core.state == mc.DISPENSE
    assert v.core._phase == 'opening'


def test_relock_time_comes_from_the_config():
    """drawer_hold_s is what the ESP32 is told to keep the drawer open for."""
    v = Visit(drawer_hold_s=15)
    v.to_dispense()
    assert v.last_of(mc.DrawerCmd)[0].relock_s == 15


# --------------------------------------------------------------------------
# Alert, going home, failure
# --------------------------------------------------------------------------

def test_nav_failure_alerts_then_goes_home_after_the_nurse_acknowledges():
    """The demo rule: alert, wait for the nurse, then always go home."""
    v = Visit()
    v.send(mc.Start('A'))
    v.result(mc.NAV, ok=False, reason='blocked')
    assert v.core.state == mc.ALERT
    assert locked_all(v.last)
    assert not [c for c in v.last if isinstance(c, mc.MeasureCmd)]    # sensors were never on
    v.wait(1000.0)
    assert v.core.state == mc.ALERT            # waits for the nurse as long as it takes
    v.result(mc.ALERT_ACTION)
    assert v.core.state == mc.GOING_HOME
    assert v.goals(mc.NAV)[-1].fields == {'target_id': 'dock'}
    v.result(mc.NAV)
    assert v.core.state == mc.IDLE and v.core.outcome == mc.ALERTED


def test_failed_alert_delivery_still_goes_home():
    """If nobody can be told, the robot still ends up safe at the dock."""
    v = Visit()
    v.send(mc.Start('A'))
    v.result(mc.NAV, ok=False)
    v.result(mc.ALERT_ACTION, ok=False, reason='server unavailable')
    assert v.core.state == mc.GOING_HOME


def test_going_home_failure_ends_failed_and_a_new_visit_is_still_possible():
    """FAILED is a dead end for the robot, not for the manager."""
    v = Visit()
    v.send(mc.Start('A'))
    v.result(mc.NAV, ok=False)
    v.result(mc.ALERT_ACTION)
    v.result(mc.NAV, ok=False, reason='lost')
    assert v.core.state == mc.FAILED and v.core.outcome == mc.FAILED_OUTCOME
    assert v.core.can_start()
    v.send(mc.Start('A'))
    assert v.core.state == mc.GO_TO_BED


def test_going_home_timeout_fails():
    """Driving home forever is a failure too."""
    v = Visit()
    v.send(mc.Start('A'))
    v.result(mc.NAV, ok=False)
    v.result(mc.ALERT_ACTION)
    v.wait(121.0)
    assert v.core.state == mc.FAILED
    assert any(isinstance(c, mc.CancelGoal) for c in v.last)


def test_every_failure_point_locks_all_drawers_and_stops_the_sensors():
    """From any step, an alert makes the robot safe in the same step."""
    steps = {
        'nav': lambda v: (v.send(mc.Start('A')), v.result(mc.NAV, ok=False)),
        'measure': lambda v: (v.send(mc.Start('A')), v.result(mc.NAV),
                              v.measure_answer(ok=False)),
        'decide': lambda v: (v.to_decide(), v.result(mc.EVALUATE, ok=False)),
        'approval': lambda v: (v.to_decide(),
                               v.result(mc.EVALUATE, outcome=0, drawer_id=1, summary=''),
                               v.result(mc.APPROVAL, ok=False)),
        'dispense': lambda v: (v.to_dispense(), v.drawer_answer(ok=False)),
    }
    for name, run in steps.items():
        v = Visit()
        run(v)
        assert v.core.state == mc.ALERT, name
        assert locked_all(v.last), name
        assert v.goals(mc.ALERT_ACTION), name


def test_no_unlock_is_ever_sent_before_approval():
    """The only way to an unlock command is through an approved approval goal."""
    v = Visit()
    v.to_decide()
    v.result(mc.EVALUATE, outcome=mc.OUTCOME_DISPENSE, drawer_id=1, summary='')
    assert not any(isinstance(c, mc.DrawerCmd) and c.unlock for c in v.commands)
    v.wait(301.0)                                   # approval times out
    assert v.core.state == mc.ALERT
    assert not any(isinstance(c, mc.DrawerCmd) and c.unlock for c in v.commands)


# --------------------------------------------------------------------------
# Timeouts, stale answers, busy
# --------------------------------------------------------------------------

@pytest.mark.parametrize('state,seconds,text', [
    (mc.GO_TO_BED, 121.0, 'driving to the bed'),
    (mc.DECIDE, 91.0, 'checking the vitals'),
    (mc.WAIT_APPROVAL, 301.0, 'waiting for approval'),
])
def test_step_timeouts_cancel_the_goal_and_alert(state, seconds, text):
    """A server that never answers cannot hang the visit."""
    v = Visit()
    if state == mc.GO_TO_BED:
        v.send(mc.Start('A'))
    elif state == mc.DECIDE:
        v.to_decide()
    else:
        v.to_decide()
        v.result(mc.EVALUATE, outcome=0, drawer_id=0, summary='')
    v.wait(seconds)
    assert v.core.state == mc.ALERT and text in v.core.alert_reason
    assert any(isinstance(c, mc.CancelGoal) for c in v.last)


def test_identify_timeout_alerts():
    """The QR scan has its own time limit."""
    v = Visit(identify_mode='qr')
    v.send(mc.Start('A'))
    v.result(mc.NAV)
    v.wait(61.0)
    assert v.core.state == mc.ALERT and 'identifying' in v.core.alert_reason


def test_sensors_that_never_answer_alert():
    """If the START command is never answered, the visit alerts after 10 s."""
    v = Visit()
    v.send(mc.Start('A'))
    v.result(mc.NAV)
    v.wait(10.5)
    assert v.core.state == mc.ALERT and 'did not start' in v.core.alert_reason


def test_a_result_for_a_cancelled_goal_is_ignored():
    """After a timeout cancelled the goal, its late answer must not move the state."""
    v = Visit()
    v.send(mc.Start('A'))
    nav_token = v.tokens[mc.NAV]
    v.wait(121.0)
    assert v.core.state == mc.ALERT
    assert v.send(mc.GoalResult(mc.NAV, nav_token, True)) == []
    assert v.core.state == mc.ALERT


def test_a_result_with_the_wrong_name_or_token_is_ignored():
    """Only the goal being waited for can move the visit forward."""
    v = Visit()
    v.send(mc.Start('A'))
    assert v.send(mc.GoalResult(mc.EVALUATE, v.tokens[mc.NAV], True)) == []
    assert v.send(mc.GoalResult(mc.NAV, v.tokens[mc.NAV] + 50, True)) == []
    assert v.core.state == mc.GO_TO_BED


def test_start_is_ignored_unless_idle():
    """A second start during a visit changes nothing."""
    v = Visit()
    v.send(mc.Start('A'))
    assert not v.core.can_start()
    out = v.send(mc.Start('B'))
    assert all(isinstance(c, mc.Log) for c in out)
    assert v.core.bed_id == 'A'


# --------------------------------------------------------------------------
# Safety events
# --------------------------------------------------------------------------

@pytest.mark.parametrize('stage', ['go', 'measure', 'decide', 'dispense', 'return'])
def test_estop_alerts_during_any_visit_step(stage):
    """An E-stop anywhere in a visit raises the alert."""
    v = Visit()
    if stage == 'go':
        v.send(mc.Start('A'))
    elif stage == 'measure':
        v.send(mc.Start('A'))
        v.result(mc.NAV)
    elif stage == 'decide':
        v.to_decide()
    elif stage == 'dispense':
        v.to_dispense()
    else:
        v.to_dispense()
        v.drawer_answer()
        v.send(mc.DrawerStatus(0b010, 0b010))
        v.send(mc.DrawerStatus(0, 0))
        assert v.core.state == mc.RETURN
    v.send(mc.EspStatus(estop=True))
    assert v.core.state == mc.ALERT and 'E-stop' in v.core.alert_reason


def test_estop_while_idle_or_alerting_is_ignored():
    """The flag is published at 2 Hz: it must not retrigger or start anything."""
    v = Visit()
    v.send(mc.EspStatus(estop=True))
    assert v.core.state == mc.IDLE
    v.send(mc.Start('A'))
    v.result(mc.NAV, ok=False)
    alerts = len(v.goals(mc.ALERT_ACTION))
    v.send(mc.EspStatus(estop=True))
    v.send(mc.EspStatus(estop=True))
    assert len(v.goals(mc.ALERT_ACTION)) == alerts


def test_vitals_fault_alerts_only_while_measuring_or_dispensing():
    """A vitals sensor fault matters from the moment the sensors are on."""
    v = Visit()
    v.send(mc.Start('A'))
    v.send(mc.EspStatus(vitals_fault=True))
    assert v.core.state == mc.GO_TO_BED
    v.result(mc.NAV)
    v.send(mc.EspStatus(vitals_fault=True))
    assert v.core.state == mc.ALERT and 'vitals sensor' in v.core.alert_reason


def test_link_lost_alerts_during_a_visit_only():
    """Losing the ESP32 link mid-visit is an alert; while idle it is just ignored."""
    v = Visit()
    v.send(mc.LinkLost())
    assert v.core.state == mc.IDLE
    v.send(mc.Start('A'))
    v.send(mc.LinkLost())
    assert v.core.state == mc.ALERT and 'link' in v.core.alert_reason


# --------------------------------------------------------------------------
# Config
# --------------------------------------------------------------------------

@pytest.mark.parametrize('kwargs', [
    {'identify_mode': 'face'}, {'drawer_hold_s': 0}, {'drawer_hold_s': 300},
    {'max_measure_attempts': 0}])
def test_bad_config_is_rejected(kwargs):
    """Settings that make no sense raise instead of misbehaving later."""
    with pytest.raises(ValueError):
        mc.MissionConfig(**kwargs)


def test_unknown_event_is_an_error():
    """A typo in the node wiring should be loud."""
    with pytest.raises(TypeError):
        mc.MissionCore().handle('nonsense', 0.0)


def test_alert_guard_protects_against_double_alerts():
    """Raising an alert while idle or already alerting does nothing but log."""
    v = Visit()
    out = v.core._alert('stray', 0.0)
    assert all(isinstance(c, mc.Log) for c in out) and v.core.state == mc.IDLE
    v.send(mc.Start('A'))
    v.result(mc.NAV, ok=False)
    sent = len(v.goals(mc.ALERT_ACTION))
    out = v.core._alert('again', 1.0)
    assert all(isinstance(c, mc.Log) for c in out)
    assert len(v.goals(mc.ALERT_ACTION)) == sent
