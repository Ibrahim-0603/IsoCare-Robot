"""ROS-free state machine of the mission_manager node.

One visit to one bed is a list of steps. The core receives events (a goal finished,
a service answered, a drawer status arrived, time passed) and returns commands for the
node to carry out. It never touches ROS and never reads a clock: the node passes `now`
in, so tests can run a whole visit in simulated time.

Happy path:
    GO_TO_BED -> IDENTIFY -> MEASURE -> DECIDE -> WAIT_APPROVAL -> DISPENSE -> RETURN -> IDLE
Any failure goes to ALERT (drawers locked, sensors off, navigation cancelled, nurse told,
wait for the nurse), then GOING_HOME, then IDLE. If even going home fails: FAILED.
"""

from dataclasses import dataclass, field

# States
IDLE = 'IDLE'
GO_TO_BED = 'GO_TO_BED'
IDENTIFY = 'IDENTIFY'
MEASURE = 'MEASURE'              # sensors starting
DECIDE = 'DECIDE'                # decision_engine collects stable readings and applies rules
WAIT_APPROVAL = 'WAIT_APPROVAL'
DISPENSE = 'DISPENSE'
RETURN = 'RETURN'
ALERT = 'ALERT'
GOING_HOME = 'GOING_HOME'
FAILED = 'FAILED'

# Action names (the node maps each to its action type and topic name)
NAV = 'nav'
IDENTIFY_ACTION = 'identify'
EVALUATE = 'evaluate'
APPROVAL = 'approval'
ALERT_ACTION = 'alert'

# Evaluate outcomes (same numbers as Evaluate.action)
OUTCOME_DISPENSE = 0
OUTCOME_HOLD = 1
OUTCOME_UNRELIABLE = 2

# How a visit ended
COMPLETED = 'completed'
ALERTED = 'alerted'
FAILED_OUTCOME = 'failed'

VISIT_STATES = (GO_TO_BED, IDENTIFY, MEASURE, DECIDE, WAIT_APPROVAL, DISPENSE, RETURN)
MEASURING_STATES = (MEASURE, DECIDE, WAIT_APPROVAL, DISPENSE)
ALL_DRAWERS = (0, 1, 2, 3)       # three drawers and the lid


@dataclass
class MissionConfig:
    """Tunable settings. The node fills these from ROS parameters."""

    identify_mode: str = 'nurse'            # 'qr', 'nurse' or 'skip' (repeat visit)
    bed_patients: dict = field(default_factory=dict)    # bed id -> assigned patient id
    dock_id: str = 'dock'
    sensor_mask: int = 7                    # MAX30102 + MLX90614 + MAX30205
    max_measure_attempts: int = 2           # first measurement plus re-measures
    drawer_hold_s: int = 20                 # how long the open drawer stays open
    drawer_unlock_timeout_s: float = 10.0   # the /drawer/unlock service must answer by then
    drawer_open_timeout_s: float = 5.0      # then the drawer must report open
    drawer_close_margin_s: float = 10.0     # extra time after the hold before "left open"
    measure_start_timeout_s: float = 10.0
    nav_timeout_s: float = 120.0
    identify_timeout_s: float = 60.0
    decide_timeout_s: float = 90.0
    approval_timeout_s: float = 300.0

    def __post_init__(self):
        """Reject settings that make no sense."""
        if self.identify_mode not in ('qr', 'nurse', 'skip'):
            raise ValueError("identify_mode must be 'qr', 'nurse' or 'skip'")
        if not 1 <= self.drawer_hold_s <= 255:
            raise ValueError('drawer_hold_s must be 1 to 255 (it is sent as one byte)')
        if self.max_measure_attempts < 1:
            raise ValueError('max_measure_attempts must be at least 1')


# ----------------------------------------------------------------------
# Events: what the node tells the core
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class Start:
    """Begin a visit to bed_id."""

    bed_id: str


@dataclass(frozen=True)
class GoalResult:
    """An action finished. ok means it succeeded (approved, acknowledged, ...).

    data holds extra fields: patient_id for identify; outcome, drawer_id and summary
    for evaluate. token is the one from the SendGoal command it answers.
    """

    name: str
    token: int
    ok: bool
    reason: str = ''
    data: dict = field(default_factory=dict)


@dataclass(frozen=True)
class MeasureResult:
    """The /vitals/measure service answered the command with this token."""

    token: int
    ok: bool
    reason: str = ''


@dataclass(frozen=True)
class DrawerResult:
    """The /drawer/unlock service answered the command with this token."""

    token: int
    ok: bool
    reason: str = ''


@dataclass(frozen=True)
class DrawerStatus:
    """A /drawer/status message: bit i of each mask is drawer i (bit 3 = lid)."""

    unlocked_mask: int
    open_mask: int


@dataclass(frozen=True)
class EspStatus:
    """The safety flags the manager cares about from /esp_status."""

    estop: bool = False
    vitals_fault: bool = False


@dataclass(frozen=True)
class LinkLost:
    """The serial link to the ESP32 is down (/link_stats link_ok went false)."""


# ----------------------------------------------------------------------
# Commands: what the core asks the node to do
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class SendGoal:
    """Send an action goal. name is one of the action names above."""

    name: str
    token: int
    fields: dict


@dataclass(frozen=True)
class CancelGoal:
    """Cancel the goal that was sent with this token."""

    name: str
    token: int


@dataclass(frozen=True)
class MeasureCmd:
    """Call /vitals/measure."""

    token: int
    start: bool
    sensor_mask: int = 0


@dataclass(frozen=True)
class DrawerCmd:
    """Call /drawer/unlock (unlock=False means lock)."""

    token: int
    drawer_id: int
    unlock: bool
    relock_s: int = 0


@dataclass(frozen=True)
class Log:
    """A line for the node's log."""

    text: str


class MissionCore:
    """The visit state machine. Feed it events with handle() and time with tick()."""

    def __init__(self, config=None):
        """Start idle."""
        self.config = config if config is not None else MissionConfig()
        self.state = IDLE
        self.outcome = ''                   # how the last visit ended
        self.alert_reason = ''
        self.bed_id = ''
        self.patient_id = ''
        self.identity_source = ''
        self.drawer_id = 0
        self.summary = ''
        self.last_patient_by_bed = {}       # for identify_mode 'skip'
        self._token = 0
        self._active = None                 # (name, token) of the goal being waited for
        self._deadline = None
        self._measuring = False
        self._measure_token = None          # token of the START we wait for
        self._attempts = 0
        self._phase = ''                    # DISPENSE: 'unlock', 'opening' or 'open'
        self._drawer_token = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def can_start(self):
        """Return True if a new visit may begin."""
        return self.state in (IDLE, FAILED)

    def handle(self, event, now):
        """Apply one event and return the list of commands to carry out."""
        if isinstance(event, Start):
            return self._on_start(event, now)
        if isinstance(event, GoalResult):
            return self._on_goal_result(event, now)
        if isinstance(event, MeasureResult):
            return self._on_measure_result(event, now)
        if isinstance(event, DrawerResult):
            return self._on_drawer_result(event, now)
        if isinstance(event, DrawerStatus):
            return self._on_drawer_status(event, now)
        if isinstance(event, EspStatus):
            return self._on_esp_status(event, now)
        if isinstance(event, LinkLost):
            return self._alert_if_visiting('serial link to the ESP32 lost', now)
        raise TypeError(f'unknown event {event!r}')

    def tick(self, now):
        """Check the deadline of the current step. Call it a few times per second."""
        if self._deadline is None or now < self._deadline:
            return []
        self._deadline = None
        if self.state in (GO_TO_BED, RETURN, IDENTIFY, DECIDE, WAIT_APPROVAL):
            step = {GO_TO_BED: 'driving to the bed', RETURN: 'driving back to the dock',
                    IDENTIFY: 'identifying the patient', DECIDE: 'checking the vitals',
                    WAIT_APPROVAL: 'waiting for approval'}[self.state]
            return self._alert(f'timed out {step}', now)
        if self.state == MEASURE:
            return self._alert('sensors did not start measuring', now)
        if self.state == DISPENSE:
            reason = {'unlock': 'drawer unlock got no answer',
                      'opening': 'drawer did not open',
                      'open': 'drawer left open'}[self._phase]
            return self._alert(reason, now)
        if self.state == GOING_HOME:
            return self._give_up('timed out driving home')
        return []

    # ------------------------------------------------------------------
    # Starting and the happy path
    # ------------------------------------------------------------------

    def _on_start(self, event, now):
        """Begin a visit if idle; otherwise ignore."""
        if not self.can_start():
            return [Log(f'start for bed {event.bed_id} ignored: state is {self.state}')]
        self.bed_id = event.bed_id
        self.patient_id = ''
        self.identity_source = ''
        self.summary = ''
        self.alert_reason = ''
        self.outcome = ''
        self._attempts = 0
        self.state = GO_TO_BED
        return [Log(f'visit to bed {self.bed_id} started'),
                self._goal(NAV, {'target_id': self.bed_id}, self.config.nav_timeout_s, now)]

    def _begin_identify(self, now):
        """Pick the identity source for this bed (see identify_mode)."""
        mode = self.config.identify_mode
        if mode == 'qr':
            self.state = IDENTIFY
            return [self._goal(IDENTIFY_ACTION, {'bed_id': self.bed_id},
                               self.config.identify_timeout_s, now)]
        if mode == 'skip' and self.bed_id in self.last_patient_by_bed:
            self.patient_id = self.last_patient_by_bed[self.bed_id]
            self.identity_source = 'previous'
            return self._begin_measure(now)
        patient = self.config.bed_patients.get(self.bed_id)
        if not patient:
            return self._alert(f'no patient is assigned to bed {self.bed_id}', now)
        self.patient_id = patient
        self.identity_source = 'nurse'
        return self._begin_measure(now)

    def _begin_measure(self, now):
        """Start the sensors (first time or a re-measure)."""
        self._attempts += 1
        self.state = MEASURE
        self._measuring = True
        token = self._next_token()
        self._measure_token = token
        self._deadline = now + self.config.measure_start_timeout_s
        return [Log(f'measuring, attempt {self._attempts}'),
                MeasureCmd(token, True, self.config.sensor_mask)]

    def _on_goal_result(self, event, now):
        """Route an action result to the step that is waiting for it."""
        if self._active != (event.name, event.token):
            return []                                   # stale: cancelled or from an old visit
        self._active = None
        self._deadline = None
        if self.state == GO_TO_BED and event.name == NAV:
            if not event.ok:
                return self._alert(f'could not reach bed {self.bed_id}: {event.reason}', now)
            return self._begin_identify(now)
        if self.state == IDENTIFY and event.name == IDENTIFY_ACTION:
            patient = event.data.get('patient_id', '')
            if not event.ok or not patient:
                return self._alert(f'patient not identified: {event.reason}', now)
            self.patient_id = patient
            self.identity_source = 'qr'
            return self._begin_measure(now)
        if self.state == DECIDE and event.name == EVALUATE:
            return self._on_evaluate(event, now)
        if self.state == WAIT_APPROVAL and event.name == APPROVAL:
            if not event.ok:
                return self._alert(f'not approved: {event.reason or "declined"}', now)
            return self._begin_dispense(now)
        if self.state == RETURN and event.name == NAV:
            if not event.ok:
                return self._alert(f'could not drive back to the dock: {event.reason}', now)
            self.last_patient_by_bed[self.bed_id] = self.patient_id
            return self._finish(COMPLETED, f'visit to bed {self.bed_id} completed')
        if self.state == ALERT and event.name == ALERT_ACTION:
            note = '' if event.ok else ' (the alert was not delivered)'
            self.state = GOING_HOME
            return [Log(f'alert acknowledged{note}, going home'),
                    self._goal(NAV, {'target_id': self.config.dock_id},
                               self.config.nav_timeout_s, now)]
        if self.state == GOING_HOME and event.name == NAV:
            if not event.ok:
                return self._give_up(f'could not drive home: {event.reason}')
            return self._finish(ALERTED, 'back at the dock after an alert')
        return []

    def _on_measure_result(self, event, now):
        """Handle the START answer: go on to the decision, or alert."""
        if self.state != MEASURE or event.token != self._measure_token:
            return []                                   # a STOP answer or a stale one
        self._deadline = None
        if not event.ok:
            return self._alert(f'sensors would not start: {event.reason}', now)
        self.state = DECIDE
        return [self._goal(EVALUATE, {'patient_id': self.patient_id, 'bed_id': self.bed_id},
                           self.config.decide_timeout_s, now)]

    def _on_evaluate(self, event, now):
        """decision_engine finished: dispense, hold, or re-measure."""
        if not event.ok:
            return self._alert(f'decision failed: {event.reason}', now)
        outcome = event.data.get('outcome')
        if outcome == OUTCOME_DISPENSE:
            self.drawer_id = int(event.data.get('drawer_id', 0))
            self.summary = str(event.data.get('summary', ''))
            commands = self._stop_measuring()
            self.state = WAIT_APPROVAL
            commands.append(self._goal(
                APPROVAL, {'patient_id': self.patient_id, 'bed_id': self.bed_id,
                           'drawer_id': self.drawer_id,
                           'identity_source': self.identity_source, 'summary': self.summary},
                self.config.approval_timeout_s, now))
            return commands
        if outcome == OUTCOME_UNRELIABLE and self._attempts < self.config.max_measure_attempts:
            return [Log('readings unreliable, measuring again')] + self._restart_measuring(now)
        if outcome == OUTCOME_UNRELIABLE:
            return self._alert('readings unreliable after re-measuring', now)
        return self._alert(f'hold all drugs: {event.reason or "abnormal vitals"}', now)

    def _restart_measuring(self, now):
        """STOP then START, so the sensors begin a fresh warm-up."""
        stop = MeasureCmd(self._next_token(), False)
        return [stop] + self._begin_measure(now)

    # ------------------------------------------------------------------
    # Dispensing
    # ------------------------------------------------------------------

    def _begin_dispense(self, now):
        """Unlock the drawer: its servo opens it and closes it again after drawer_hold_s."""
        self.state = DISPENSE
        self._phase = 'unlock'
        self._drawer_token = self._next_token()
        self._deadline = now + self.config.drawer_unlock_timeout_s
        return [Log(f'unlocking drawer {self.drawer_id}'),
                DrawerCmd(self._drawer_token, self.drawer_id, True, self.config.drawer_hold_s)]

    def _on_drawer_result(self, event, now):
        """Handle the unlock answer (lock answers are not tracked)."""
        if self.state != DISPENSE or event.token != self._drawer_token:
            return []
        if not event.ok:
            return self._alert(f'drawer {self.drawer_id} unlock refused: {event.reason}', now)
        if self._phase == 'unlock':
            self._phase = 'opening'
            self._deadline = now + self.config.drawer_open_timeout_s
        return []

    def _on_drawer_status(self, event, now):
        """Watch the limit switch: open, then closed again, means the dose was given."""
        if self.state != DISPENSE:
            return []
        is_open = bool(event.open_mask & (1 << self.drawer_id))
        if self._phase in ('unlock', 'opening') and is_open:
            self._phase = 'open'
            self._deadline = (now + self.config.drawer_hold_s
                              + self.config.drawer_close_margin_s)
            return [Log(f'drawer {self.drawer_id} is open')]
        if self._phase == 'open' and not is_open:
            self._deadline = None
            self.state = RETURN
            return [Log(f'drawer {self.drawer_id} closed again: dispensed'),
                    self._goal(NAV, {'target_id': self.config.dock_id},
                               self.config.nav_timeout_s, now)]
        return []

    # ------------------------------------------------------------------
    # Safety events
    # ------------------------------------------------------------------

    def _on_esp_status(self, event, now):
        """Alert on an E-stop during a visit, or on a vitals sensor fault while measuring."""
        if event.estop:
            return self._alert_if_visiting('E-stop pressed', now)
        if event.vitals_fault and self.state in MEASURING_STATES:
            return self._alert('a vitals sensor stopped responding', now)
        return []

    def _alert_if_visiting(self, reason, now):
        """Alert only while a visit is in progress (not idle, not already alerting)."""
        if self.state in VISIT_STATES:
            return self._alert(reason, now)
        return []

    # ------------------------------------------------------------------
    # Alert, going home, finishing
    # ------------------------------------------------------------------

    def _alert(self, reason, now):
        """Make everything safe, tell the nurse and wait for them."""
        if self.state not in VISIT_STATES:
            return [Log(f'alert ignored in {self.state}: {reason}')]
        commands = [Log(f'ALERT: {reason}')]
        commands += self._cancel_active()
        commands += self._stop_measuring()
        for drawer in ALL_DRAWERS:
            commands.append(DrawerCmd(self._next_token(), drawer, False))
        self.alert_reason = reason
        self.state = ALERT
        self._phase = ''
        self._deadline = None                       # waits for the nurse as long as it takes
        commands.append(self._goal(ALERT_ACTION, {'bed_id': self.bed_id, 'reason': reason},
                                   None, now))
        return commands

    def _give_up(self, reason):
        """Nothing more the robot can do on its own."""
        commands = self._cancel_active()
        self.state = FAILED
        self.outcome = FAILED_OUTCOME
        self._deadline = None
        return commands + [Log(f'FAILED: {reason}')]

    def _finish(self, outcome, text):
        """End the visit and go idle."""
        self.state = IDLE
        self.outcome = outcome
        self._deadline = None
        return [Log(text)]

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _next_token(self):
        """Return a new token (every goal and command gets its own)."""
        self._token += 1
        return self._token

    def _goal(self, name, fields, timeout_s, now):
        """Make a SendGoal command and remember it as the goal being waited for."""
        token = self._next_token()
        self._active = (name, token)
        self._deadline = None if timeout_s is None else now + timeout_s
        return SendGoal(name, token, fields)

    def _cancel_active(self):
        """Cancel the goal being waited for, if any."""
        if self._active is None:
            return []
        name, token = self._active
        self._active = None
        return [CancelGoal(name, token)]

    def _stop_measuring(self):
        """Switch the sensors off if they are on."""
        if not self._measuring:
            return []
        self._measuring = False
        self._measure_token = None
        return [MeasureCmd(self._next_token(), False)]
