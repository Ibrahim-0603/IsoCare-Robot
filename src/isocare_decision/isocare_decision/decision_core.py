"""ROS-free decision logic of the decision_engine node.

One evaluation (one goal) collects vitals samples, decides per vital when it is stable, checks
the abnormal bands and then applies the doctor's drug rules. The core never touches ROS and
never reads a clock: the node passes `now` (seconds, any steady clock) and `wall` (epoch seconds,
only used for dose intervals) in, so tests can run a whole evaluation in simulated time.

Order of the checks (the first one that triggers decides):
    1. empty patient id                      -> failure (success=False)
    2. a stable vital in its danger band     -> HOLD (decided as soon as that vital is stable)
    3. at the deadline HR or SpO2 not stable -> UNRELIABLE
    4. the drug rules, in file order; the first that passes -> DISPENSE with its drawer
    5. no rule passes                        -> HOLD (the reason says why, e.g. no drug indicated)
Note-band values and skipped vitals never change the outcome, they only show in the summary.
"""

from collections import deque
from dataclasses import dataclass, field

# Evaluate outcomes (same numbers as Evaluate.action)
OUTCOME_DISPENSE = 0
OUTCOME_HOLD = 1
OUTCOME_UNRELIABLE = 2

# The vitals. HR and SpO2 are mandatory: they gate the hold-on-abnormal check.
HR = 'hr'
SPO2 = 'spo2'
TEMP = 'temperature_c'
VITALS = (HR, SPO2, TEMP)
MANDATORY = (HR, SPO2)
LABELS = {HR: 'HR', SPO2: 'SpO2', TEMP: 'temperature'}

# Mission states that matter for recording a dose (same strings as mission_core.py)
STATE_DISPENSE = 'DISPENSE'
STATE_RETURN = 'RETURN'
STATES_THAT_DROP_A_DOSE = ('ALERT', 'GOING_HOME', 'FAILED', 'IDLE', 'GO_TO_BED')


@dataclass
class Sample:
    """One /vitals message, already converted to plain numbers."""

    hr_valid: bool = False
    spo2_valid: bool = False
    ir_valid: bool = False
    contact_valid: bool = False
    finger: bool = False
    hr: float = 0.0
    spo2: float = 0.0
    ir_c: float = 0.0
    contact_c: float = 0.0


@dataclass
class Bands:
    """Abnormal limits of one vital. None means no limit on that side."""

    danger_low: float = None
    note_low: float = None
    note_high: float = None
    danger_high: float = None

    def classify(self, value):
        """Return ('danger'|'note'|'normal', 'low'|'high'|'')."""
        if self.danger_low is not None and value < self.danger_low:
            return 'danger', 'low'
        if self.danger_high is not None and value > self.danger_high:
            return 'danger', 'high'
        if self.note_low is not None and value < self.note_low:
            return 'note', 'low'
        if self.note_high is not None and value > self.note_high:
            return 'note', 'high'
        return 'normal', ''


@dataclass
class Rule:
    """One drug rule (see rules.yaml)."""

    drug: str
    drawer_id: int
    when: dict                              # vital -> {'above'|'below'|'at_least'|'at_most': x}
    min_interval_h: float = 0.0
    blocked_by_allergy: tuple = ()

    @property
    def needs(self):
        """The vitals this rule depends on."""
        return tuple(self.when)


@dataclass
class DecisionConfig:
    """Tunable settings. The node fills these from ROS parameters."""

    settle_s: float = 5.0           # ignore the samples of the first seconds
    window: int = 5                 # stable readings needed in a row
    deadline_s: float = 80.0        # give up here (the manager gives up at 90 s)
    tolerance: dict = field(default_factory=lambda: {HR: 5.0, SPO2: 2.0, TEMP: 0.3})


@dataclass
class Result:
    """What the action returns."""

    success: bool
    outcome: int = OUTCOME_HOLD
    drawer_id: int = 0
    summary: str = ''
    reason: str = ''


class VitalWindow:
    """The last `size` valid readings of one vital, in a row. Frozen once it is stable."""

    def __init__(self, size, tolerance):
        """Start empty."""
        self.size, self.tolerance = size, tolerance
        self.readings = deque(maxlen=size)
        self.source = None
        self.value = None               # the average, set once and then kept
        self.seen = False               # True once any valid reading arrived

    @property
    def stable(self):
        """True once a stable average has been found."""
        return self.value is not None

    @property
    def count(self):
        """How many readings are in the window (the full size once stable)."""
        return self.size if self.stable else len(self.readings)

    def add(self, reading, source=None):
        """Add a valid reading, or None for an invalid one (which breaks the row)."""
        if self.stable:
            return
        if reading is None or (self.source is not None and source != self.source):
            self.readings.clear()
        if reading is None:
            self.source = None
            return
        self.source = source
        self.seen = True
        self.readings.append(float(reading))
        if (len(self.readings) == self.size
                and max(self.readings) - min(self.readings) <= self.tolerance):
            self.value = sum(self.readings) / self.size


class DecisionCore:
    """Collects vitals, judges them and applies the drug rules."""

    def __init__(self, config, rules, bands, patients, history):
        """Keep the settings. `history` needs last(patient, drug) and record(patient, drug, t)."""
        self.config, self.rules, self.bands = config, rules, bands
        self.patients, self.history = patients, history
        self.active = False
        self.pending = None             # (patient_id, drug) waiting for RETURN after DISPENSE
        self._prev_state = ''
        self._chosen = None
        self.patient_id = self.bed_id = ''
        self.t0 = 0.0
        self.windows = {}

    # -- one evaluation ---------------------------------------------------------------------
    def start(self, patient_id, bed_id, now):
        """Begin an evaluation. Returns a Result if it cannot even start, else None."""
        self.pending = None
        self.active = False
        if not patient_id:
            return Result(False, reason='no patient id given')
        self.patient_id, self.bed_id, self.t0 = patient_id, bed_id, now
        self.windows = {v: VitalWindow(self.config.window, self.config.tolerance[v])
                        for v in VITALS}
        self.active = True
        return None

    def add_sample(self, sample, now):
        """Feed one /vitals sample. Ignored during the settle time or when not evaluating."""
        if not self.active or now - self.t0 < self.config.settle_s:
            return
        self.windows[HR].add(sample.hr if sample.hr_valid and sample.finger else None)
        self.windows[SPO2].add(sample.spo2 if sample.spo2_valid and sample.finger else None)
        if sample.ir_valid:
            self.windows[TEMP].add(sample.ir_c, 'ir')
        elif sample.contact_valid:
            self.windows[TEMP].add(sample.contact_c, 'contact')
        else:
            self.windows[TEMP].add(None)

    def feedback(self, now):
        """Return (status text, smallest window count of the mandatory vitals)."""
        if now - self.t0 < self.config.settle_s:
            return f'settling {max(0.0, self.config.settle_s - (now - self.t0)):.0f} s', 0
        counts = {v: self.windows[v].count for v in VITALS}
        text = ', '.join(f'{LABELS[v]} {counts[v]} of {self.config.window}' for v in VITALS)
        return 'stable readings: ' + text, min(counts[HR], counts[SPO2])

    def poll(self, now, wall):
        """Return the Result when the evaluation is finished, else None (keep waiting)."""
        if not self.active:
            return None
        values = {v: self.windows[v].value for v in VITALS}
        for vital in VITALS:                            # a stable dangerous vital ends it now
            if values[vital] is not None:
                level, side = self.bands[vital].classify(values[vital])
                if level == 'danger':
                    return self._finish(self._danger(vital, values, side))
        timed_out = now - self.t0 >= self.config.deadline_s
        if not timed_out and any(values[v] is None for v in VITALS):
            return None
        missing = [v for v in MANDATORY if values[v] is None]
        if missing:
            names = ' and '.join(LABELS[v] for v in missing)
            return self._finish(Result(True, OUTCOME_UNRELIABLE, 0, self._summary(values),
                                       f'{names} not stable in time'))
        return self._finish(self._apply_rules(values, wall))

    def cancel(self):
        """Stop an evaluation (the manager cancelled the goal)."""
        self.active = False

    def _finish(self, result):
        """End the evaluation. A DISPENSE result waits for the visit to reach RETURN."""
        self.active = False
        if result.success and result.outcome == OUTCOME_DISPENSE:
            self.pending = (self.patient_id, self._chosen)
        return result

    # -- judging ----------------------------------------------------------------------------
    def _danger(self, vital, values, side):
        """HOLD result for a vital in its danger band."""
        bands = self.bands[vital]
        word, limit = ('below', bands.danger_low) if side == 'low' else ('above',
                                                                         bands.danger_high)
        reason = (f'{LABELS[vital]} {self._fmt(vital, values[vital])} is {word} the danger '
                  f'limit {self._fmt(vital, limit)}: hold all drugs')
        return Result(True, OUTCOME_HOLD, 0, self._summary(values), reason)

    def _apply_rules(self, values, wall):
        """Go through the rules in order; the first that passes gives DISPENSE."""
        allergies = self.patients.get(self.patient_id, {}).get('allergies')
        refusals = []
        for rule in self.rules:
            problem = self._check_rule(rule, values, allergies, wall)
            if problem is None:
                self._chosen = rule.drug
                summary = (f'{self._summary(values)} | rule: {rule.drug} '
                           f'({self._condition_text(rule)})')
                if allergies is None:
                    summary += ' | allergies not on record, please confirm'
                return Result(True, OUTCOME_DISPENSE, int(rule.drawer_id), summary, '')
            refusals.append(f'{rule.drug}: {problem}')
        reason = 'no drug indicated' if refusals else 'no drug rules configured'
        if refusals:
            reason += ' (' + '; '.join(refusals) + ')'
        return Result(True, OUTCOME_HOLD, 0, self._summary(values), reason)

    def _check_rule(self, rule, values, allergies, wall):
        """Return None if the rule passes, else the short reason it does not."""
        for vital in rule.needs:
            if values[vital] is None:
                return self._missing(vital)
        for vital, test in rule.when.items():
            for op, limit in test.items():
                if not _compare(op, values[vital], limit):
                    return (f'{LABELS[vital]} {self._fmt(vital, values[vital])} is not '
                            f'{op.replace("_", " ")} {limit}')
        if rule.min_interval_h > 0:
            last = self.history.last(self.patient_id, rule.drug)
            if last is not None:
                hours = (wall - last) / 3600.0
                if hours < rule.min_interval_h:
                    return f'given {hours:.1f} h ago, minimum {rule.min_interval_h:g} h'
        if allergies is not None:
            for name in rule.blocked_by_allergy:
                if name in allergies:
                    return f'patient is allergic ({name})'
        return None

    def _summary(self, values):
        """One line for the nurse, with note-band values marked and skipped vitals named."""
        parts = []
        for vital in VITALS:
            if values[vital] is None:
                parts.append(self._missing(vital))
                continue
            text = f'{LABELS[vital]} {self._fmt(vital, values[vital])}'
            level, side = self.bands[vital].classify(values[vital])
            if level != 'normal':
                text += f' ({side}, {level})'
            parts.append(text)
        return ', '.join(parts)

    def _missing(self, vital):
        """Say why a vital has no value: no valid reading at all, or readings that never agreed."""
        window = self.windows.get(vital)
        state = 'not stable' if window is not None and window.seen else 'not measured'
        return f'{LABELS[vital]} {state}'

    @staticmethod
    def _fmt(vital, value):
        """Format a vital: HR and SpO2 as whole numbers, temperature with one decimal."""
        return f'{value:.1f} C' if vital == TEMP else f'{value:.0f}'

    @staticmethod
    def _condition_text(rule):
        """Short text of a rule condition for the summary."""
        return ', '.join(f'{LABELS[vital]} {op.replace("_", " ")} {limit}'
                         for vital, test in rule.when.items() for op, limit in test.items())

    # -- recording doses --------------------------------------------------------------------
    def on_mission_state(self, state, wall):
        """Watch the visit: a dose counts only when RETURN follows DISPENSE."""
        if self.pending is not None:
            if self._prev_state == STATE_DISPENSE and state == STATE_RETURN:
                patient, drug = self.pending
                self.history.record(patient, drug, wall)
                self.pending = None
            elif state in STATES_THAT_DROP_A_DOSE:
                self.pending = None
        self._prev_state = state


def _compare(op, value, limit):
    """One comparison of a rule condition."""
    return {'above': value > limit, 'below': value < limit,
            'at_least': value >= limit, 'at_most': value <= limit}[op]
