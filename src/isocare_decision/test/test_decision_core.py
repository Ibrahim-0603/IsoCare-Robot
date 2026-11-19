"""Tests for isocare_decision (core, config loader, dose history). Plain pytest, no ROS."""

from pathlib import Path
import sys
import textwrap

try:
    from isocare_decision import decision_config as cfg, decision_core as dc
    from isocare_decision.dose_history import DoseHistory
except ImportError:  # running from the repo without sourcing the workspace
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isocare_decision import decision_config as cfg, decision_core as dc
    from isocare_decision.dose_history import DoseHistory
import pytest

CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
HOUR = 3600.0


def good(hr=72, spo2=98, ir=36.6, **over):
    """Return a sample where everything is valid."""
    values = {'hr_valid': True, 'spo2_valid': True, 'ir_valid': True, 'finger': True,
              'hr': hr, 'spo2': spo2, 'ir_c': ir}
    values.update(over)
    return dc.Sample(**values)


class Run:
    """Drive a DecisionCore by hand in simulated time (one sample per second)."""

    def __init__(self, patients=None, rules=None, history=None, **config):
        """Create a core with the shipped placeholder rules and bands."""
        self.history = history or DoseHistory(':memory:')
        self.core = dc.DecisionCore(
            dc.DecisionConfig(**config),
            rules if rules is not None else cfg.load_rules(CONFIG_DIR / 'rules.yaml'),
            cfg.load_bands(CONFIG_DIR / 'bands.yaml'),
            {'P1': {'allergies': []}} if patients is None else patients, self.history)
        self.now = 100.0
        self.wall = 1_000_000.0

    def begin(self, patient='P1'):
        """Start an evaluation."""
        return self.core.start(patient, 'A', self.now)

    def feed(self, make, seconds, start_after_settle=True):
        """Send `seconds` one-per-second samples made by make(i); stop early on a result."""
        if start_after_settle and self.now - self.core.t0 < self.core.config.settle_s:
            self.now = self.core.t0 + self.core.config.settle_s
        for i in range(seconds):
            self.core.add_sample(make(i), self.now)
            result = self.core.poll(self.now, self.wall)
            self.now += 1.0
            if result is not None:
                return result
        return None

    def until_deadline(self):
        """Jump to the deadline and poll."""
        self.now = self.core.t0 + self.core.config.deadline_s
        return self.core.poll(self.now, self.wall)


# --- stability windows --------------------------------------------------------------------
def test_five_close_readings_make_a_vital_stable_and_average():
    """Five close readings make a vital stable and average."""
    window = dc.VitalWindow(5, 5.0)
    for value in (70, 72, 74, 71, 73):
        window.add(value)
    assert window.stable and window.value == pytest.approx(72.0)


def test_spread_just_over_tolerance_is_not_stable_and_the_window_slides():
    """Spread just over tolerance is not stable and the window slides."""
    window = dc.VitalWindow(5, 5.0)
    for value in (70, 72, 74, 71, 76):          # spread 6 > 5
        window.add(value)
    assert not window.stable
    window.add(74)                              # window is now 72, 74, 71, 76, 74: spread 5
    assert window.stable


def test_an_invalid_reading_breaks_the_row():
    """An invalid reading breaks the row."""
    window = dc.VitalWindow(5, 5.0)
    for value in (70, 70, 70, 70):
        window.add(value)
    window.add(None)
    window.add(70)
    assert not window.stable and window.count == 1


def test_a_stable_window_is_frozen():
    """A stable window is frozen."""
    window = dc.VitalWindow(5, 5.0)
    for value in (70, 70, 70, 70, 70):
        window.add(value)
    window.add(200)
    assert window.value == 70


def test_temperature_source_switch_restarts_the_window():
    """Temperature source switch restarts the window."""
    window = dc.VitalWindow(5, 0.3)
    for _ in range(4):
        window.add(36.5, 'ir')
    window.add(36.5, 'contact')
    assert window.count == 1 and not window.stable


# --- the evaluation -----------------------------------------------------------------------
def test_empty_patient_id_fails_the_evaluation():
    """Empty patient id fails the evaluation."""
    result = Run().begin(patient='')
    assert result is not None and not result.success and 'patient' in result.reason


def test_samples_during_the_settle_time_are_ignored():
    """Samples during the settle time are ignored."""
    run = Run()
    run.begin()
    for _ in range(10):
        run.core.add_sample(good(), run.core.t0 + 4.9)
    assert all(run.core.windows[v].count == 0 for v in dc.VITALS)


def test_normal_vitals_and_no_fever_hold_with_no_drug_indicated():
    """Normal vitals and no fever hold with no drug indicated."""
    run = Run()
    run.begin()
    result = run.feed(lambda i: good(), 20)
    assert result.success and result.outcome == dc.OUTCOME_HOLD
    assert 'no drug indicated' in result.reason and 'paracetamol' in result.reason
    assert 'HR 72' in result.summary and 'SpO2 98' in result.summary


def test_fever_dispenses_paracetamol_from_drawer_1():
    """Fever dispenses paracetamol from drawer 1."""
    run = Run()
    run.begin()
    result = run.feed(lambda i: good(ir=38.6), 20)
    assert result.success and result.outcome == dc.OUTCOME_DISPENSE and result.drawer_id == 1
    assert '38.6 C (high, note)' in result.summary and 'rule: paracetamol' in result.summary


def test_the_decision_waits_until_all_three_vitals_are_stable():
    """The decision waits until all three vitals are stable."""
    run = Run()
    run.begin()
    assert run.feed(lambda i: good(ir_valid=False), 10) is None     # temperature never valid
    assert run.core.windows[dc.TEMP].count == 0


def test_a_stable_danger_value_holds_at_once_with_the_reason():
    """A stable danger value holds at once with the reason."""
    run = Run()
    run.begin()
    result = run.feed(lambda i: good(spo2=86, ir_valid=False), 10)
    assert result.outcome == dc.OUTCOME_HOLD
    assert 'SpO2 86' in result.reason and 'below' in result.reason
    assert run.now - run.core.t0 < 15           # did not wait for the deadline


def test_danger_high_temperature_holds_all_drugs_even_though_a_rule_would_fire():
    """Danger high temperature holds all drugs even though a rule would fire."""
    run = Run()
    run.begin()
    result = run.feed(lambda i: good(ir=40.1), 20)
    assert result.outcome == dc.OUTCOME_HOLD and 'temperature' in result.reason
    assert run.core.pending is None


def test_skipped_temperature_still_decides_and_names_it():
    """Skipped temperature still decides and names it."""
    run = Run()
    run.begin()
    run.feed(lambda i: good(ir_valid=False), 10)
    result = run.until_deadline()
    assert result.outcome == dc.OUTCOME_HOLD and 'temperature not measured' in result.summary
    assert 'temperature not measured' in result.reason     # the rule that needs it names it


def test_a_skipped_temperature_cannot_dispense_a_temperature_rule():
    """A skipped temperature cannot dispense a temperature rule."""
    run = Run()
    run.begin()
    run.feed(lambda i: good(ir_valid=False, contact_valid=False), 10)
    assert run.until_deadline().outcome != dc.OUTCOME_DISPENSE


def test_contact_sensor_is_used_when_the_ir_sensor_is_invalid():
    """Contact sensor is used when the ir sensor is invalid."""
    run = Run()
    run.begin()
    result = run.feed(lambda i: good(ir_valid=False, contact_valid=True, contact_c=38.4), 20)
    assert result.outcome == dc.OUTCOME_DISPENSE


def test_no_finger_means_hr_and_spo2_are_unusable():
    """No finger means hr and spo2 are unusable."""
    run = Run()
    run.begin()
    run.feed(lambda i: good(finger=False), 10)
    result = run.until_deadline()
    assert result.outcome == dc.OUTCOME_UNRELIABLE
    assert 'HR and SpO2' in result.reason


def test_unstable_hr_alone_is_unreliable_and_names_it():
    """Unstable hr alone is unreliable and names it."""
    run = Run()
    run.begin()
    result = run.feed(lambda i: good(hr=60 + 20 * (i % 2)), 12)     # jumps 60, 80, 60, 80
    assert result is None
    result = run.until_deadline()
    assert result.outcome == dc.OUTCOME_UNRELIABLE and result.reason == 'HR not stable in time'


def test_spo2_note_band_is_marked_but_does_not_stop_a_dose():
    """Spo2 note band is marked but does not stop a dose."""
    run = Run()
    run.begin()
    result = run.feed(lambda i: good(spo2=93, ir=38.6), 20)
    assert result.outcome == dc.OUTCOME_DISPENSE and 'SpO2 93 (low, note)' in result.summary


def test_feedback_reports_settling_then_counts():
    """Feedback reports settling then counts."""
    run = Run()
    run.begin()
    assert run.core.feedback(run.now + 2)[0].startswith('settling')
    run.feed(lambda i: good(), 3)
    status, stable = run.core.feedback(run.now)
    assert 'HR 3 of 5' in status and stable == 3


def test_a_cancelled_evaluation_gives_no_result():
    """A cancelled evaluation gives no result."""
    run = Run()
    run.begin()
    run.core.cancel()
    assert run.core.poll(run.now + 50, run.wall) is None


# --- dose interval, allergies and recording doses ------------------------------------------
def dispense(run, ir=38.6, patient='P1'):
    """Run one evaluation to its result."""
    run.begin(patient)
    return run.feed(lambda i: good(ir=ir), 20)


def test_dose_interval_blocks_a_second_dose_until_six_hours_pass():
    """Dose interval blocks a second dose until six hours pass."""
    run = Run()
    run.history.record('P1', 'paracetamol', run.wall - 3 * HOUR)
    result = dispense(run)
    assert result.outcome == dc.OUTCOME_HOLD and 'given 3.0 h ago, minimum 6 h' in result.reason
    run.wall += 3.1 * HOUR
    assert dispense(run).outcome == dc.OUTCOME_DISPENSE


def test_known_allergy_blocks_the_drug():
    """Known allergy blocks the drug."""
    run = Run(patients={'P1': {'allergies': ['paracetamol']}})
    result = dispense(run)
    assert result.outcome == dc.OUTCOME_HOLD and 'allergic' in result.reason


def test_allergies_not_on_record_still_dispenses_but_tells_the_nurse():
    """Allergies not on record still dispenses but tells the nurse."""
    for patients in ({}, {'P1': {'allergies': None}}):
        run = Run(patients=patients)
        result = dispense(run)
        assert result.outcome == dc.OUTCOME_DISPENSE
        assert 'allergies not on record, please confirm' in result.summary


def test_known_empty_allergy_list_gives_no_allergy_warning():
    """Known empty allergy list gives no allergy warning."""
    result = dispense(Run())
    assert 'allergies' not in result.summary


def test_a_dose_is_recorded_only_when_return_follows_dispense():
    """A dose is recorded only when return follows dispense."""
    run = Run()
    dispense(run)
    for state in ('WAIT_APPROVAL', 'DISPENSE', 'RETURN', 'IDLE'):
        run.core.on_mission_state(state, run.wall)
    assert run.history.last('P1', 'paracetamol') == run.wall


@pytest.mark.parametrize('states', [
    ('WAIT_APPROVAL', 'ALERT', 'GOING_HOME', 'IDLE'),            # nurse declined
    ('WAIT_APPROVAL', 'DISPENSE', 'ALERT', 'GOING_HOME', 'IDLE'),  # the drawer failed
    ('WAIT_APPROVAL', 'FAILED'),
])
def test_declined_or_failed_dispense_records_nothing(states):
    """Declined or failed dispense records nothing."""
    run = Run()
    dispense(run)
    for state in states:
        run.core.on_mission_state(state, run.wall)
    assert run.history.last('P1', 'paracetamol') is None and run.core.pending is None


def test_a_new_evaluation_drops_an_unfinished_pending_dose():
    """A new evaluation drops an unfinished pending dose."""
    run = Run()
    dispense(run)
    run.begin()
    assert run.core.pending is None


def test_hold_and_unreliable_never_leave_a_pending_dose():
    """Hold and unreliable never leave a pending dose."""
    run = Run()
    dispense(run, ir=38.0)          # 38.0 is not above 38.0
    assert run.core.pending is None


def test_mission_state_names_match_mission_core_when_available():
    """Mission state names match mission core when available."""
    sibling = Path(__file__).resolve().parents[2] / 'isocare_mission'
    if sibling.is_dir():
        sys.path.insert(0, str(sibling))
    mission_core = pytest.importorskip('isocare_mission.mission_core')
    assert dc.STATE_DISPENSE == mission_core.DISPENSE
    assert dc.STATE_RETURN == mission_core.RETURN
    assert set(dc.STATES_THAT_DROP_A_DOSE) <= {
        mission_core.ALERT, mission_core.GOING_HOME, mission_core.FAILED, mission_core.IDLE,
        mission_core.GO_TO_BED}
    assert (dc.OUTCOME_DISPENSE, dc.OUTCOME_HOLD, dc.OUTCOME_UNRELIABLE) == (
        mission_core.OUTCOME_DISPENSE, mission_core.OUTCOME_HOLD,
        mission_core.OUTCOME_UNRELIABLE)


# --- rule order ---------------------------------------------------------------------------
def test_first_passing_rule_wins_in_file_order():
    """First passing rule wins in file order."""
    rules = [dc.Rule('first', 0, {dc.TEMP: {'above': 38.0}}),
             dc.Rule('second', 2, {dc.TEMP: {'above': 37.0}})]
    result = dispense(Run(rules=rules))
    assert result.drawer_id == 0 and 'rule: first' in result.summary


def test_a_rule_that_fails_falls_through_to_the_next():
    """A rule that fails falls through to the next."""
    rules = [dc.Rule('first', 0, {dc.HR: {'above': 100}}),
             dc.Rule('second', 2, {dc.TEMP: {'above': 37.0}})]
    result = dispense(Run(rules=rules))
    assert result.drawer_id == 2


def test_no_rules_configured_holds_with_a_clear_reason():
    """No rules configured holds with a clear reason."""
    result = dispense(Run(rules=[]))
    assert result.outcome == dc.OUTCOME_HOLD and 'no drug rules configured' in result.reason


# --- bands --------------------------------------------------------------------------------
def test_band_classification_edges():
    """Band classification edges."""
    bands = dc.Bands(danger_low=90.0, note_low=95.0)
    assert bands.classify(89.9) == ('danger', 'low')
    assert bands.classify(90.0) == ('note', 'low')
    assert bands.classify(95.0) == ('normal', '')
    assert bands.classify(100.0) == ('normal', '')


# --- config files -------------------------------------------------------------------------
def write(tmp_path, text, name='f.yaml'):
    """Write a YAML file under tmp_path and return its path."""
    path = tmp_path / name
    path.write_text(textwrap.dedent(text))
    return path


def test_shipped_config_files_load():
    """Shipped config files load."""
    assert [r.drug for r in cfg.load_rules(CONFIG_DIR / 'rules.yaml')] == ['paracetamol']
    assert set(cfg.load_bands(CONFIG_DIR / 'bands.yaml')) == set(dc.VITALS)
    patients = cfg.load_patients(CONFIG_DIR / 'patients.yaml')
    assert patients['P001']['allergies'] == []


@pytest.mark.parametrize('body, message', [
    ('rules: []\nextra', 'cannot read'),
    ('rules:\n  - drug: a b\n    drawer_id: 1\n    when: {hr: {above: 1}}', 'one word'),
    ('rules:\n  - drug: x\n    drawer_id: 3\n    when: {hr: {above: 1}}', 'drawer_id'),
    ('rules:\n  - drug: x\n    drawer_id: 1\n    when: {}', 'at least one'),
    ('rules:\n  - drug: x\n    drawer_id: 1\n    when: {pulse: {above: 1}}', 'unknown vital'),
    ('rules:\n  - drug: x\n    drawer_id: 1\n    when: {hr: {over: 1}}', 'bad test'),
    ('rules:\n  - drug: x\n    drawer_id: 1\n    when: {hr: {above: high}}', 'bad test'),
    ('rules:\n  - drug: x\n    drawer_id: 1\n    when: {hr: {above: 1}}\n    min_interval_h: -1',
     'min_interval_h'),
    ('rules:\n  - drug: x\n    drawer_id: 1\n    when: {hr: {above: 1}}\n'
     '  - drug: x\n    drawer_id: 2\n    when: {hr: {above: 1}}', 'more than one'),
    ('nothing', 'top level'),
])
def test_bad_rules_files_are_rejected_with_a_clear_message(tmp_path, body, message):
    """Bad rules files are rejected with a clear message."""
    with pytest.raises(cfg.ConfigError, match=message):
        cfg.load_rules(write(tmp_path, body))


def test_missing_file_is_a_config_error(tmp_path):
    """Missing file is a config error."""
    with pytest.raises(cfg.ConfigError, match='cannot read'):
        cfg.load_rules(tmp_path / 'nope.yaml')


def test_bad_bands_files_are_rejected(tmp_path):
    """Bad bands files are rejected."""
    full = 'hr: {danger_low: 40}\nspo2: {danger_low: 90}\ntemperature_c: {danger_low: 35.0}\n'
    cfg.load_bands(write(tmp_path, full))
    with pytest.raises(cfg.ConfigError, match='missing "temperature_c"'):
        cfg.load_bands(write(tmp_path, 'hr: {}\nspo2: {}\n'))
    with pytest.raises(cfg.ConfigError, match='order'):
        cfg.load_bands(write(tmp_path, full.replace('{danger_low: 40}',
                                                    '{danger_low: 60, note_low: 50}')))
    with pytest.raises(cfg.ConfigError, match='unknown key'):
        cfg.load_bands(write(tmp_path, full.replace('danger_low: 40', 'low: 40')))


def test_patients_file_distinguishes_empty_from_missing_allergies(tmp_path):
    """Patients file distinguishes empty from missing allergies."""
    path = write(tmp_path, 'patients:\n  P1: {allergies: []}\n  P2: {}\n  P3:\n')
    patients = cfg.load_patients(path)
    assert patients['P1']['allergies'] == []
    assert patients['P2']['allergies'] is None and patients['P3']['allergies'] is None
    with pytest.raises(cfg.ConfigError, match='must be a list'):
        cfg.load_patients(write(tmp_path, 'patients:\n  P1: {allergies: nuts}\n', 'b.yaml'))


# --- dose history -------------------------------------------------------------------------
def test_history_keeps_the_latest_dose_per_patient_and_drug(tmp_path):
    """History keeps the latest dose per patient and drug."""
    db = DoseHistory(str(tmp_path / 'sub' / 'h.db'))
    assert db.last('P1', 'x') is None
    db.record('P1', 'x', 10.0)
    db.record('P1', 'x', 20.0)
    db.record('P2', 'x', 5.0)
    assert db.last('P1', 'x') == 20.0 and db.last('P2', 'x') == 5.0
    assert DoseHistory(str(tmp_path / 'sub' / 'h.db')).last('P1', 'x') == 20.0   # survives reopen
    assert db.clear() == 2 and db.last('P1', 'x') is None
