"""Load and check the three YAML files of the decision_engine (rules, bands, patients).

Every problem raises ConfigError with a message that names the file and the entry, so a typo is
found when the node starts, not in the middle of a visit.
"""

from isocare_decision.decision_core import Bands, Rule, VITALS

OPS = ('above', 'below', 'at_least', 'at_most')
BAND_KEYS = ('danger_low', 'note_low', 'note_high', 'danger_high')


class ConfigError(Exception):
    """A config file is missing or wrong."""


def _load(path):
    """Read one YAML file."""
    import yaml
    try:
        with open(path) as handle:
            return yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError(f'cannot read {path}: {error}') from error


def _number(value):
    """Return True for a real number (a bool is not one)."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def load_rules(path):
    """Return the list of Rule objects of rules.yaml, in file order."""
    data = _load(path)
    if not isinstance(data, dict) or not isinstance(data.get('rules'), list):
        raise ConfigError(f'{path}: top level must be "rules:" followed by a list')
    rules = []
    for index, entry in enumerate(data['rules']):
        where = f'{path}: rule {index + 1}'
        if not isinstance(entry, dict):
            raise ConfigError(f'{where} must be a mapping')
        drug = entry.get('drug')
        if not isinstance(drug, str) or not drug.strip() or ' ' in drug.strip():
            raise ConfigError(f'{where}: "drug" must be one word')
        where = f'{path}: rule "{drug}"'
        drawer = entry.get('drawer_id')
        if isinstance(drawer, bool) or drawer not in (0, 1, 2):
            raise ConfigError(f'{where}: "drawer_id" must be 0, 1 or 2')
        when = entry.get('when')
        if not isinstance(when, dict) or not when:
            raise ConfigError(f'{where}: "when" needs at least one condition')
        for vital, test in when.items():
            if vital not in VITALS:
                raise ConfigError(f'{where}: unknown vital "{vital}" (use {", ".join(VITALS)})')
            if not isinstance(test, dict) or not test:
                raise ConfigError(f'{where}: "{vital}" needs a test such as {{above: 38.0}}')
            for op, limit in test.items():
                if op not in OPS or not _number(limit):
                    raise ConfigError(f'{where}: bad test "{op}: {limit}" '
                                      f'(operators {", ".join(OPS)}, value a number)')
        interval = entry.get('min_interval_h', 0)
        if not _number(interval) or interval < 0:
            raise ConfigError(f'{where}: "min_interval_h" must be a number of hours >= 0')
        allergies = entry.get('blocked_by_allergy', [])
        if not isinstance(allergies, list) or not all(isinstance(a, str) for a in allergies):
            raise ConfigError(f'{where}: "blocked_by_allergy" must be a list of names')
        rules.append(Rule(drug.strip(), drawer, when, float(interval), tuple(allergies)))
    names = [rule.drug for rule in rules]
    if len(set(names)) != len(names):
        raise ConfigError(f'{path}: a drug appears in more than one rule')
    return rules


def load_bands(path):
    """Return {vital: Bands} from bands.yaml. Every vital must be present."""
    data = _load(path)
    if not isinstance(data, dict):
        raise ConfigError(f'{path}: must be a mapping of vital names')
    bands = {}
    for vital in VITALS:
        entry = data.get(vital)
        if not isinstance(entry, dict):
            raise ConfigError(f'{path}: missing "{vital}"')
        for key in entry:
            if key not in BAND_KEYS:
                raise ConfigError(f'{path}: {vital}: unknown key "{key}" (use '
                                  f'{", ".join(BAND_KEYS)})')
        values = {key: entry.get(key) for key in BAND_KEYS}
        for key, value in values.items():
            if value is not None and not _number(value):
                raise ConfigError(f'{path}: {vital}.{key} must be a number')
        present = [values[key] for key in BAND_KEYS if values[key] is not None]
        if present != sorted(present):
            raise ConfigError(f'{path}: {vital}: limits must go in order '
                              f'danger_low <= note_low <= note_high <= danger_high')
        bands[vital] = Bands(**{key: None if v is None else float(v)
                                for key, v in values.items()})
    return bands


def load_patients(path):
    """Return {patient_id: {'allergies': list or None}} from patients.yaml."""
    data = _load(path)
    if not isinstance(data, dict) or not isinstance(data.get('patients'), dict):
        raise ConfigError(f'{path}: top level must be "patients:" followed by a mapping')
    patients = {}
    for patient_id, entry in data['patients'].items():
        entry = {} if entry is None else entry
        if not isinstance(entry, dict):
            raise ConfigError(f'{path}: patient {patient_id} must be a mapping')
        allergies = entry.get('allergies')       # missing key = not on record (None)
        if allergies is not None and (not isinstance(allergies, list)
                                      or not all(isinstance(a, str) for a in allergies)):
            raise ConfigError(f'{path}: patient {patient_id}: "allergies" must be a list')
        patients[str(patient_id)] = {'allergies': allergies}
    return patients
