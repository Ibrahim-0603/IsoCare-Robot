# decision_engine: what it does (for Youssef Hesham to check)

The engine does not choose drugs. It applies the rules in `rules.yaml` to measured vitals and
asks a nurse to approve.

## 1. Reading the vitals
- Ignore the first 5 s after the engine is asked to evaluate.
- Each vital (HR, SpO2, temperature) is judged on its own: it is stable after 5 valid readings in a
  row whose spread (max minus min) is within a tolerance: HR 5 bpm, SpO2 2 points, temperature 0.3 C.
  The stable value is the average.
- HR and SpO2 only count while a finger is detected. Temperature uses the forehead sensor, and the
  contact sensor only if the forehead one is not valid.
- If HR or SpO2 is not stable after 80 s the answer is UNRELIABLE (the robot re-measures once, then
  alerts a nurse). If only temperature fails it is skipped, and the nurse is told.

## 2. Abnormal limits (`bands.yaml`, placeholders)
| Vital | Danger below | Note below | Note above | Danger above |
| --- | --- | --- | --- | --- |
| HR (bpm) | 40 | 50 | 110 | 130 |
| SpO2 (%) | 90 | 95 | none | none |
| Temperature (C) | 35.0 | 36.0 | 37.5 | 39.5 |
Danger = hold all drugs and alert the nurse. Note = continue, but the nurse sees the value marked.

## 3. Drug rules (`rules.yaml`)
Checked in file order, the first that passes is offered. A rule can name vitals with above, below,
at_least or at_most, a minimum number of hours since the last dose, and allergies that block it. A
rule that needs a vital that was not measured is skipped. Placeholder: paracetamol, drawer 1,
temperature above 38.0, at least 6 h between doses.

## 4. Doses and allergies
A dose is recorded only when the visit really reaches the return trip after dispensing, not when
the engine suggests it or the nurse approves. Allergy data comes from `patients.yaml`; a patient
with no allergy entry is flagged "allergies not on record, please confirm" and the nurse decides.

## 5. Questions for Hesham
1. Real limits for the table in section 2?
2. Which drugs, thresholds and intervals for the demo?
3. Patient ids and allergies for the demo (they must match mission_manager's bed list)?
4. Is the 0.3 C temperature tolerance realistic for the forehead sensor?
