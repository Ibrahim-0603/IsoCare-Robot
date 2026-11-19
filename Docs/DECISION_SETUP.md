# decision_engine: setup and first run

New package `src/isocare_decision/` (copy the whole folder into your workspace `src/`).
It already has `package.xml`, `setup.py`, `setup.cfg` and `resource/`. **Check `package.xml`
against your other packages** (maintainer email and license are placeholders): a package whose
`package.xml` does not parse is silently skipped by `colcon build`.

## Build and check

```
cd ~/isocare_ws
colcon build --packages-select isocare_decision 2>&1 | head -20     # read the top for "Failed to parse"
source install/setup.bash
ros2 pkg executables isocare_decision        # must list decision_engine and clear_dose_history
```

## Run it next to the rest (stubs for everything except the evaluate action)

```
ros2 launch isocare_bringup robot.launch.py use_stubs:=false
ros2 run isocare_mission demo_stubs --ros-args -p stub_nav:=true -p skip:="['evaluate']"
ros2 run isocare_decision decision_engine
```

A goal by hand, with fake vitals (a fever) from a second terminal:

```
ros2 topic pub -r 1 /vitals isocare_interfaces/msg/Vitals "{hr_valid: true, spo2_valid: true, ir_temp_valid: true, finger_detected: true, heart_rate_bpm: 72, spo2_percent: 98, temp_ir_centi_c: 3860}"
ros2 action send_goal /decision/evaluate isocare_interfaces/action/Evaluate "{patient_id: 'P001', bed_id: 'A'}" --feedback
```

Expect about 10 s of feedback ("settling", then "stable readings ...") and then
`outcome: 0, drawer_id: 1`. Run it again right away: the dose is only recorded when a visit
goes DISPENSE then RETURN, so the second run still dispenses until you run a full visit.

## Reset the dose history before rehearsals

```
ros2 run isocare_decision clear_dose_history
```

## Launch file (optional, when you want it started with the rest)

Add to `robot.launch.py` (floats need a decimal point):

```python
Node(package='isocare_decision', executable='decision_engine', name='decision_engine',
     output='screen'),
```

## Files Youssef Hesham edits (placeholders until he confirms)

`config/rules.yaml` (drugs), `config/bands.yaml` (abnormal limits), `config/patients.yaml`
(allergies). A mistake in any of them stops the node at start with a message naming the file.
After editing, rebuild (`colcon build --packages-select isocare_decision`) or pass
`-p rules_file:=/path/to/rules.yaml`.

## Not verified here

pytest (50 new tests, 352 in all), lint and `dev/smoke_decision.py` pass on stub rclpy. Real
rclpy behaviour (multithreaded executor, the `/vitals` subscription, cancel timing, QoS on
`/mission/state`) is untested until it runs on your server.
