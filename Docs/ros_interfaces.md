# IsoCare ROS interfaces for decision_engine, app_bridge and patient_id

**Owners:** decision_engine = Ibrahim (rule content comes from Youssef Hesham), app_bridge = Youssef Eslam, patient_id = Youssef Hesham.

**Status:** draft v1 (Nov 19, 2026). Feature freeze is **Sun Nov 22**. If you need a change, tell
Ibrahim before you code it: the interface files, `mission_manager` and the stubs change together.

You do not call hardware or navigation. `mission_manager` calls you. Your node is an **action
server** (or reads a topic) and answers when asked. The interface files are in
`src/isocare_interfaces/` (`action/`, `msg/`, `srv/`). That folder is the source of truth: if this
page and a `.action` file disagree, the `.action` file wins and this page has a bug.

## 0. The big picture (one visit)

`mission_manager` runs one visit at a time:

`GO_TO_BED -> IDENTIFY -> MEASURE -> DECIDE -> WAIT_APPROVAL -> DISPENSE -> RETURN -> IDLE`

| Step | mission_manager calls | Owner |
| --- | --- | --- |
| IDENTIFY | action `/patient/identify` (only when `identify_mode` is `qr`) | patient_id |
| MEASURE + DECIDE | starts the sensors, then action `/decision/evaluate` | decision_engine |
| WAIT_APPROVAL | action `/nurse/approval` | app_bridge |
| any failure | action `/nurse/alert` | app_bridge |

Why actions and not services: each of these takes seconds or minutes (waiting for stable vitals,
waiting for a nurse). An action gives the caller feedback, and lets it **cancel**.

## 1. Rules that apply to every action you serve

1. **Answer with a result, do not crash.** If you cannot do the job, return `success=false` (or
   `approved=false`) with a short `reason`. `mission_manager` turns that into an alert for the nurse.
2. **Always finish.** Every goal needs a result, or an abort/cancel. A goal that never ends freezes
   the visit until `mission_manager` times out (see the timeouts below).
3. **Honour cancel.** If the manager cancels (visit aborted, E-stop), stop working, call
   `goal_handle.canceled()` and return. Check `is_cancel_requested` in any loop or wait.
4. **One goal at a time is enough.** The manager never sends two goals to the same action.
5. **Wire types.** Float fields need a Python `float`, int fields an `int`, strings a `str`.
   A wrong type does not raise a nice error, it **aborts the whole process**.
   Cast explicitly: `float(x)`, `int(x)`, `str(x)`.
6. **Do not name your own attributes** `_clients`, `_services`, `_subscriptions`, `_publishers`,
   `_timers`, `_guards`, `_waitables` or `_parameters` in a `Node` subclass. rclpy uses them.
7. Use a `MultiThreadedExecutor` and a `ReentrantCallbackGroup` if your goal callback waits
   (sleeps, loops). Otherwise a waiting goal blocks the cancel callback.
8. Failures come back as a normal result with `success=false`, not as an exception.

## 2. decision_engine: action `/decision/evaluate` (`action/Evaluate`)

**Goal:** `string patient_id`, `string bed_id`

**Result:** `bool success`, `uint8 outcome`, `uint8 drawer_id`, `string summary`, `string reason`

**Feedback:** `string status`, `uint8 stable_readings`

| `outcome` | Meaning | What mission_manager does |
| --- | --- | --- |
| 0 `OUTCOME_DISPENSE` | A rule allows a drug; `drawer_id` (0 to 2) says which | asks the nurse to approve |
| 1 `OUTCOME_HOLD` | Abnormal vitals, or a rule blocks it: hold all drugs | alert the nurse |
| 2 `OUTCOME_UNRELIABLE` | No stable readings in time | one re-measure, then alert |

- `success=false` means the evaluation itself broke (bad patient id, no record). `outcome` is only
  read when `success=true`.
- **Before** your goal arrives, the manager has already started the sensors
  (`/vitals/measure`, mask 7). After your result it stops them. You never start or stop sensors.
- **You** listen to the topic `/vitals` (`isocare_interfaces/Vitals`, about 1 Hz while measuring).
  Reading validation is entirely yours:
  - Ignore the first **5 s** after the goal starts (sensors settle).
  - Then you need **5 valid readings in a row that are close together**, then average them.
  - A field is only usable if its valid flag is true (`hr_valid`, `spo2_valid`, `ir_temp_valid`,
    `contact_temp_valid`). **Zero never means "measured zero".** `finger_detected` must be true for HR/SpO2.
  - Temperature units are 0.01 degC (`3650` = 36.50 degC). `temp_ir_centi_c` (forehead) is primary,
    `temp_contact_centi_c` is the cross-check.
  - Publish feedback as you go, for example `status="stable readings 3 of 5"`.
- **Time budget:** the manager gives you **90 s** (`decide_timeout_s`). If you have no stable set
  by about 80 s, return `success=true, outcome=2`. Do not wait for the manager to time out.
- `summary` is shown to the nurse, for example `"HR 72, SpO2 98, 38.6 C"`.
- **Safety rules the team agreed (README):** abnormal vitals mean hold all drugs. Never choose a new
  drug: only apply the doctor's rules for the common cases. The patient record (allergies, dose
  intervals) comes from your own data source. Example rule: paracetamol only if temperature is
  above 38 C and at most once every 6 h.

## 3. app_bridge: actions `/nurse/approval` and `/nurse/alert`

### `/nurse/approval` (`action/Approval`)

**Goal:** `string patient_id`, `string bed_id`, `uint8 drawer_id`, `string identity_source`, `string summary`

**Result:** `bool approved`, `string reason`   **Feedback:** `string status`

- Show the nurse the bed, the patient, which drawer, and the `summary` (vitals and the rule).
- `identity_source` is `"qr"` (the wristband was scanned), `"nurse"` (**the nurse must confirm who the
  patient is**: show that clearly) or `"previous"` (same patient as the last visit to that bed).
- Block until the nurse answers. `approved=true` = dispense. `approved=false` with `reason` =
  declined (this also goes to an alert).
- Time budget: **300 s** (`approval_timeout_s`). If you are cancelled, close the approval page
  cleanly.
- Core requirement: the nurse approval page served from the Pi (or laptop). Phone app and push alerts
  are stretch.

### `/nurse/alert` (`action/Alert`)

**Goal:** `string bed_id` (may be empty), `string reason`   **Result:** `bool acknowledged`

- Show the alert (bed and reason) and wait until a nurse acknowledges. The manager waits "as long as
  it takes" and the robot does not go home until you answer. Return `acknowledged=true` when the
  nurse acknowledges.
- Return `acknowledged=false` only if you cannot reach any nurse at all.
- Typical reasons: `blocked`, `e-stop`, `link lost`, `vitals fault`, `nurse declined`,
  `unreliable readings`, `abnormal vitals`.

## 4. patient_id: action `/patient/identify` (`action/Identify`)

Only used when `identify_mode` is `qr`.

**Goal:** `string bed_id`   **Result:** `bool success`, `string patient_id`, `string reason`   **Feedback:** `string status`

- Read the wristband QR from the camera and return the patient id it contains. `success=false` with a
  `reason` such as `no QR in view` if it cannot (the nurse then confirms the patient instead).
- Time budget: **60 s**. Return before that.
- Face match is a stretch goal. Identity is always confirmed by QR or the nurse, never skipped.

## 5. Other interfaces you may read (do not publish them)

| Name | Type | Notes |
| --- | --- | --- |
| `/vitals` | `Vitals` msg | decision_engine only |
| `/mission/state` | `std_msgs/String`, latched | current state of the visit, good for a status display |
| `/drawer/status` | `DrawerStatus`, latched | `unlocked[4]`, `open[4]`, index 3 = lid |
| `/esp_status` | `EspStatus`, latched | `estop`, faults, `battery_mv` |
| `/mission/start` | `StartVisit` srv | `bed_id` -> `accepted, message`. app_bridge can offer a "start visit" button that calls this |

Latched topics use QoS `RELIABLE` + `TRANSIENT_LOCAL`, depth 1. A subscriber with the default QoS
will not receive the last message on connect.

## 6. Test your node with no robot

You do not need the robot, the ESP32 or the other nodes. You need the repo built on a Ubuntu 24.04 machine
with ROS 2 Jazzy (`colcon build && source install/setup.bash`).

1. Start the robot side with the stubs off, then start the stubs yourself with **your** action skipped:
   - `ros2 launch isocare_bringup robot.launch.py use_stubs:=false`
   - `ros2 run isocare_mission demo_stubs --ros-args -p stub_nav:=true -p skip:="['evaluate']"`
     (use `'approval', 'alert'` for app_bridge, `'identify'` for patient_id)
   - then run your own node. The stubs now serve every action except yours, so a full visit can run.
2. Call your action by hand, for example:
   `ros2 action send_goal /decision/evaluate isocare_interfaces/action/Evaluate "{patient_id: 'P001', bed_id: 'A'}" --feedback`
3. Fake vitals for decision_engine: publish `/vitals` yourself, for example
   `ros2 topic pub -r 1 /vitals isocare_interfaces/msg/Vitals "{hr_valid: true, spo2_valid: true, ir_temp_valid: true, finger_detected: true, heart_rate_bpm: 72, spo2_percent: 98, temp_ir_centi_c: 3660}"`
4. Full visit: with the setup from step 1 running (plus Ibrahim's fake ESP32 terminals), call
   `ros2 service call /mission/start isocare_interfaces/srv/StartVisit "{bed_id: 'A'}"` and watch `/mission/state`.
5. Failure behaviour to test: your node returns `success=false`; you cancel mid-goal; the nurse
   declines. Each must end in a normal result, never a hang or a crash.

## 7. Timeouts the manager uses (parameters, can be changed)

identify 60 s, decide 90 s, approval 300 s, alert: waits as long as it takes.
The manager cancels the goal when its timeout passes, and treats that as a failure.
