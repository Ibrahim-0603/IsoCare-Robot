# IsoCare: context for your AI assistant (paste or attach this at the start of a chat)

Your AI assistant does not know this project. Give it this file, `Docs/ros_interfaces.md`,
`Docs/protocol.md` (only if you touch the serial link), and the `.action`/`.msg`/`.srv` files in
`src/isocare_interfaces/` for the interfaces you use. Then tell it which role you are (section 6).

## 1. The project in 10 lines

IsoCare is a small wheeled medicine cabinet for isolation wards (university course demo, **feature
freeze Sun Nov 22, demo Mon Dec 7**). It drives to a bed on ArUco markers, identifies the patient
(wristband QR, or the nurse confirms), measures heart rate, SpO2 and temperature, checks them against
doctor-set rules, and after a nurse approves, unlocks the correct drawer. It never prescribes and
never chooses a new drug. Abnormal vitals mean hold all drugs and alert the nurse. Never dispense on a
sensor error. Hobby-grade sensors, harmless placeholder pills.

Team: Belal (hardware), Ibrahim (ROS 2, navigation, the visit state machine, decision_engine), Youssef Hesham (AI:
vision, patient ID, decision rules content), Youssef Eslam (software: nurse interface, data, alerts).

## 2. Platform

ROS 2 **Jazzy**, Ubuntu 24.04, Python (`ament_python` nodes), built with `colcon`. The robot computer
is a laptop in an Ubuntu VM or a Raspberry Pi 5. An ESP32 does motors and sensors and is **not** a ROS
node. Do not suggest Nav2, SLAM, micro-ROS or other large frameworks: the team deliberately avoids them.

## 3. How the nodes fit together

`mission_manager` (Ibrahim) runs one visit at a time and **calls** your node:
GO_TO_BED -> IDENTIFY -> MEASURE -> DECIDE -> WAIT_APPROVAL -> DISPENSE -> RETURN.
Your nodes are **action servers** with fixed names and types (see `ros_interfaces.md`):
`/patient/identify`, `/decision/evaluate`, `/nurse/approval`, `/nurse/alert`.
Do not change an interface file on your own. Ask Ibrahim: the manager, the stubs and the tests
change together.

## 4. Rules for the code your AI writes (these caused real crashes)

1. **Wire types:** float fields need a Python `float`, int fields an `int`, strings a `str`. A wrong
   type aborts the whole process. Cast explicitly.
2. Never name your own `Node` attributes `_clients`, `_services`, `_subscriptions`, `_publishers`,
   `_timers`, `_guards`, `_waitables`, `_parameters`. rclpy uses them.
3. Action servers: always return a result (`success=false` + `reason` on failure, never an uncaught
   exception), honour cancel (`is_cancel_requested`), return before the manager's timeout.
4. Waiting goal callbacks need a `MultiThreadedExecutor` and `ReentrantCallbackGroup`.
5. Put the logic in a ROS-free file (`*_core.py`) that takes the time as an argument, and keep the
   node file thin (it only moves data). That is how it gets unit tests without ROS.
6. All numbers (thresholds, timeouts) are ROS parameters. Floats are written with a decimal point.
7. Latched topics (`/mission/state`, `/drawer/status`, `/esp_status`) use QoS RELIABLE +
   TRANSIENT_LOCAL, depth 1.
8. Style: 99 columns, single quotes, Google import order, a docstring on everything.
9. Be honest about what was run. Code that was not run against real ROS must be described as untested.
   Never claim a result you did not see.
10. If something fails, paste the exact terminal output to the AI and diagnose from it.

## 5. How to work (this matches how Ibrahim's side worked and it went well)

- Understand first, code second: explain each design decision in plain words with a small example and
  a recommended default, get a yes, then build.
- One concept at a time. Short answers.
- A package that fails to parse its `package.xml` is silently ignored by `colcon build` even when it
  says "Finished". Read the top of the build output for `Failed to parse ROS package manifest` and run
  `ros2 pkg executables <pkg>`.
- Launch files and config files need a `glob` line in `setup.py` `data_files`.

## 6. Your role (tell your AI which one)

**Youssef Hesham, patient_id** (`/patient/identify`, `Identify`), YOLO and marker work with Ibrahim
(`marker_detector`, `yolo_detector`), and the **content of the decision rules**: which drugs, thresholds
and the patient record format (allergies, last dose time). He gives Ibrahim the rule table.
Rules for patient_id are in `ros_interfaces.md` section 4. Face matching is a stretch goal.

**Ibrahim, decision_engine** (`/decision/evaluate`, `Evaluate`), `mission_manager`, navigation and
serial. Decision logic is simple, rule-based and testable on fake vitals with no ROS.
See `ros_interfaces.md` section 2.

**Youssef Eslam, app_bridge** (`/nurse/approval`, `/nurse/alert`, optionally a "start visit" button that
calls `/mission/start`). Core: a minimal nurse approval page served from the robot computer.
Flutter app, push alerts, live video are stretch and only after the core works. Data logging (SQLite)
is also yours.

## 7. Prompt to paste into your AI at the start

> You are helping me build one node of a ROS 2 Jazzy project (IsoCare, a medication and vitals robot).
> I have attached: AI_CONTEXT.md (project and rules), ros_interfaces.md (exact interfaces my node must
> serve), and the interface files. My role is: [patient_id and rules content / app_bridge].
> Follow the rules in AI_CONTEXT.md section 4 and the way of working in section 5. Do not write code
> yet: first confirm you understand my node's interface in your own words (names, types, outcomes,
> time limits), list what you will build, and ask me one question. Then work one concept at a time.
