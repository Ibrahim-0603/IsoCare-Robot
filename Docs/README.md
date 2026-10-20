# IsoCare

An autonomous medication and vitals robot for isolation wards.

IsoCare is a small wheeled medicine cabinet that does routine rounds in quarantined rooms. At each bed it identifies the patient, takes quick measurements (heart rate, SpO2, temperature), checks them against rules set by a doctor, and either unlocks the correct drawer or alerts a nurse. It never prescribes and it does not replace nurses: doctors set the rules, nurses load the drawers and confirm before anything is dispensed.

Final demo planned for **Mon Dec 6-10, 2026** (date to be confirmed).

> **Status:** early development (Week 5, foundations). Details below may change as the team makes decisions.

## Team

| Member | Area (provisional) |
| --- | --- |
| Belal Tamer | Hardware: sensors, motors, power, data acquisition |
| Ibrahim Tamer | ROS 2: node architecture, communication, navigation |
| Youssef Hesham | AI: vision, patient/bed detection, decision logic |
| Youssef Eslam | Software: nurse/doctor interface, data, alerts |

Roles are not fixed and may change.

## How a round works

1. A nurse loads the drawers.
2. The robot navigates to the bed.
3. It identifies the patient (wristband QR; face match is a stretch goal).
4. It measures heart rate, SpO2 and temperature.
5. It decides against the doctor's rules and the patient record (allergies, dose intervals).
6. It acts: unlock the correct drawer after nurse approval, or alert the nurse.

## System architecture

```
 Raspberry Pi 5 (ROS 2 Jazzy, Ubuntu 24.04)
 ├─ camera, marker_detector, yolo_detector
 ├─ route_planner, navigator, odometry
 ├─ mission_manager, patient_id, decision_engine
 ├─ voice, app_bridge
 └─ serial_bridge ──── USB serial (custom framed protocol) ────┐
                                                               │
 ESP32 (low-level controller, not a ROS node)  ◄───────────────┘
 ├─ motors + encoders, IMU, range sensors
 ├─ vitals sensors
 └─ drawer locks
```

- **Pi 5** runs ROS 2, OpenCV and YOLO. The webcam plugs into the Pi.
- **ESP32** handles real-time hardware work. It is not a ROS node (no micro-ROS); `serial_bridge` talks to it over USB using a custom frame: `SYNC | TYPE | LEN | payload | XOR checksum`.


### Safety (enforced on the ESP32)

- Motors stop if `/cmd_vel` is silent for 0.5 s.
- Slows at 40 cm and stops at 15 cm from an obstacle, independent of the Pi.
- Hardware E-stop cuts motor power through a relay.
- Never dispenses on a sensor error. Abnormal vitals mean hold all drugs and alert the nurse.

## Navigation

No Nav2, SLAM, LiDAR, costmap or EKF. Instead:

- ArUco markers at junctions, bedsides and the dock form a graph. Each marker's position is tape-measured into a table; edges hold distance and heading.
- Dijkstra picks the marker route.
- The navigator drives leg by leg at about 0.3 m/s or less. It steers on the marker's bearing when visible, otherwise dead-reckons from encoder ticks plus gyro yaw. Every marker sighting resets position and heading.
- The ESP32 streams raw ticks, gyro and ranges (about 50 to 100 Hz); the `odometry` node on the Pi computes `/odom`.
- Final approach: align on the marker, set standoff with ToF/ultrasonic, YOLO confirms the bed or person. Target about ±5 cm and ±10°.
- Blocked path: wait, retry, alternate edge, otherwise alert the nurse. Lost: spin to find a marker, otherwise stop and alert.

## Vitals, dispensing and decisions

- **Sensors:** MAX30102 (HR + SpO2), MLX90614 (non-contact temperature), MAX30205 (contact temperature, cross-check). These are hobby-grade, not medical devices; readings are validated against commercial devices.
- **Drawers:** servo locks with limit switches that confirm each drawer opened. Only the correct drawer unlocks. The demo uses harmless placeholder pills.
- **Decisions:** simple rules for common drugs in common cases (for example, paracetamol only if temperature is above 38 °C and at most once every 6 h). Allergy and dose-interval checks come from the patient record. An unreliable reading means re-measure, then call the nurse. A nurse or doctor must confirm before dispensing. The AI never chooses new drugs.
- **AI:** YOLOv8n fine-tuned to confirm the bed or person at final approach; OpenCV ArUco for markers.
- **Interface:** a minimal nurse/doctor approval page served from the Pi is core. Flutter app, push alerts, live video and voice are stretch.
- **Data:** logged in SQLite and rosbag2.

## Scope

**Core (must be in the demo):** drive to a bed via markers, obstacle stop, QR patient ID, HR/SpO2/temperature, rule-based decision with nurse approval, drawer unlock with confirmation, fail-safes, logging.

**Stretch (decided at the Week 8 checkpoint, in this order):** Flutter doctor app with push alerts, voice keywords, face match, live video.

## Timeline

Organized by function rather than by person. Each week ends with a proof that something works end to end. Features freeze before the final weeks. This plan is not final.

| Week | Dates | Focus | Proof |
| --- | --- | --- | --- |
| 5 | Oct 19 | **Foundations.** Confirm deadline, finalize BOM, order long-lead parts. Agree serial frame spec, ROS topics and message types. Set up repo, Pi and ESP32 toolchain. Measure demo space. Fake-data skeleton of the whole chain. | A fake `/cmd_vel` goes Pi to `serial_bridge` to ESP32, and fake ticks return as `/sensor_data`. MAX30102 and MLX90614 each read on a breadboard. |
| 6 | Oct 26 | **Drive base and vitals alive.** Chassis, motors, drivers, encoders, battery, E-stop, power rails. ESP32 speed control, watchdog, obstacle stop. Odometry. Camera and ArUco with calibration. Vitals processing. One baseline YOLO run on the Pi (CPU, RAM, FPS). | Drives 2 m and turns 90° by teleop within tolerance, stops for an obstacle, reads a marker at 1 to 2 m, streams vitals to `/vitals`. |
| 7 | Nov 2 | **Navigation core and decisions.** Marker graph, `route_planner`, `navigator`, marker table. Drawers and unlock command. Patient records, QR read, decision engine v1. Photos for YOLO fine-tuning. | Docks to bed A on its own over 2+ legs. Decision engine passes tests on fake vitals. A drawer opens only on its command. |
| 8 | Nov 9 | **First full visit.** Final approach, `mission_manager` state machine, patient script, nurse approval page, fail-safes and recovery. | One full visit (help allowed): dock, bed, ID, measure, decide, approve, dispense, return. |
| 9 | Nov 16 | **Reliability, then freeze.** Repeat visits, tune, recalibrate, add a second bed. Stretch features only if Week 8 passed. **Feature freeze Sun Nov 22.** | A visit with no help and a two-bed round. |
| 10 | Nov 23 | **Testing only** (bug fixes allowed). Vitals vs commercial devices, navigation (10 runs per route), safety tests. | Test log and plots per metric. |
| 11 | Nov 30 | **Report and rehearsal.** Report, plots from logs, demo script, three full rehearsals, backup video, spare parts. | Ready to demo. |
| | Dec 7 | **Demo day** | |

## Getting started

Setup instructions will be added as each part comes together. Planned environment:

- Robot: Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy
- Firmware: ESP32 (toolchain to be confirmed)
- Development: ROS packages are built with `colcon`; build output (`build/`, `install/`, `log/`) is not committed
