# mission_manager skeleton: setup and demo

## 1. New package isocare_mission
In ~/isocare_ws/src run (this makes package.xml and setup.py with your maintainer lines):

    ros2 pkg create --build-type ament_python isocare_mission --dependencies rclpy std_msgs action_msgs isocare_interfaces

Unzip over the repo (it adds isocare_mission/isocare_mission/*.py, test/, README.md), then in
isocare_mission/setup.py add to `console_scripts`:

    'mission_manager = isocare_mission.mission_manager:main',
    'demo_stubs = isocare_mission.demo_stubs:main',

## 2. isocare_interfaces (new actions and one service)
The zip adds action/*.action, srv/StartVisit.srv and an updated CMakeLists.txt.
In isocare_interfaces/package.xml add (once):

    <depend>action_msgs</depend>

Also make sure `std_msgs` is still a dependency there. Then rebuild everything (`cb`), and read the
top of the build output for "Failed to parse ROS package manifest".

## 3. Run the demo (5 terminals)
1. socat:   socat -d -d pty,raw,echo=0,link=/tmp/esp_fake pty,raw,echo=0,link=/tmp/esp_pi
2. fake:    python3 tools/fake_esp32.py /tmp/esp_fake --servo-drawers
3. launch:  ros2 launch isocare_bringup robot.launch.py use_stubs:=true
4. watch:   ros2 topic echo /mission/state
5. start:   ros2 service call /mission/start isocare_interfaces/srv/StartVisit "{bed_id: 'A'}"

Expected: states GO_TO_BED, MEASURE, DECIDE, WAIT_APPROVAL, DISPENSE (the fake's drawer 1 opens by
itself, 20 s later closes), RETURN, IDLE. About 35 s in total.

Try failures (stop and restart demo_stubs between runs):
    ros2 run isocare_mission demo_stubs --ros-args -p fail:="['nav']"        # blocked path
    ros2 run isocare_mission demo_stubs --ros-args -p evaluate_outcome:=1    # hold all drugs
    ros2 run isocare_mission demo_stubs --ros-args -p approve:=false         # nurse declines
    (type `estop on` at the fake's prompt during the visit)

## 4. Wording to change in protocol.md (servo drawers: unlock opens, lock closes)
- DRAWER_CMD relock_s: "Auto-close and relock this many seconds after the drawer has OPENED;
  0 = stay open until a lock command."  (was: after the drawer is closed)
- DRAWER_CMD action: "unlock = the servo opens the drawer; lock = the servo closes and locks it."
- DRAWER_STATUS: open_mask now confirms the mechanism moved, not that a person took anything.
- To raise with Belal: stall and pinch safety for a servo that closes a drawer (low torque, slow
  close, stalled servo = stop and report a fault).

## 5. Not done yet
- Real navigator, patient_id, decision_engine and app_bridge (demo_stubs answers for them).
- decision_engine must read /vitals itself (ignore the first 5 s, then 5 valid close readings in
  a row, average them) while the manager has /vitals/measure running.
- The marker reset in odometry (waits for marker_detector and the /markers message).
