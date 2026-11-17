# navigator (+ odometry set_pose, planner marker positions, forward-only obstacle stop)

## 1. Wiring (once)
- isocare_navigation/setup.py, add to `console_scripts`:
      'navigator = isocare_navigation.navigator:main',
- isocare_navigation/package.xml: make sure these are listed (add what is missing):
      <depend>rclpy</depend> <depend>geometry_msgs</depend> <depend>nav_msgs</depend>
      <depend>isocare_interfaces</depend> <depend>python3-yaml</depend>
- Unzip over the repo, then `cb` (isocare_interfaces has new SetPose.srv and a changed PlanRoute.srv,
  so everything rebuilds). Read the top of the build output for "Failed to parse ROS package manifest".

## 2. Staged check (5 terminals: socat, fake, launch, goals, spare)
    socat -d -d pty,raw,echo=0,link=/tmp/esp_fake pty,raw,echo=0,link=/tmp/esp_pi
    python3 tools/fake_esp32.py /tmp/esp_fake --servo-drawers
    ros2 launch isocare_bringup robot.launch.py use_stubs:=true

1. Startup. navigator logs `dock pose set: ready (dock assumed)`, odometry logs `pose set to x=0.40
   y=0.00 heading=3.14`, and `ros2 topic echo /odom --once` shows x 0.4 (the robot starts parked
   in front of the dock marker, facing it).
2. A goal on its own:
       ros2 action send_goal /nav/go_to isocare_interfaces/action/GoTo "{target_id: 'A'}" --feedback
   Expect: it turns on the spot first (about 7 s), then drives, feedback lines like `to junction1,
   1.8 m`, about 30 s in total, result success=true. `/odom` ends near x 2.9, y 1.5, heading about
   1.5 (facing the marker). Then the same with target_id 'dock' to drive back.
3. Failures: target_id 'Z' gives success=false, reason 'no route'. While a goal is driving, type
   `range front 100` at the fake's prompt: the robot stops and waits (log: path blocked: waiting);
   type `range front 2000` within 10 s and it resumes. Leave it at 100 for 20 s and it plans around
   (log: avoiding junction1-A) or fails with reason 'blocked'. Note: the fake's range does not move
   when the robot turns, so after a replan it will still read 100 mm. `estop on` gives 'e-stop'.
4. The whole visit (the mission manager now uses the REAL navigator; demo_stubs answers the rest):
       ros2 service call /mission/start isocare_interfaces/srv/StartVisit "{bed_id: 'A'}"
   About 90 s: drive to A, measure, approval, drawer opens by itself and closes after 20 s,
   drive back to the dock.

## 3. Wording to change in protocol.md section 7
- "Obstacle within 15 cm: motors stop, OBST_STOP flag" becomes: "Obstacle within 15 cm: FORWARD motion
  is blocked and the OBST_STOP flag is set. Turning on the spot and reversing stay allowed, so the
  robot can turn away."
- "Obstacle within 40 cm: speed limited" becomes: "... FORWARD speed is limited to 100 mm/s; turning
  keeps its speed."
- To tell Belal: the real firmware must follow this (limit the forward part of the wheel speeds, keep
  the turn part), or a robot stopped by an obstacle can never get out.

## 4. Defaults chosen without asking (all parameters of navigator)
- arrive_radius_m 0.5 (I said about 0.4: it must be above the 0.4 m dock standoff, which is where the
  robot starts); the final approach starts standoff + 0.3 m from the marker; heading tolerance 10 deg.
- The final approach and facing must finish within 30 s, or the goal fails with 'no progress'.
- Blocked: 'blocked' means front range <= 200 mm or OBST_STOP while commanding forward motion; it is
  clear again at >= 350 mm. The waiting rounds are 10 s each, two by default (blocked_retries = 1).
- No data from /odom or /sensor_data for 0.5 s counts as a lost link ('link lost').
- Startup assumes the robot is on the dock when the navigator starts. Restarting it elsewhere puts
  the pose in the wrong place until the marker reset exists (odometry v2).

## 5. Not done yet
- Marker detection: no camera alignment, no pose reset from markers, no spin-to-find-a-marker.
- YOLO confirmation at the bed (Youssef Hesham's node).
- Real tuning: speeds, gains, standoffs and radii are placeholders until the chassis exists.
