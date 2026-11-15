# route_planner: setup and check

## 1. Wiring (once)
- isocare_navigation/setup.py, add to `console_scripts`:
      'route_planner = isocare_navigation.route_planner:main',
- isocare_navigation/package.xml, add:
      <exec_depend>python3-yaml</exec_depend>
- isocare_bringup must install the new config folder. If it is ament_python, add to `data_files`
  in setup.py (next to the launch line):
      (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
  If it is ament_cmake, add to CMakeLists.txt:
      install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
- Unzip over the repo, then `cb` (isocare_interfaces gets the new PlanRoute.srv, so rebuild all).

## 2. Check (the launch file now starts route_planner too)
    ros2 launch isocare_bringup robot.launch.py
    -> route_planner logs: loaded 5 markers and 5 edges from .../markers.yaml

    ros2 service call /plan_route isocare_interfaces/srv/PlanRoute "{from_name: 'dock', to_name: 'B'}"
    -> names: [dock, junction1, A, B], total_m about 8.54

    ros2 service call /plan_route isocare_interfaces/srv/PlanRoute "{from_name: 'dock', to_name: 'B', blocked_from: ['A'], blocked_to: ['B']}"
    -> names: [dock, junction1, junction2, B], total_m 9.0

## 3. The marker table
src/isocare_bringup/config/markers.yaml holds PLACEHOLDER numbers. After measuring the real space,
edit that one file: the ArUco id printed on each marker, its name, x/y in metres, the direction it
faces, and which pairs are directly connected. A wrong table stops the node at startup with a message
saying what and where (duplicate ids, a connection to an unknown name, unreachable markers, ...).
