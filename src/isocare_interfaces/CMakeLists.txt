cmake_minimum_required(VERSION 3.8)
project(isocare_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  "msg/Vitals.msg"
  "msg/DrawerStatus.msg"
  "msg/EspStatus.msg"
  "msg/LinkStats.msg"
  "srv/DrawerUnlock.srv"
  "srv/Measure.srv"
  "srv/SystemCmd.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the copyright linter until a license header is added to all files.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
