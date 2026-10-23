cmake_minimum_required(VERSION 3.8)
# 고규범 신규 패키지 (2026-10-21): common observation interfaces and monitor.
project(vehicle_recorder)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandEvent.msg" "msg/StopStatus.msg"
  DEPENDENCIES std_msgs builtin_interfaces)
install(PROGRAMS scripts/stop_observer scripts/record DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/stop_observation.py scripts/recorder_node.py scripts/run_recorder.py scripts/path_monitor.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_stop_observation COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_stop_observation.py)
  add_test(NAME test_recorder COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_recorder.py)
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
