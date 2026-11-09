cmake_minimum_required(VERSION 3.8)
project(sentry_runtime)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sentry_system_interfaces REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS sentry_runtime/tunnel_region_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tunnel_region_monitor)
install(PROGRAMS sentry_runtime/referee_state_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME referee_state_adapter)
ament_package()
