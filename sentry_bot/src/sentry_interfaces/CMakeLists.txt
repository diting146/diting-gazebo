cmake_minimum_required(VERSION 3.8)
project(sentry_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
 "action/OpeningMission.action" "action/TraverseTunnel.action" "srv/SetMotionOwner.srv" "msg/MotionState.msg"
 DEPENDENCIES geometry_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
