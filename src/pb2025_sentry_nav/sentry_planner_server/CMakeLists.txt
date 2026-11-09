cmake_minimum_required(VERSION 3.5)
project(sentry_planner_server)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tdt_yastar_planner REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name planner_server)
set(library_name ${PROJECT_NAME}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  pluginlib
  nav2_core
  tdt_yastar_planner
)

add_library(${library_name} SHARED
  src/planner_server.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "sentry_planner_server::PlannerServer")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES LICENSE README.md UPSTREAM.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_goal_scoped_cancel test/test_goal_scoped_cancel.cpp TIMEOUT 45)
  ament_target_dependencies(test_goal_scoped_cancel rclcpp rclcpp_action nav2_util nav2_msgs)
  ament_add_gtest(test_planner_cancel_dispatch test/test_planner_cancel_dispatch.cpp TIMEOUT 30)
  target_link_libraries(test_planner_cancel_dispatch ${library_name})
  ament_target_dependencies(test_planner_cancel_dispatch ${dependencies})
  ament_add_gtest(test_planner_callback_cancel test/test_planner_callback_cancel.cpp TIMEOUT 45)
  target_link_libraries(test_planner_callback_cancel ${library_name})
  ament_target_dependencies(test_planner_callback_cancel ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
