cmake_minimum_required(VERSION 3.8)
project(uam_planner)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uam_planner_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(uam_util REQUIRED)

include_directories(include planner_plugins)

set(executable_name planner_server)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  nav_msgs
  geometry_msgs
  uam_planner_msgs
  uam_util
  builtin_interfaces
  tf2_ros
  pluginlib
)


add_library(${library_name} SHARED src/planner_server.cpp)

ament_target_dependencies(${library_name} ${dependencies})

add_executable(${executable_name}
  src/planner_server_main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "uam_planner::PlannerServer")

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

#pluginlib_export_plugin_description_file(uam_planner rrtx_static_plugin.xml)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
