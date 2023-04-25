cmake_minimum_required(VERSION 3.8)
project(uam_navigator)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(uam_navigator_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(uam_util REQUIRED)
find_package(uam_vehicle_interface REQUIRED)
find_package(uam_planner_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

add_library(navigator_modes SHARED
  src/navigator_modes/navigate_to_pose.cpp
  src/navigator_modes/loiter.cpp
  src/navigator_modes/takeoff.cpp)
ament_target_dependencies(navigator_modes rclcpp rclcpp_action Eigen3 nav_msgs uam_util uam_planner_msgs uam_navigator_msgs)
target_include_directories(navigator_modes PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

set(navigator_dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  pluginlib
  builtin_interfaces
  uam_navigator_msgs
  nav_msgs
  uam_util
  rclcpp_components
)

add_library(navigator SHARED
  src/navigator.cpp)
target_link_libraries(navigator navigator_modes)
ament_target_dependencies(navigator ${navigator_dependencies})
target_include_directories(navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

rclcpp_components_register_nodes(navigator "uam_navigator::Navigator")

add_executable(navigator_main src/navigator_main.cpp)
ament_target_dependencies(navigator_main ${navigator_dependencies})
target_include_directories(navigator_main PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(navigator_main navigator)
install(TARGETS navigator_main DESTINATION lib/${PROJECT_NAME})

install(TARGETS navigator navigator_modes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(navigator)
ament_export_dependencies(${navigator_dependencies})
ament_package()
