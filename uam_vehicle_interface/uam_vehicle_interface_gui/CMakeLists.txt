cmake_minimum_required(VERSION 3.8)
project(uam_vehicle_interface_gui)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uam_vehicle_interface_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(uam_navigator_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent UiTools)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(library_name vehicle_rviz_plugins)

include_directories(include)

set(${library_name}_headers
  include/uam_vehicle_interface_gui/vehicle_interface_gui.hpp
)

add_library(${library_name} SHARED
  src/vehicle_interface_gui.cpp
  ${${library_name}_headers}
)

set(${library_name}_dependencies
  geometry_msgs
  uam_vehicle_interface_msgs
  uam_navigator_msgs
  nav_msgs
  Qt5
  rclcpp
  pluginlib
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  yaml_cpp_vendor
)

ament_target_dependencies(${library_name}
  ${${library_name}_dependencies}
)

target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(${library_name}
  rviz_common::rviz_common
  Qt5::UiTools
)

target_compile_definitions(${library_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  geometry_msgs
  uam_vehicle_interface_msgs
  uam_navigator_msgs
  nav_msgs
  rclcpp
)

ament_package()
