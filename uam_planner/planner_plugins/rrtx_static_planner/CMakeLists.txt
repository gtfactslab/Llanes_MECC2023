cmake_minimum_required(VERSION 3.8)
project(rrtx_static_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(ompl REQUIRED)
find_package(uam_mapping_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(uam_planner REQUIRED)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)

set(library_name rrtx_static)

set(dependencies
  rclcpp
  Eigen3
  geometric_shapes
  uam_mapping_msgs
  uam_planner
  pluginlib
)

add_library(${library_name} SHARED
  src/rrtx_static.cpp
)

target_link_libraries(${library_name} ${OMPL_LIBRARIES})

ament_target_dependencies(${library_name}
  ${dependencies}
)

pluginlib_export_plugin_description_file(uam_planner rrtx_static_planner_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES rrtx_static_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include ${OMPL_INCLUDE_DIRS})
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()