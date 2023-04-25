cmake_minimum_required(VERSION 3.8)
project(uam_vehicle_interface)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(uam_control_msgs REQUIRED)
find_package(uam_flight_manager_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(uam_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(uam_vehicle_interface_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_ros_com REQUIRED)

include_directories(include)

set(PX4_INTERFACE_DEPS
  Eigen3
  rclcpp
  px4_msgs
  uam_flight_manager_msgs
  uam_control_msgs
  uam_util
  nav_msgs
  uam_vehicle_interface_msgs
  tf2_ros
  px4_ros_com
)

add_executable(px4_interface src/px4_interface.cpp)
ament_target_dependencies(px4_interface ${PX4_INTERFACE_DEPS})
target_include_directories(px4_interface
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
install(TARGETS px4_interface DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()
