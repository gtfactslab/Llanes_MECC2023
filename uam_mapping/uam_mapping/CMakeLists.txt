cmake_minimum_required(VERSION 3.8)
project(uam_mapping)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uam_mapping_msgs REQUIRED)

include_directories(include)

add_executable(obstacle_advertiser src/obstacle_advertiser.cpp)
ament_target_dependencies(obstacle_advertiser rclcpp Eigen3 shape_msgs uam_mapping_msgs)
install(TARGETS obstacle_advertiser DESTINATION lib/${PROJECT_NAME})


ament_package()
