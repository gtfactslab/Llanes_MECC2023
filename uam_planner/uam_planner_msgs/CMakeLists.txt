cmake_minimum_required(VERSION 3.8)
project(uam_planner_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
)

set(action_files
  "action/ComputePathToPose.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
#  ${msg_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
