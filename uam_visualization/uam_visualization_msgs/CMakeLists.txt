cmake_minimum_required(VERSION 3.8)
project(uam_visualization_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  )

set(action_files

  )

#rosidl_generate_interfaces(${PROJECT_NAME}
##  ${msg_files}
#  #  ${action_files}
#  DEPENDENCIES builtin_interfaces
#  )

ament_export_dependencies(rosidl_default_runtime)
ament_package()
