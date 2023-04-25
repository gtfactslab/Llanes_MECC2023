cmake_minimum_required(VERSION 3.8)
project(uam_navigator_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
  "msg/Path.msg"
  "msg/NavigatorStatus.msg"
)

set(action_files
  "action/NavigatorCommand.action"
)


rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${action_files}
        DEPENDENCIES std_msgs geometry_msgs nav_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs nav_msgs)
ament_package()
