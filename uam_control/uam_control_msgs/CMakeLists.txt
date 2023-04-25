cmake_minimum_required(VERSION 3.8)
project(uam_control_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
        "msg/QLearningStatus.msg"
        "msg/AttitudeSetpoint.msg"
        )

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES std_msgs
        )

ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_package()
