add_library(${library_name} SHARED
  node_utils.cpp
  node_thread.cpp
  lifecycle_node.cpp
  lifecycle_service_client.cpp
)

ament_target_dependencies(${library_name}
  rclcpp
  lifecycle_msgs
  rclcpp_lifecycle
  bondcpp
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)