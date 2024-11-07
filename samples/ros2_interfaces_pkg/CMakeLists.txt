cmake_minimum_required(VERSION 3.8)
project(ros2_interfaces_pkg)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Message.msg
  srv/Service.srv
  action/Action.action
  DEPENDENCIES
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
