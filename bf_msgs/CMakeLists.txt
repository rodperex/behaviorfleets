project(bf_msgs)

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionCommand.msg"
  "msg/MissionStatus.msg"
  "msg/Mission.msg"
  "msg/Blackboard.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
