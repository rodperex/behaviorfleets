cmake_minimum_required(VERSION 3.5)
project(bf_example)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bf_msgs REQUIRED)
find_package(behaviorfleets REQUIRED)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp
    ament_index_cpp
    bf_msgs
    behaviorfleets
)

include_directories(include)

add_executable(bf_example 
  src/main.cpp
  src/Test.cpp
)

ament_target_dependencies(bf_example ${dependencies})

install(TARGETS
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
