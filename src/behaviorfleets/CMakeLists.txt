cmake_minimum_required(VERSION 3.5)
project(behaviorfleets)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bf_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp
    ament_index_cpp
    bf_msgs
    geometry_msgs
    sensor_msgs
)


include_directories(include)


# add_library(delegation_node SHARED src/behaviorfleets/DelegationNode.cpp)
# add_library(delegate_bt_node SHARED src/behaviorfleets/deco/DelegateBT.cpp)
add_library(delegate_action_node SHARED src/behaviorfleets/DelegateActionNode.cpp)
add_library(remote_delegate_action_node SHARED src/behaviorfleets/RemoteDelegateActionNode.cpp)
# add_library(t SHARED src/behaviorfleets/Test.cpp)

# ament_target_dependencies(t ${dependencies})
ament_target_dependencies(remote_delegate_action_node ${dependencies})


list(APPEND plugin_libs
  # delegation_node
  # delegate_bt_node
  delegate_action_node
  remote_delegate_action_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(source_tree src/examples/source_tree_main.cpp)
ament_target_dependencies(source_tree ${dependencies})
# new
target_link_libraries(source_tree yaml-cpp)

add_executable(remote_tree src/examples/remote_tree_main.cpp)
ament_target_dependencies(remote_tree ${dependencies})
target_link_libraries(remote_tree remote_delegate_action_node)


install(TARGETS
  ${plugin_libs}
  source_tree
  remote_tree
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
