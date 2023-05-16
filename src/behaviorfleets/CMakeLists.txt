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
)


include_directories(include)


# add_library(delegation_node SHARED src/behaviorfleets/DelegationNode.cpp)
# add_library(delegate_bt_node SHARED src/behaviorfleets/deco/DelegateBT.cpp)
add_library(delegate_action_node SHARED src/behaviorfleets/DelegateActionNode.cpp)
add_library(delegate_action_node_any SHARED src/behaviorfleets/DelegateActionNodeAny.cpp)
add_library(remote_delegate_action_node SHARED src/behaviorfleets/RemoteDelegateActionNode.cpp)
add_library(remote_delegate_action_node_any SHARED src/behaviorfleets/RemoteDelegateActionNodeAny.cpp)

list(APPEND plugin_libs
  # delegation_node
  # delegate_bt_node
  delegate_action_node
  delegate_action_node_any
  remote_delegate_action_node
  remote_delegate_action_node_any
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(source_tree src/examples/source_tree_main.cpp)
ament_target_dependencies(source_tree ${dependencies})
target_link_libraries(source_tree yaml-cpp)

add_executable(remote_tree src/examples/remote_tree_main.cpp)
ament_target_dependencies(remote_tree ${dependencies})
target_link_libraries(remote_tree remote_delegate_action_node)

add_executable(remote_tree_any src/examples/remote_tree_main_any.cpp)
ament_target_dependencies(remote_tree_any ${dependencies})
target_link_libraries(remote_tree_any remote_delegate_action_node_any)


install(TARGETS
  ${plugin_libs}
  source_tree
  remote_tree
  remote_tree_any
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

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
