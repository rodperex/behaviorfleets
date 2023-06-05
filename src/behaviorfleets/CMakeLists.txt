cmake_minimum_required(VERSION 3.5)
project(behaviorfleets)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bf_msgs REQUIRED)


set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp
    ament_index_cpp
    bf_msgs
)


include_directories(include)


# add_library(blackboard_manager SHARED src/behaviorfleets/BlackboardManager.cpp)
add_library(delegate_action_node SHARED src/behaviorfleets/DelegateActionNode.cpp)
add_library(remote_delegate_action_node SHARED src/behaviorfleets/RemoteDelegateActionNode.cpp)


list(APPEND plugin_libs
  delegate_action_node
  remote_delegate_action_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(source src/exec/source_main.cpp)
ament_target_dependencies(source ${dependencies})
target_link_libraries(source yaml-cpp)

add_executable(bbshare src/exec/bbtest_main.cpp)
ament_target_dependencies(bbshare ${dependencies})
target_link_libraries(bbshare yaml-cpp)

add_executable(remote src/exec/remote_main.cpp)
ament_target_dependencies(remote ${dependencies})
target_link_libraries(remote remote_delegate_action_node)

add_executable(remoteconfig src/exec/remoteconf_main.cpp)
ament_target_dependencies(remoteconfig ${dependencies})
target_link_libraries(remoteconfig remote_delegate_action_node yaml-cpp)


install(TARGETS
  ${plugin_libs}
  source
  remote
  remoteconfig
  bbshare
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
