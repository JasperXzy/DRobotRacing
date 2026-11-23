cmake_minimum_required(VERSION 3.8)
project(smac_pid)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(smac_pid_node src/smac_pid_node.cpp)
target_compile_features(smac_pid_node PUBLIC cxx_std_17)
ament_target_dependencies(
  smac_pid_node
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(smac_pid_node yaml-cpp)

install(TARGETS smac_pid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

set(SMAC_PID_DOC_FILES)
foreach(doc_file README.md 快捷指令.md 常用调参.md)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${doc_file}")
    list(APPEND SMAC_PID_DOC_FILES "${doc_file}")
  endif()
endforeach()

if(SMAC_PID_DOC_FILES)
  install(FILES ${SMAC_PID_DOC_FILES}
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
