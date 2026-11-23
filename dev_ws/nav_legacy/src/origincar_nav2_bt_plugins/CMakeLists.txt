cmake_minimum_required(VERSION 3.8)
project(origincar_nav2_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(origincar_nav2_adjust_blocked_goals_bt_node SHARED
  src/adjust_blocked_goals_action.cpp
)

target_compile_features(origincar_nav2_adjust_blocked_goals_bt_node PUBLIC cxx_std_17)
target_compile_definitions(origincar_nav2_adjust_blocked_goals_bt_node PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(origincar_nav2_adjust_blocked_goals_bt_node
  behaviortree_cpp_v3
  geometry_msgs
  nav_msgs
  rclcpp
)

install(TARGETS origincar_nav2_adjust_blocked_goals_bt_node
  LIBRARY DESTINATION lib
)

ament_package()
