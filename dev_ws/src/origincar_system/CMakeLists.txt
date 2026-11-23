cmake_minimum_required(VERSION 3.8)
project(origincar_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(origincar_route_planning STATIC
  src/route_geometry.cpp
  src/circle_envelope.cpp
  src/circle_envelope_candidate.cpp
  src/waypoint_shift_confirmation.cpp
)
target_compile_features(origincar_route_planning PUBLIC cxx_std_17)
target_include_directories(origincar_route_planning PUBLIC include)

add_executable(visual_lidar_path_follower
  src/visual_lidar_path_follower.cpp
  src/control_loop.cpp
  src/control_output.cpp
  src/follower_parameters.cpp
  src/follower_localization.cpp
  src/race_mission.cpp
  src/channel_transition.cpp
  src/route_tracking.cpp
  src/static_route_planning.cpp
  src/route_shaping.cpp
  src/lidar_avoidance.cpp
  src/obstacle_tracking.cpp
  src/route_diagnostics.cpp
)
target_compile_features(visual_lidar_path_follower PRIVATE cxx_std_17)
target_include_directories(visual_lidar_path_follower PRIVATE include)
ament_target_dependencies(
  visual_lidar_path_follower
  ai_msgs geometry_msgs nav_msgs rclcpp rcutils sensor_msgs std_msgs tf2 tf2_ros visualization_msgs)
target_link_libraries(visual_lidar_path_follower origincar_route_planning yaml-cpp)

install(TARGETS
  visual_lidar_path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config foxglove map DESTINATION share/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/generate_channel_waypoints.py
  scripts/generate_route_profile.py
  scripts/generate_task1_high.py
  scripts/generate_task1_low.py
  scripts/generate_task3_fast.py
  scripts/generate_task3_slow.py
  scripts/generate_task1_waypoints.py
  scripts/open_foxglove.sh
  scripts/route_snapshot_renderer.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(test_circle_envelope test/test_circle_envelope.cpp)
  target_link_libraries(test_circle_envelope PRIVATE origincar_route_planning)
  add_test(NAME circle_envelope COMMAND test_circle_envelope)

  add_executable(test_waypoint_shift_confirmation test/test_waypoint_shift_confirmation.cpp)
  target_link_libraries(test_waypoint_shift_confirmation PRIVATE origincar_route_planning)
  add_test(NAME waypoint_shift_confirmation COMMAND test_waypoint_shift_confirmation)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
