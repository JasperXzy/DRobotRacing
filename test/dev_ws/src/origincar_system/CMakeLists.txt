cmake_minimum_required(VERSION 3.8)
project(origincar_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(visual_lidar_path_follower src/visual_lidar_path_follower.cpp)
target_include_directories(visual_lidar_path_follower PRIVATE include)
ament_target_dependencies(
  visual_lidar_path_follower
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_ros visualization_msgs)
target_link_libraries(visual_lidar_path_follower yaml-cpp)

install(TARGETS
  visual_lidar_path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config foxglove map DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/open_foxglove.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
