cmake_minimum_required(VERSION 3.8)
project(perception_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-unresolved-symbols=ignore-in-shared-libs")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  /usr/include
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(/usr/hobot/lib)

add_executable(perception_detector_node
  src/perception_detector_node.cpp
  src/yolo26_detector.cpp
  src/nv12_utils.cpp
  src/qr_roi_publisher.cpp
)

ament_target_dependencies(perception_detector_node
  rclcpp
  sensor_msgs
  builtin_interfaces
  hbm_img_msgs
  ai_msgs
  OpenCV
)

target_link_libraries(perception_detector_node
  ${OpenCV_LIBS}
  dnn
  hbucp
  pthread
  rt
  dl
  m
)

install(TARGETS perception_detector_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
