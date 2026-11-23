cmake_minimum_required(VERSION 3.8)
project(line_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-unresolved-symbols=ignore-in-shared-libs")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  /usr/include
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(/usr/hobot/lib)

add_executable(line_detector_node
  src/line_detector.cpp
  src/line_detector_node.cpp
)

ament_target_dependencies(line_detector_node
  rclcpp
  std_msgs
  builtin_interfaces
  hbm_img_msgs
  ai_msgs
  geometry_msgs
  OpenCV
)

target_link_libraries(line_detector_node
  ${OpenCV_LIBS}
  dnn
  hbucp
  pthread
  rt
  dl
  m
)

install(TARGETS line_detector_node
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
