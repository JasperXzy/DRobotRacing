cmake_minimum_required(VERSION 3.8)
project(qr_detector_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)  # 添加pkg-config支持
find_package(origincar_msg REQUIRED)

pkg_check_modules(ZBAR REQUIRED zbar)  # 使用pkg-config查找zbar

add_executable(qr_detector_node src/qr_detector_node.cpp)
target_compile_features(qr_detector_node PUBLIC cxx_std_17)
target_include_directories(qr_detector_node PUBLIC
  ${ZBAR_INCLUDE_DIRS}  # 添加ZBar头文件目录
)
ament_target_dependencies(qr_detector_node rclcpp sensor_msgs std_msgs cv_bridge OpenCV origincar_msg)
target_link_libraries(qr_detector_node ${ZBAR_LIBRARIES})  # 链接ZBar库

install(TARGETS
  qr_detector_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
