cmake_minimum_required(VERSION 3.8)
project(bmi088_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(bmi088_device src/bmi088_device.cpp)
target_include_directories(bmi088_device PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(bmi088_node src/bmi088_node.cpp)
target_link_libraries(bmi088_node bmi088_device)
ament_target_dependencies(bmi088_node
  diagnostic_msgs
  rclcpp
  sensor_msgs
  std_srvs)

install(TARGETS bmi088_device bmi088_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config dts launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/check_spi.sh
  scripts/install_s100_spi0_cs1_overlay.sh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bmi088_device test/test_bmi088_device.cpp)
  if(TARGET test_bmi088_device)
    target_link_libraries(test_bmi088_device bmi088_device)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(bmi088_device)
ament_package()
