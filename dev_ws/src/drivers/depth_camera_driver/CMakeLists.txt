cmake_minimum_required(VERSION 3.5)
project(depth_camera_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED) 
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# Specialization for AURORA930
set(STREAM_SDK_TYPE "AURORA930")
add_definitions(-DSTREAM_SDK_TYPE_AURORA930)

set(EXTERNAL_DIR_NAME ext)

string(TOLOWER "${CMAKE_SYSTEM_NAME}" LOWER_SYSTEM_NAME)
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" LOWER_PROCESSOR_NAME)
set(SYSTEM_PLATFORM "${LOWER_SYSTEM_NAME}-${LOWER_PROCESSOR_NAME}")
message(STATUS "SYSTEM_PLATFORM: ${SYSTEM_PLATFORM}")

set(DEPTRUM_STREAM_SDK_VERSION 1.1.22)
set(STREAM_SDK_LIBS deptrum_stream_aurora900)
set(STREAM_SDK_PKG_NAME "deptrum-stream-aurora900-${SYSTEM_PLATFORM}-v${DEPTRUM_STREAM_SDK_VERSION}-18.04")
set(DEPTRUM_STREAM_SDK_NAME aurora900)

# SDK path configuration (Local files only)
set(STREAM_SDK_PACKAGE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${EXTERNAL_DIR_NAME}/${STREAM_SDK_PKG_NAME})

# Always define these paths so they are available
set(DEPTRUM_STREAM_AURORA900_INCLUDE_DIRS "${STREAM_SDK_PACKAGE_PATH}/include")
set(DEPTRUM_STREAM_AURORA900_STATIC_LIBRARY_DIRS "${STREAM_SDK_PACKAGE_PATH}/lib")

include_directories(include)
include_directories(${DEPTRUM_STREAM_AURORA900_INCLUDE_DIRS})
link_directories(${DEPTRUM_STREAM_AURORA900_STATIC_LIBRARY_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Build the Aurora930 node
add_executable(aurora930_node src/ros2_node.cc src/ros2_device.cc src/aurora900_ros2_device.cc src/camera_parameters.cc)

target_link_libraries(aurora930_node
  ${STREAM_SDK_LIBS}
  opencv_core
  opencv_highgui
)

# Export libraries
ament_export_libraries(deptrum_stream_aurora900.so)
ament_export_libraries(export_aurora930_node)
ament_export_libraries(export_${PROJECT_NAME})

ament_target_dependencies(aurora930_node rclcpp std_msgs sensor_msgs cv_bridge geometry_msgs tf2 tf2_ros tf2_geometry_msgs angles)

# Installation
install(TARGETS aurora930_node DESTINATION lib/${PROJECT_NAME})

install(FILES
  ${STREAM_SDK_PACKAGE_PATH}/scripts/99-deptrum-libusb.rules
  DESTINATION share/${PROJECT_NAME}
)

install(FILES 
  ${DEPTRUM_STREAM_AURORA900_STATIC_LIBRARY_DIRS}/libdeptrum_stream_aurora900.so.${DEPTRUM_STREAM_SDK_VERSION}
  DESTINATION lib
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")

install(FILES rviz/aurora930-ros2.rviz DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
