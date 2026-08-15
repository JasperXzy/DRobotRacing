cmake_minimum_required(VERSION 3.8)
project(dual_rate_tts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(ALSA REQUIRED alsa)

set(TROS_DISTRO "$ENV{TROS_DISTRO}")
if(TROS_DISTRO STREQUAL "")
  set(TROS_DISTRO "humble")
endif()
set(TROS_LIB_DIR "/opt/tros/${TROS_DISTRO}/lib")

find_library(TTS_LIBRARY
  NAMES tts
  PATHS "${TROS_LIB_DIR}"
  NO_DEFAULT_PATH)
set(ONNXRUNTIME_LIBRARY
  "${TROS_LIB_DIR}/libonnxruntime.so.1.11.1")

if(NOT TTS_LIBRARY)
  message(FATAL_ERROR "libtts.so was not found under ${TROS_LIB_DIR}")
endif()
if(NOT EXISTS "${ONNXRUNTIME_LIBRARY}")
  message(FATAL_ERROR
    "libonnxruntime.so.1.11.1 was not found under ${TROS_LIB_DIR}")
endif()

add_executable(dual_rate_tts_node src/dual_rate_tts_node.cpp)
target_compile_features(dual_rate_tts_node PUBLIC cxx_std_17)
target_include_directories(dual_rate_tts_node PRIVATE
  include
  ${ALSA_INCLUDE_DIRS})
ament_target_dependencies(dual_rate_tts_node rclcpp std_msgs)
target_link_libraries(dual_rate_tts_node
  ${TTS_LIBRARY}
  ${ONNXRUNTIME_LIBRARY}
  ${ALSA_LIBRARIES}
  ${CMAKE_DL_LIBS}
  Threads::Threads)
set_target_properties(dual_rate_tts_node PROPERTIES
  BUILD_RPATH "${TROS_LIB_DIR}"
  INSTALL_RPATH "${TROS_LIB_DIR}")

install(TARGETS dual_rate_tts_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
