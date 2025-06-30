# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_racing_obstacle_detection_yolo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED racing_obstacle_detection_yolo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(racing_obstacle_detection_yolo_FOUND FALSE)
  elseif(NOT racing_obstacle_detection_yolo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(racing_obstacle_detection_yolo_FOUND FALSE)
  endif()
  return()
endif()
set(_racing_obstacle_detection_yolo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT racing_obstacle_detection_yolo_FIND_QUIETLY)
  message(STATUS "Found racing_obstacle_detection_yolo: 2.0.1 (${racing_obstacle_detection_yolo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'racing_obstacle_detection_yolo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${racing_obstacle_detection_yolo_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(racing_obstacle_detection_yolo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${racing_obstacle_detection_yolo_DIR}/${_extra}")
endforeach()
