# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_odometry_estimator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED odometry_estimator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(odometry_estimator_FOUND FALSE)
  elseif(NOT odometry_estimator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(odometry_estimator_FOUND FALSE)
  endif()
  return()
endif()
set(_odometry_estimator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT odometry_estimator_FIND_QUIETLY)
  message(STATUS "Found odometry_estimator: 0.0.1 (${odometry_estimator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'odometry_estimator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${odometry_estimator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(odometry_estimator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${odometry_estimator_DIR}/${_extra}")
endforeach()
