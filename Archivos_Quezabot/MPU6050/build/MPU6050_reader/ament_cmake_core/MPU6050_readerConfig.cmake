# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_MPU6050_reader_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED MPU6050_reader_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(MPU6050_reader_FOUND FALSE)
  elseif(NOT MPU6050_reader_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(MPU6050_reader_FOUND FALSE)
  endif()
  return()
endif()
set(_MPU6050_reader_CONFIG_INCLUDED TRUE)

# output package information
if(NOT MPU6050_reader_FIND_QUIETLY)
  message(STATUS "Found MPU6050_reader: 0.0.0 (${MPU6050_reader_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'MPU6050_reader' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT MPU6050_reader_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(MPU6050_reader_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${MPU6050_reader_DIR}/${_extra}")
endforeach()
