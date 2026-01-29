# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_velodyne_pcl_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED velodyne_pcl_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(velodyne_pcl_FOUND FALSE)
  elseif(NOT velodyne_pcl_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(velodyne_pcl_FOUND FALSE)
  endif()
  return()
endif()
set(_velodyne_pcl_CONFIG_INCLUDED TRUE)

# output package information
if(NOT velodyne_pcl_FIND_QUIETLY)
  message(STATUS "Found velodyne_pcl: 0.0.0 (${velodyne_pcl_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'velodyne_pcl' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT velodyne_pcl_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(velodyne_pcl_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${velodyne_pcl_DIR}/${_extra}")
endforeach()
