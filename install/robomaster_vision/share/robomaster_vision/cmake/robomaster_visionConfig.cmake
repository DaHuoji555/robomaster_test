# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_robomaster_vision_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED robomaster_vision_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(robomaster_vision_FOUND FALSE)
  elseif(NOT robomaster_vision_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(robomaster_vision_FOUND FALSE)
  endif()
  return()
endif()
set(_robomaster_vision_CONFIG_INCLUDED TRUE)

# output package information
if(NOT robomaster_vision_FIND_QUIETLY)
  message(STATUS "Found robomaster_vision: 0.0.1 (${robomaster_vision_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'robomaster_vision' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${robomaster_vision_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(robomaster_vision_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${robomaster_vision_DIR}/${_extra}")
endforeach()
