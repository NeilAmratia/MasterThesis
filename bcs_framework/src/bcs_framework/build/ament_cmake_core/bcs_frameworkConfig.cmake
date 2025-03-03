# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bcs_framework_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bcs_framework_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bcs_framework_FOUND FALSE)
  elseif(NOT bcs_framework_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bcs_framework_FOUND FALSE)
  endif()
  return()
endif()
set(_bcs_framework_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bcs_framework_FIND_QUIETLY)
  message(STATUS "Found bcs_framework: 0.0.0 (${bcs_framework_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bcs_framework' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT bcs_framework_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bcs_framework_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bcs_framework_DIR}/${_extra}")
endforeach()
