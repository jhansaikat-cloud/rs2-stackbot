# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_interaction_execution_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED interaction_execution_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(interaction_execution_FOUND FALSE)
  elseif(NOT interaction_execution_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(interaction_execution_FOUND FALSE)
  endif()
  return()
endif()
set(_interaction_execution_CONFIG_INCLUDED TRUE)

# output package information
if(NOT interaction_execution_FIND_QUIETLY)
  message(STATUS "Found interaction_execution: 0.0.0 (${interaction_execution_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'interaction_execution' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${interaction_execution_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(interaction_execution_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${interaction_execution_DIR}/${_extra}")
endforeach()
