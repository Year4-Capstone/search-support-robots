# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_behavior-tree_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED behavior-tree_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(behavior-tree_FOUND FALSE)
  elseif(NOT behavior-tree_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(behavior-tree_FOUND FALSE)
  endif()
  return()
endif()
set(_behavior-tree_CONFIG_INCLUDED TRUE)

# output package information
if(NOT behavior-tree_FIND_QUIETLY)
  message(STATUS "Found behavior-tree: 0.0.0 (${behavior-tree_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'behavior-tree' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT behavior-tree_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(behavior-tree_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${behavior-tree_DIR}/${_extra}")
endforeach()
