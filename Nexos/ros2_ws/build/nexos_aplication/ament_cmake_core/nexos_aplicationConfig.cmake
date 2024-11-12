# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_nexos_aplication_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED nexos_aplication_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(nexos_aplication_FOUND FALSE)
  elseif(NOT nexos_aplication_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(nexos_aplication_FOUND FALSE)
  endif()
  return()
endif()
set(_nexos_aplication_CONFIG_INCLUDED TRUE)

# output package information
if(NOT nexos_aplication_FIND_QUIETLY)
  message(STATUS "Found nexos_aplication: 0.0.0 (${nexos_aplication_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'nexos_aplication' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${nexos_aplication_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(nexos_aplication_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${nexos_aplication_DIR}/${_extra}")
endforeach()
