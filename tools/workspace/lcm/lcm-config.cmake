
# Generated by cps2cmake https://github.com/mwoehlke/pycps
# and then subsequently edited by hand.

if(CMAKE_VERSION VERSION_LESS 3.9.0)
  message(FATAL_ERROR "CMake >= 3.9 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 3.0)
set(CMAKE_IMPORT_FILE_VERSION 1)

include(CMakeFindDependencyMacro)

get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)

if(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX STREQUAL "/")
  set(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX)
endif()

find_dependency(jchart2d CONFIG HINTS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/cmake/jchart2d")
set(_expectedTargets lcm::lcm-coretypes lcm::lcm lcm::lcm-gen lcm::lcm-java lcm::lcm-logger lcm::lcm-logplayer lcm::lcm-logplayer-gui lcm::lcm-spy)

set(_targetsDefined)
set(_targetsNotDefined)

foreach(_expectedTarget ${_expectedTargets})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)

set(lcm_VERSION "1.4.0")

add_library(lcm::lcm-coretypes INTERFACE IMPORTED)
set_target_properties(lcm::lcm-coretypes PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include/lcm"
)

add_library(lcm::lcm SHARED IMPORTED)
set_target_properties(lcm::lcm PROPERTIES
  IMPORTED_LOCATION "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/libdrake_lcm.so"
  IMPORTED_SONAME "libdrake_lcm.so"
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include/lcm"
  INTERFACE_LINK_LIBRARIES "lcm::lcm-coretypes"
)

add_executable(lcm::lcm-gen IMPORTED)
set_target_properties(lcm::lcm-gen PROPERTIES
  IMPORTED_LOCATION "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/bin/lcm-gen"
)

add_library(lcm::lcm-java STATIC IMPORTED)
set_target_properties(lcm::lcm-java PROPERTIES
  IMPORTED_LOCATION "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/share/java/lcm.jar"
  JAR_FILE "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/share/java/lcm.jar"
)

add_executable(lcm::lcm-logger IMPORTED)
set_target_properties(lcm::lcm-logger PROPERTIES
  IMPORTED_LOCATION "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/bin/lcm-logger"
)

add_executable(lcm::lcm-logplayer IMPORTED)
set_target_properties(lcm::lcm-logplayer PROPERTIES
  IMPORTED_LOCATION "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/bin/lcm-logplayer"
)

add_executable(lcm::lcm-logplayer-gui IMPORTED)
set_target_properties(lcm::lcm-logplayer-gui PROPERTIES
  IMPORTED_LOCATION "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/bin/lcm-logplayer-gui"
)

add_executable(lcm::lcm-spy IMPORTED)
set_target_properties(lcm::lcm-spy PROPERTIES
  IMPORTED_LOCATION "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/bin/lcm-spy"
)

set(lcm_LIBRARIES "lcm::lcm")
set(lcm_INCLUDE_DIRS "")

set(LCM_NAMESPACE "lcm::")
set(LCM_VERSION "${lcm_VERSION}")
set(LCM_USE_FILE "${CMAKE_CURRENT_LIST_DIR}/lcmUtilities.cmake")

unset(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX)
unset(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)

