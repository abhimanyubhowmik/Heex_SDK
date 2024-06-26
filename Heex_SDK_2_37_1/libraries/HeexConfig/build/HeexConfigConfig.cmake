get_filename_component(HEEXCONFIG_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

list(APPEND CMAKE_MODULE_PATH ${HEEXCONFIG_CMAKE_DIR})

set(HEEX_BUILD_DIR "build/")
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(HEEX_BUILD_DIR "debugBuild/")
endif()

# #############################################
# Targets import

if(NOT TARGET HeexConfig::HeexConfig)
  # #############################################
  # Path works for import. Mostly relative to Heex.

  # Check mandatory variable to be set
  if(NOT DEFINED HEEX_SDK_DIR)
    # HEEXCONFIG_CMAKE_DIR points to the libraries/HeexConfig/build folder. We compute the realpath for two parent directories above.
    get_filename_component(HEEX_SDK_DIR "${HEEXCONFIG_CMAKE_DIR}/../../.." REALPATH)
    message(STATUS "Auto Setting Heex SDK path to HEEX_SDK_DIR=${HEEX_SDK_DIR}")
  endif()

  if(NOT DEFINED HEEX_LIBRARIES_DIR)
    set(HEEX_LIBRARIES_DIR ${HEEX_SDK_DIR}/libraries)
    message(STATUS "Auto Setting HEEX_LIBRARIES_DIR=${HEEX_LIBRARIES_DIR}")
  endif()

  if(NOT DEFINED HEEX_3RDPARTY_DIR)
    set(HEEX_3RDPARTY_DIR ${HEEX_SDK_DIR}/3rdparty)
    message(STATUS "Auto Setting HEEX_3RDPARTY_DIR=${HEEX_3RDPARTY_DIR}")
  endif()

  include(${HEEXCONFIG_CMAKE_DIR}/FindHeexConfigDeps.cmake)
  include("${HEEXCONFIG_CMAKE_DIR}/HeexConfigTargets.cmake")
endif()

set(HEEXCONFIG_lIBRARIES HeexConfig::HeexConfig)
