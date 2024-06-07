
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was HeexComConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################
get_filename_component(HEEXCOM_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

list(APPEND CMAKE_MODULE_PATH ${HEEXCOM_CMAKE_DIR})

set(HEEX_BUILD_DIR "build/")
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(HEEX_BUILD_DIR "debugBuild/")
endif()

# #############################################
# Targets import

if(NOT TARGET HeexCom::HeexCom)
  # #############################################
  # Path works for import. Mostly relative to Heex.

  # Check mandatory variable to be set
  if(NOT DEFINED HEEX_SDK_DIR)
    # HEEXCOM_CMAKE_DIR points to the libraries/HeexCom/build folder. We compute the realpath for two parent directories above.
    get_filename_component(HEEX_SDK_DIR "${HEEXCOM_CMAKE_DIR}/../../.." REALPATH)
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

  include("${HEEXCOM_CMAKE_DIR}/FindHeexComDeps.cmake")
  include("${HEEXCOM_CMAKE_DIR}/HeexComTargets.cmake")
endif()

set(HEEXCOM_lIBRARIES HeexCom::HeexCom)
