
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was HeexCustomerSideConfig.cmake.in                            ########

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
get_filename_component(HEEXCUSTOMERSIDE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
message(STATUS "Found Heex SDK build in ${HEEXCUSTOMERSIDE_CMAKE_DIR}")
include(CMakeFindDependencyMacro)

list(APPEND CMAKE_MODULE_PATH ${HEEXCUSTOMERSIDE_CMAKE_DIR})

# #############################################
# Targets import

if(NOT TARGET HeexCustomerSide::HeexCustomerSide)
  # #############################################
  # Path works for import. Mostly relative for Heex.

  # Check mandatory variable to be set
  if(NOT DEFINED HEEX_SDK_DIR)
    # HEEXCUSTOMERSIDE_CMAKE_DIR points to the sdk/build folder. We compute the realpath for two parent directories above.
    get_filename_component(HEEX_SDK_DIR "${HEEXCUSTOMERSIDE_CMAKE_DIR}/../.." REALPATH)
    message(STATUS "Auto Setting Heex SDK path to HEEX_SDK_DIR=${HEEX_SDK_DIR}")
  else()
    # Ensure that the provided path of the Heex SDK is coherent with the current package path. Reveal find package silent errors due to guesses.
    # We make sure to resolve any symbolic link as they will failed the absolute path consistency check.
    get_filename_component(HEEX_SDK_DIR "${HEEX_SDK_DIR}" REALPATH)
    get_filename_component(HEEX_SDK_DIR_CURRENT "${HEEXCUSTOMERSIDE_CMAKE_DIR}/../.." REALPATH)
    if(NOT HEEX_SDK_DIR STREQUAL HEEX_SDK_DIR_CURRENT)
      message(FATAL_ERROR "Heex SDK path doesn't match: Provided ${HEEX_SDK_DIR} , Current: ${HEEX_SDK_DIR_CURRENT} . This error is encountered if you have multiple Heex SDK installed. Please define HEEX_SDK_DIR and then import using find_package(HeexCustomerSide ${HEEX_BUILD_VERSION} REQUIRED PATHS ${HEEX_SDK_DIR}/sdk/build NO_DEFAULT_PATH)")
    else()
      message(STATUS "Heex SDK path set to: ${HEEX_SDK_DIR}")
    endif()
  endif()

  if(NOT DEFINED HEEX_LIBRARIES_DIR)
    set(HEEX_LIBRARIES_DIR ${HEEX_SDK_DIR}/libraries)
    message(STATUS "Auto Setting HEEX_LIBRARIES_DIR=${HEEX_LIBRARIES_DIR}")
  endif()

  if(NOT DEFINED HEEX_3RDPARTY_DIR)
    set(HEEX_3RDPARTY_DIR ${HEEX_SDK_DIR}/3rdparty)
    message(STATUS "Auto Setting HEEX_3RDPARTY_DIR=${HEEX_3RDPARTY_DIR}")
  endif()

  include("${HEEXCUSTOMERSIDE_CMAKE_DIR}/FindHeexCustomerSideDeps.cmake")
  include("${HEEXCUSTOMERSIDE_CMAKE_DIR}/HeexCustomerSideTargets.cmake")
endif()

set(HEEXCUSTOMERSIDE_lIBRARIES HeexCustomerSide::HeexCustomerSide)
