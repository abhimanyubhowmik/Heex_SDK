# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget HeexCom::TcpServer HeexCom::TcpClient)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
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


# Create imported target HeexCom::TcpServer
add_library(HeexCom::TcpServer STATIC IMPORTED)

set_target_properties(HeexCom::TcpServer PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom"
  INTERFACE_LINK_LIBRARIES "/home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_thread.a;-lpthread;/home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_chrono.a;/home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_date_time.a;/home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_atomic.a;HeexUtils::HeexUtils"
)

# Create imported target HeexCom::TcpClient
add_library(HeexCom::TcpClient STATIC IMPORTED)

set_target_properties(HeexCom::TcpClient PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom"
  INTERFACE_LINK_LIBRARIES "/home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_thread.a;-lpthread;/home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_chrono.a;/home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_date_time.a;/home/abhimanyu/Heex_SDK_2_39_1/3rdparty/boost_1_75_0/stage/lib/libboost_atomic.a;HeexUtils::HeexUtils"
)

# Import target "HeexCom::TcpServer" for configuration "Release"
set_property(TARGET HeexCom::TcpServer APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(HeexCom::TcpServer PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "/home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/libTcpServer.a"
  )

# Import target "HeexCom::TcpClient" for configuration "Release"
set_property(TARGET HeexCom::TcpClient APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(HeexCom::TcpClient PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "/home/abhimanyu/Heex_SDK_2_39_1/libraries/HeexCom/libTcpClient.a"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
