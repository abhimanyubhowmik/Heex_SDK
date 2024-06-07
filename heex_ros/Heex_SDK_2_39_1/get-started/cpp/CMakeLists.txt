cmake_minimum_required(VERSION 3.10.2) # Preferred cmake-3.16.3

# Note:
#   This CMakeLists.txt is built with the HeexSDK to generate examples.
#   It may introduced more code that necessary for a fresh extern project.
#   If you plan to reuse this project outside of the heex-sdk folder, please follow the instructions in the ADAPTATION NOTE of each section.

# #############################################
# Path works for build. Mostly relative to Heex.

# HEEX_SDK_DIR section: The following lines set the value of the HEEX_SDK_DIR as the root directory of the Heex SDK.
if(HEEX_SDK_DIR)
  message(STATUS "Heex SDK path set to: ${HEEX_SDK_DIR}")
else()
  get_filename_component(HEEX_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." REALPATH)
  message(STATUS "HEEX_SDK_DIR variable isn't set. Auto Setting HEEX_SDK_DIR=${HEEX_SDK_DIR}")
endif()
## ADAPTATION NOTE: Remove the previous HEEX_SDK_DIR code section above and uncomment the line below.
## You may alternatively set the following variable directly from cmake using "-DHEEX_SDK_DIR:STRING=~/heex/" in your cmake command or from a defined system variable using "$ENV{HEEX_SDK_DIR}"
# set(HEEX_SDK_DIR "~/heex/")

# #############################################
# Set version and project info

# CMAKE_PROJECT section: The following lines set the project settings and version to the SDK version for this example.
get_filename_component(HEEX_SDK_DIR "${HEEX_SDK_DIR}" REALPATH)  # resolve any path issue
file (STRINGS "${HEEX_SDK_DIR}/version" HEEX_BUILD_VERSION)
project("get-started"
  VERSION ${HEEX_BUILD_VERSION}
  LANGUAGES CXX
  DESCRIPTION "Example of the cross-platform SDK integration and primary usage to implement a Monitor and a Recorder."
)
message(STATUS "${CMAKE_PROJECT_NAME} version: ${CMAKE_PROJECT_VERSION}")
message(STATUS "Using Heex SDK version: ${HEEX_BUILD_VERSION}")
## ADAPTATION NOTE: Replace the HEEX_BUILD_VERSION in line "VERSION ${HEEX_BUILD_VERSION}"" by the value of your choosing.

# The following lines set the output for library and execs to main project folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Debug/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Debug/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Debug/)

# #############################################
# Copy the section below to import Heex library dependencies.
# Setting HEEX_BOOST_VERSION is optional.

# HEEX_BOOST_VERSION section: Set the Boost version required to build the Heex SDK.
if(NOT DEFINED HEEX_BOOST_VERSION)
  set(HEEX_BOOST_VERSION 1.75.0)
  if(legacy1804HeexSdk)
    set(HEEX_BOOST_VERSION 1.65.1)
  endif()
endif()
## ADAPTATION NOTE: Remove the previous HEEX_BOOST_VERSION code section above and uncomment the line below
## You may alternatively set the following variable directly from cmake using "-DHEEX_BOOST_VERSION:STRING=1.75.0" in your cmake command.
# set(HEEX_BOOST_VERSION 1.75.0)

set(HEEX_BUILD_DIR "build/")
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(HEEX_BUILD_DIR "debugBuild/")
endif()

# Find HeexCustomerSide library.
# HeexCustomerSide section: Import the C++ Heex SDK and its libraries
# For a facilitated import, HeexCustomerSide requires HEEX_SDK_DIR to be set.
find_package(HeexCustomerSide ${HEEX_BUILD_VERSION} REQUIRED PATHS ${HEEX_SDK_DIR}/sdk/${HEEX_BUILD_DIR} NO_DEFAULT_PATH)
message(STATUS "HeexCustomerSide_DIR=${HeexCustomerSide_DIR}")

# #############################################


# Main executables
add_executable(GetStartedMonitor getStartedMonitor.cpp)
add_executable(GetStartedRecorder getStartedRecorder.cpp)
target_link_libraries(GetStartedMonitor PRIVATE HeexCustomerSide::HeexCustomerSide)
target_link_libraries(GetStartedRecorder PRIVATE HeexCustomerSide::HeexCustomerSide)
# Support Windows 7 or later
target_compile_definitions(GetStartedMonitor PRIVATE "$<$<CXX_COMPILER_ID:MSVC>:_WIN32_WINNT=0x0601;>")
target_compile_definitions(GetStartedRecorder PRIVATE "$<$<CXX_COMPILER_ID:MSVC>:_WIN32_WINNT=0x0601;>")
