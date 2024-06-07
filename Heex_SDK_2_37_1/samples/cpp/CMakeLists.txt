cmake_minimum_required(VERSION 3.10.2) # Preferred cmake-3.16.3

project(HeexSDKcppSamples)

# #############################################
# Path works for build. Mostly relative to Heex.

# Set relatively the Heex librairies and 3rd party dirs
if(NOT DEFINED HEEX_SDK_DIR)
  get_filename_component(HEEX_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." REALPATH)
  message(STATUS "Auto Setting Heex SDK path to HEEX_SDK_DIR=${HEEX_SDK_DIR}")
else()
  # Ensure that the provided path of the Heex SDK is coherent with the current package path. Reveal find package silent errors due to guesses.
  # We make sure to resolve any symbolic link that may failed the check
  get_filename_component(HEEX_SDK_DIR "${HEEX_SDK_DIR}" REALPATH)
  get_filename_component(HEEX_SDK_DIR_CURRENT "${CMAKE_CURRENT_SOURCE_DIR}/../.." REALPATH)
  if(NOT HEEX_SDK_DIR STREQUAL HEEX_SDK_DIR_CURRENT)
    message(FATAL_ERROR "Heex SDK path doesn't match: Provided ${HEEX_SDK_DIR} , Current: ${HEEX_SDK_DIR_CURRENT} . This error is encountered if you have multiple Heex SDK installed. Please define HEEX_SDK_DIR and then import using find_package(HeexCustomerSide ${HEEX_BUILD_VERSION} REQUIRED PATHS ${HEEX_SDK_DIR}/sdk/build NO_DEFAULT_PATH)")
  else()
    message(STATUS "Heex SDK path set to: ${HEEX_SDK_DIR}")
  endif()
endif()

# #############################################
# Set version and project info

file (STRINGS "${HEEX_SDK_DIR}/version" HEEX_BUILD_VERSION)

set(HEEX_BUILD_DIR "build/")
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(HEEX_BUILD_DIR "debugBuild/")
endif()

##############################################
# find HeexCustomerSide and set properties

find_package(HeexCustomerSide ${HEEX_BUILD_VERSION} REQUIRED PATHS ${HEEX_SDK_DIR}/sdk/${HEEX_BUILD_DIR} NO_DEFAULT_PATH)
message(STATUS "HeexCustomerSide_DIR=${HeexCustomerSide_DIR}")

##############################################
# Create target and set properties


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}") # for now, we want the executables to be at same location as the source files. This should be changed in the future
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}") # for now, we want the executables to be at same location as the source files. This should be changed in the future

# Monitors
add_executable(basicSampleMonitor_Boolean basicSampleMonitor_Boolean.cpp)
target_link_libraries(basicSampleMonitor_Boolean
  PRIVATE 
    HeexCustomerSide::HeexCustomerSide
)
set_target_properties(basicSampleMonitor_Boolean PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(basicSampleMonitor_Interval basicSampleMonitor_Interval.cpp)
target_link_libraries(basicSampleMonitor_Interval
  PRIVATE 
    HeexCustomerSide::HeexCustomerSide
)
set_target_properties(basicSampleMonitor_Interval PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(basicSampleMonitor_String basicSampleMonitor_String.cpp)
target_link_libraries(basicSampleMonitor_String
  PRIVATE 
    HeexCustomerSide::HeexCustomerSide
)
set_target_properties(basicSampleMonitor_String PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(basicSampleMonitor_Threshold basicSampleMonitor_Threshold.cpp)
target_link_libraries(basicSampleMonitor_Threshold
  PRIVATE 
    HeexCustomerSide::HeexCustomerSide
)
set_target_properties(basicSampleMonitor_Threshold PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(basicSampleMonitor_Zone basicSampleMonitor_Zone.cpp)
target_link_libraries(basicSampleMonitor_Zone
  PRIVATE 
    HeexCustomerSide::HeexCustomerSide
)
set_target_properties(basicSampleMonitor_Zone PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(featureSampleMonitor_ConfChangedCallback featureSampleMonitor_ConfChangedCallback.cpp)
target_link_libraries(featureSampleMonitor_ConfChangedCallback
  PRIVATE 
    HeexCustomerSide::HeexCustomerSide
)
set_target_properties(featureSampleMonitor_ConfChangedCallback PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Recorders
add_executable(basicSampleRecorder basicSampleRecorder.cpp)
target_link_libraries(basicSampleRecorder
  PRIVATE 
    HeexCustomerSide::HeexCustomerSide
    ${Boost_FILESYSTEM_LIBRARY}
)
set_target_properties(basicSampleRecorder PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(featureSampleRecorder_ConfChangedCallback featureSampleRecorder_ConfChangedCallback.cpp)
target_link_libraries(featureSampleRecorder_ConfChangedCallback
  PRIVATE 
    HeexCustomerSide::HeexCustomerSide
)
set_target_properties(featureSampleRecorder_ConfChangedCallback PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set the cpp samples target properties

## Compile options
set(HEEX_CPP_SAMPLES_COMPILE_OPTIONS
  # Add default compile flags for GNU gcc for All builds
  "$<$<CXX_COMPILER_ID:GNU>:-Wextra;-Wall;-W;-pedantic;-std=c++14;>"
  # Add default compile flags for GNU gcc for Release build
  "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:RELEASE>>:-O3;>"
  # Add default compile flags for GNU gcc for Debug build
  "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:DEBUG>>:-g3;-O0;>"
  # Disable warning C4127 causes by HEEX_LOG. if constexpr is not available in c++14
  "$<$<CXX_COMPILER_ID:MSVC>:/wd4127;>"
  # Set default compile flags for Windows MSVC for Release build
  "$<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:RELEASE>>:/MD;/O2;/Ob2;/DNDEBUG;/W4;>"
  # Set default compile flags for Windows MSVC for Debug build
  "$<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:DEBUG>>:/MDd;/Od;/Ob0;/Zi;/RTC1;/DDEBUG;/D_DEBUG;>"
)

set(HEEX_CPP_SAMPLES_COMPILE_DEFINITIONS
  # Define macro DEBUG_BUILD for Debug build
  "$<$<CONFIG:DEBUG>:DEBUG_BUILD;>"
  # Set target platform to be win7+
  "$<$<CXX_COMPILER_ID:MSVC>:_WIN32_WINNT=0x0601;>"
  # Ask Boost to report information about automatic linking requests (Windows-only)
  "$<$<CXX_COMPILER_ID:MSVC>:Boost_LIB_DIAGNOSTIC_DEFINITIONS;>"
)

# Set the targets defined in the samples/cpp directory with the same COMPILE_OPTIONS and COMPILE_DEFINITIONS (private) properties as SDK.
set_property(DIRECTORY APPEND PROPERTY COMPILE_OPTIONS ${HEEX_CPP_SAMPLES_COMPILE_OPTIONS})
set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS ${HEEX_CPP_SAMPLES_COMPILE_DEFINITIONS})
