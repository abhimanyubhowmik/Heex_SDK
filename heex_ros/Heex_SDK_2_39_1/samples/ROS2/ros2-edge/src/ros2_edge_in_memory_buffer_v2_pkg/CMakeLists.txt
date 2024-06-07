cmake_minimum_required(VERSION 3.5) # Preferred cmake-3.16.3
# Note:
#   This CMakeLists.txt is built with the HeexSDK to generate examples.
#   It may introduced more code that necessary for a fresh extern project.
#   Please add the HEEX_SDK_DIR as an argument when building sample, or follow the instructions in the ADAPTATION NOTES sections from this file:
#         colcon build --ament-cmake-args -DHEEX_SDK_DIR=/path/to/heex-sdk-dir

# #############################################
# Path works for build. Mostly relative to Heex.

if(HEEX_SDK_DIR)
    add_definitions(-DHEEX_SDK_DIR=${HEEX_SDK_DIR})
    message(STATUS "Heex SDK path set to: ${HEEX_SDK_DIR}")
else()
    message(FATAL_ERROR "HEEX_SDK_DIR variable isn't set. Consider setting it when building using 'colcon build --ament-cmake-args -DHEEX_SDK_DIR=/path/to/heex-sdk-dir' cmake flag. You can also hard set it in this CMakeLists.txt directly by uncommenting and modifying the 'set(HEEX_SDK_DIR ~/heex/)' line.")
endif()
## ADAPTATION NOTE: Remove the previous HEEX_SDK_DIR code section above and uncomment the line below.
## You may alternatively set the following variable directly from by using "--ament-cmake-args -DHEEX_SDK_DIR=/path/to/heex-sdk-dir" when running your colcon build command or from a defined system variable using "$ENV{HEEX_SDK_DIR}"
# set(HEEX_SDK_DIR "~/heex/")

# #############################################
# Set version and project info

# CMAKE_PROJECT section: The following lines set the project settings and version to the SDK version for this example.
get_filename_component(HEEX_SDK_DIR "${HEEX_SDK_DIR}" REALPATH)  # resolve any path issue
file (STRINGS "${HEEX_SDK_DIR}/version" HEEX_BUILD_VERSION)
project("ros2_edge_in_memory_buffer_v2_pkg"
  VERSION ${HEEX_BUILD_VERSION}
  LANGUAGES CXX
  DESCRIPTION "Example of the SDK usage for ROS2 that implements Monitors and Recorders for the edge (online) use case."
)
message(STATUS "${CMAKE_PROJECT_NAME} version: ${CMAKE_PROJECT_VERSION}")
message(STATUS "Using Heex SDK version: ${HEEX_BUILD_VERSION}")
## ADAPTATION NOTE: Replace the HEEX_BUILD_VERSION in line "VERSION ${HEEX_BUILD_VERSION}"" by the value of your choosing.

# #############################################
# ament_cmake related cmake contents

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/ros2_edge_in_memory_buffer_v2_pkg
)

# HEEX_BOOST_VERSION section: Set the Boost version required to build the Heex SDK.
set(BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG ON)
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

# HeexCustomerSide section: Import the C++ Heex SDK and its libraries
# For a facilitated import, HeexCustomerSide requires HEEX_SDK_DIR to be set. HEEX_BOOST_VERSION is optional.
find_package(HeexCustomerSide ${HEEX_BUILD_VERSION} REQUIRED PATHS ${HEEX_SDK_DIR}/sdk/${HEEX_BUILD_DIR} NO_DEFAULT_PATH)
message(STATUS "HeexCustomerSide_DIR=${HeexCustomerSide_DIR}")

##############################################
# Create target and set properties

# Add all subprojects
add_subdirectory(src/SampleRos2BooleanMonitor)
add_subdirectory(src/SampleRos2BagRecorder)

ament_auto_add_executable(ros2EdgeInMemoryBufferV2 src/Ros2EdgeInMemoryBufferV2.cpp ${SOURCE_RECORDER_SampleRos2BagRecorder} ${SOURCE_MONITOR_DISENGAGEMENT})

## Specify libraries to link a library or executable target against
target_link_libraries(ros2EdgeInMemoryBufferV2
    HeexCustomerSide::HeexCustomerSide
    HeexUtils::HeexUtils
)

ament_auto_package()
