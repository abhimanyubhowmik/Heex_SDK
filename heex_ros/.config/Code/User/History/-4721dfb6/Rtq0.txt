cmake_minimum_required(VERSION 3.5) # Preferred cmake-3.16.3

# Note:
#   This CMakeLists.txt is built with the HeexSDK to generate examples.
#   It may introduced more code that necessary for a fresh extern project.
#   If you plan to reuse this project outside of the heex-sdk folder, please follow the instructions in the ADAPTATION NOTE of each section.

# #############################################
# Path works for build. Mostly relative to Heex.

# HEEX_SDK_DIR section: The following lines set the value of the HEEX_SDK_DIR as the root directory of the Heex SDK.
# if(HEEX_SDK_DIR)
#   message(STATUS "Heex SDK path set to: ${HEEX_SDK_DIR}")
# else()
#   message(FATAL_ERROR "HEEX_SDK_DIR variable isn't set. Consider setting it when building your catkin workspace using 'catkin_make -DHEEX_SDK_DIR=pathToHeexSDKdir'cmake flag. You can also hard set it in this CMakeLists.txt directly by uncommenting and modifying the 'set(HEEX_SDK_DIR ~/heex/)' line.")
# endif()
## ADAPTATION NOTE: Remove the previous HEEX_SDK_DIR code section above and uncomment the line below.
## You may alternatively set the following variable directly from cmake using "-DHEEX_SDK_DIR:STRING=~/heex/" in your cmake command or from a defined system variable using "$ENV{HEEX_SDK_DIR}"
set(HEEX_SDK_DIR "/home/abhimanyu/Heex_SDK_2_39_1/")

# #############################################
# Set version and project info

# CMAKE_PROJECT section: The following lines set the project settings and version to the SDK version for this example.
get_filename_component(HEEX_SDK_DIR "${HEEX_SDK_DIR}" REALPATH)  # resolve any path issue
file (STRINGS "${HEEX_SDK_DIR}/version" HEEX_BUILD_VERSION)
project("ros_edge_in_memory_buffer_pkg"
  VERSION ${HEEX_BUILD_VERSION}
  LANGUAGES CXX
  DESCRIPTION "Example of the SDK usage for ROS that implements Monitors and Recorders for the edge (online) use case."
)
message(STATUS "${CMAKE_PROJECT_NAME} version: ${CMAKE_PROJECT_VERSION}")
message(STATUS "Using Heex SDK version: ${HEEX_BUILD_VERSION}")
## ADAPTATION NOTE: Replace the HEEX_BUILD_VERSION in line "VERSION ${HEEX_BUILD_VERSION}"" by the value of your choosing.

# #############################################
# Catkin related cmake contents

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_edge_in_memory_buffer_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Import Rosbag C++ API and targets
find_package(rosbag)

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

# HeexCustomerSide section: Import the C++ Heex SDK and its libraries
# For a facilitated import, HeexCustomerSide requires HEEX_SDK_DIR to be set. HEEX_BOOST_VERSION is optional.
message(STATUS "HEEX_BUILD_VERSION=${HEEX_BUILD_VERSION}")
message(STATUS "HEEX_SDK_DIR=${HEEX_SDK_DIR}")
message(STATUS "path=${HEEX_SDK_DIR}/sdk/${HEEX_BUILD_DIR}")
find_package(HeexCustomerSide ${HEEX_BUILD_VERSION} REQUIRED PATHS ${HEEX_SDK_DIR}/sdk/${HEEX_BUILD_DIR} NO_DEFAULT_PATH)
message(STATUS "HeexCustomerSide_DIR=${HeexCustomerSide_DIR}")

##############################################
# Create target and set properties

# Add all subprojects
add_subdirectory(src/MonitorADTransitionDisengagement)
add_subdirectory(src/RosbagRecorderSnapshotter)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(rosEdgeInMemoryBuffer src/RosEdgeInMemoryBuffer.cpp ${SOURCE_MONITOR_DISENGAGEMENT} ${SOURCE_RECORDER_ROSBAGRECORDERSNAPSHOTTER})
target_include_directories(rosEdgeInMemoryBuffer
  PRIVATE
    src/MonitorADTransitionDisengagement
    src/RosbagRecorderSnapshotter
    ${rosbag_INCLUDE_DIRS}
)
## Specify libraries to link a library or executable target against
target_link_libraries(rosEdgeInMemoryBuffer
  ${catkin_LIBRARIES}
  # HeexCustomerSide HeexMessages TcpClient HeexConfig HeexUtils ${Boost_LIBRARIES} Threads::Threads
  HeexCustomerSide::HeexCustomerSide
  ${rosbag_LIBRARIES}
)
