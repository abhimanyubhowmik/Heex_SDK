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
  set(HEEX_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
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
project("ros-datalake"
  VERSION ${HEEX_BUILD_VERSION}
  LANGUAGES CXX
  DESCRIPTION "Example of the SDK usage for ROS that implements Monitors and Recorders for a datalake rosbag extraction (offline) use case."
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
# Find Heex library dependencies.

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
# For a facilitated import, HeexCustomerSide requires HEEX_SDK_DIR to be set. HEEX_BOOST_VERSION is are optional.
find_package(HeexCustomerSide ${HEEX_BUILD_VERSION} REQUIRED PATHS ${HEEX_SDK_DIR}/sdk/${HEEX_BUILD_DIR} NO_DEFAULT_PATH)
message(STATUS "HeexCustomerSide_DIR=${HeexCustomerSide_DIR}")

##############################################
# Create target and set properties

# Add all subprojects
add_subdirectory(MonitorADTransitionDisengagement)
add_subdirectory(RecorderRosbagDataLakeExtractor)

# This main executable involves:
# - Datalake extraction from rosbag: ${rosbag_LIBRARIES}
# - SDK CustomerSide for Monitors and Recorders: HeexCustomerSide with its SDK Librairies and dependencies: HeexMessages TcpClient HeexConfig HeexUtils and Boost libraries ${Boost_LIBRARIES}
add_executable(heexExampleRosDataLakeExtraction main.cpp ${SOURCE_MONITOR_DISENGAGEMENT} ${SOURCE_RECORDER_ROSBAGDATAEXTRACTOR})

## Set target properties
target_include_directories(heexExampleRosDataLakeExtraction
  PRIVATE
    ${Boost_INCLUDE_DIRS} # Need to be before rosbag to force chosen boost version
    ${rosbag_INCLUDE_DIRS}
    /opt/ros/noetic/include/std_msgs # ROS message headers (std_msgs::Bool)
    # Uncomment & Adapt the following line for each of your custom ROS message (myMessage.h). These headers are required for compilation.
    # ~/catkin_ws/devel/include/myMessage.h
)

target_link_libraries(heexExampleRosDataLakeExtraction
  PRIVATE
    HeexCustomerSide::HeexCustomerSide
    ${Boost_LIBRARIES}
    ${rosbag_LIBRARIES}
    Threads::Threads
)
