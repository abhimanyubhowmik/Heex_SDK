# Copyright 2023 Heex Technologies.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)

project(apex_ai_heex_demo_pkg)

find_package(ament_cmake_auto REQUIRED)
find_package(apex_cmake REQUIRED)
ament_auto_find_build_dependencies()

# #############################################
# Copy the section below to import Heex library dependencies.

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

ament_auto_add_library(monitorSubscriber src/monitor_node.cpp)
ament_auto_add_library(recorderSubscriber src/recorder_node.cpp)

ament_auto_add_executable(apex_ai_heex_demo_main_exe src/main.cpp)

apex_set_compile_options(monitorSubscriber recorderSubscriber apex_ai_heex_demo_main_exe)

target_link_libraries(monitorSubscriber HeexCustomerSide::HeexCustomerSide)
target_link_libraries(recorderSubscriber HeexCustomerSide::HeexCustomerSide)

ament_auto_package()
