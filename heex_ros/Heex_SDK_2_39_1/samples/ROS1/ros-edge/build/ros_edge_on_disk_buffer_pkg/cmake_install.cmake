# Install script for directory: /home/abhimanyu/Heex_SDK_2_39_1/samples/ROS1/ros-edge/src/ros_edge_on_disk_buffer_pkg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/abhimanyu/Heex_SDK_2_39_1/samples/ROS1/ros-edge/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/abhimanyu/Heex_SDK_2_39_1/samples/ROS1/ros-edge/build/ros_edge_on_disk_buffer_pkg/catkin_generated/installspace/ros_edge_on_disk_buffer_pkg.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_edge_on_disk_buffer_pkg/cmake" TYPE FILE FILES
    "/home/abhimanyu/Heex_SDK_2_39_1/samples/ROS1/ros-edge/build/ros_edge_on_disk_buffer_pkg/catkin_generated/installspace/ros_edge_on_disk_buffer_pkgConfig.cmake"
    "/home/abhimanyu/Heex_SDK_2_39_1/samples/ROS1/ros-edge/build/ros_edge_on_disk_buffer_pkg/catkin_generated/installspace/ros_edge_on_disk_buffer_pkgConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros_edge_on_disk_buffer_pkg" TYPE FILE FILES "/home/abhimanyu/Heex_SDK_2_39_1/samples/ROS1/ros-edge/src/ros_edge_on_disk_buffer_pkg/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/abhimanyu/Heex_SDK_2_39_1/samples/ROS1/ros-edge/build/ros_edge_on_disk_buffer_pkg/src/MonitorADTransitionDisengagement/cmake_install.cmake")
  include("/home/abhimanyu/Heex_SDK_2_39_1/samples/ROS1/ros-edge/build/ros_edge_on_disk_buffer_pkg/src/RosBagRecorder/cmake_install.cmake")

endif()

