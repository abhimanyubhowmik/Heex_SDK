/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

/// @file Ros2EdgeInMemoryBuffer.cpp
/// @author Simon Demmer (simon@heex.io)
/// @brief Main ROS2 node that runs both the Disengagement Monitor and the Snapshotter Recorder as a single ROS2 node.
/// @version 0.2
/// @date 2023-10-06
#include "Ros2MonitorADTransitionDisengagement.h"
#include "Rosbag2RecorderSnapshotter.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Instanciate Monitor that interfaces with ROS for topic "/demo/bool"
  Ros2MonitorADTransitionDisengagement monitorBagDisengagement("127.0.0.1", 4242);
  // Instanciate Recorder that interfaces with ROS for topic "/demo/bool"
  Rosbag2RecorderSnapshotter rosbag2RecorderSnapshotter("127.0.0.1", 4243);

  // Wait for kernel connection
  monitorBagDisengagement.awaitReady();
  rosbag2RecorderSnapshotter.awaitReady();

  // Run the nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(monitorBagDisengagement.get_node());
  executor.add_node(rosbag2RecorderSnapshotter.get_node());
  executor.add_node(rosbag2RecorderSnapshotter.get_node()->get_single_msg_caches_node());
  while (rclcpp::ok())
  {
    executor.spin_some(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
