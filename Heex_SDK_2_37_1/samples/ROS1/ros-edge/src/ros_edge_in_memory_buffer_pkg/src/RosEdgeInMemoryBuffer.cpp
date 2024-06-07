///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file RosEdgeInMemoryBuffer.cpp
/// @author Matthieu Carré (matthieu@heex.io)
/// @brief Main ROS node that runs both the Disengagement Monitor and the Snapshotter Recorder as a single ROS node.
/// @version 0.2
/// @date 2023-10-06
#include <ros/ros.h>

#include "MonitorADTransitionDisengagement.h"
#include "RosbagRecorderSnapshotter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_edge_in_memory_buffer_node");
  ros::NodeHandle nh;

  // Instanciate Monitor that interfaces with ROS for topic "/demo/bool"
  MonitorADTransitionDisengagement monitorBagDisengagement(&nh, "127.0.0.1", 4242);

  // Instanciate Recorder that interfaces with ROS for topic "/demo/bool"
  RosbagRecorderSnapshotter rosbagRecorderSnapshotter(&nh, "127.0.0.1", 4243);

  // Wait for the Monitor to be fully configured and ready to operate
  monitorBagDisengagement.awaitReady();

  // Run the nodes
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin();                       // spin() will not return until the node has been shutdown

  return EXIT_SUCCESS;
}
