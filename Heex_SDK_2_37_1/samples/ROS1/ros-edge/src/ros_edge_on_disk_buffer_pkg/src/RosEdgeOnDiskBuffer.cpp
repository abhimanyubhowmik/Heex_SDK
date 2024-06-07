/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file RosEdgeOnDiskBuffer.cpp
/// @brief Main ROS node that runs both the Disengagement Monitor and the
/// RosBag Recorder as a single ROS node.
/// @version 0.2
/// @date 2023-10-06
#define BOOST_BIND_GLOBAL_PLACEHOLDERS // remove warning on the Bind \
                                       // placeholders for ROS
#include <ros/ros.h>

#include "MonitorADTransitionDisengagement.h"
#include "RosBagRecorder.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_edge_on_disk_buffer_node");
  ros::NodeHandle nh;

  // Instanciate Monitor that interfaces with ROS for topic "/demo/bool"
  MonitorADTransitionDisengagement monitorBagDisengagement(&nh, "127.0.0.1", 4242);

  RosBagRecorder rosbagRecorder("R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", "127.0.0.1", 4243);

  // Wait for the Monitor to be fully configured and ready to operate
  monitorBagDisengagement.awaitReady();

  // Run the nodes
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin();                       // spin() will not return until the node has been shutdown

  return EXIT_SUCCESS;
}
