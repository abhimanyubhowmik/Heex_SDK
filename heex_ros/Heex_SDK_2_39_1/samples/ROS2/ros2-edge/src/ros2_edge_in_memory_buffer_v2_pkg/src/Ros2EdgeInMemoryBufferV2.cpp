/// Copyright (c) 2024 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "SampleRos2BagRecorder.h"
#include "SampleRos2BooleanMonitor.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const std::string serverIp            = "127.0.0.1";
  const unsigned int monitorServerPort  = 4242;
  const unsigned int recorderServerPort = 4243;

  // Instanciate Monitor that interfaces with ROS for topic "/demo/bool"
  SampleRos2BooleanMonitor monitorBagDisengagement("M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, monitorServerPort);
  SampleRos2BagRecorder SampleRos2BagRecorder("R-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, recorderServerPort);

  // Wait for kernel connection
  monitorBagDisengagement.awaitReady();
  SampleRos2BagRecorder.awaitReady();

  // Run the nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(monitorBagDisengagement.getMonitorNode());
  executor.add_node(SampleRos2BagRecorder.getBagRecorderNode());
  while (rclcpp::ok())
  {
    executor.spin_some(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
