/// Copyright (c) 2024 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "SampleRos2BooleanMonitor.h"

#include <chrono>
#include <sstream>

#include "HeexUtilsLog.h"
#include "ros2SampleUtils.hpp"

// CONSTANTS
static constexpr const char* MONITOR_NODE_NAME = "ros2_edge_in_memory_buffer_v2_monitorNode";

SampleRos2BooleanMonitor::SampleRos2BooleanMonitor(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort)
    : BooleanMonitor(uuid, serverIp, serverPort)
{
  _node                       = rclcpp::Node::make_shared(MONITOR_NODE_NAME);
  std::string topicName       = "/demo/bool";
  rclcpp::QoS topicQosProfile = rclcpp::QoS{10}; // If your topic uses a specific QoS, you need to update it here accordingly.

  __detectionSub =
      _node->create_subscription<std_msgs::msg::Bool>(topicName, topicQosProfile, std::bind(&SampleRos2BooleanMonitor::booleanMsgCallback, this, std::placeholders::_1));
}

void SampleRos2BooleanMonitor::booleanMsgCallback(const std::shared_ptr<std_msgs::msg::Bool> msg)
{
  // Get the content of the std_msgs::Bool message.
  // Use current time , not the timestamp of the message (As not all messages contain a timestamp in their content, we use current time as reception time)
  rclcpp::Clock rclcpp_clock;
  const rclcpp::Time current_time    = rclcpp_clock.now();
  const std::string iso_extended_now = ros2_sample_utils::rclcpptime_to_iso_extended_str(current_time);

  // Outputs for example purpose: Monitor the engagement signal
  if (msg->data)
  {
    HEEX_LOG(info) << "I heard: True" << std::endl;
  }
  else
  {
    HEEX_LOG(info) << "I heard: False" << std::endl;
  }

  // Detection logic
  // Transform sysclock timestamp to iso_extended format and use updateValue to notify monitor of a new value.
  this->updateValue(msg->data, iso_extended_now);
}

void SampleRos2BooleanMonitor::awaitReady()
{
  HEEX_LOG(info) << "Waiting to connect to kernel" << std::endl;
  while (!this->isReady() || !rclcpp::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  HEEX_LOG(info) << "Connected to kernel." << std::endl;
}
