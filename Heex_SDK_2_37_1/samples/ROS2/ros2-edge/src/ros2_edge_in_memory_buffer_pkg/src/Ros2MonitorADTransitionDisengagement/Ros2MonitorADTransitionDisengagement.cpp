/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include "Ros2MonitorADTransitionDisengagement.h"

#include <chrono>
#include <sstream>

// CONSTANTS
static const std::string MONITOR_NODE_NAME = "ros2_edge_in_memory_buffer_monitorNode";

Ros2MonitorADTransitionDisengagement::Ros2MonitorADTransitionDisengagement(const std::string& serverIp, const unsigned int& serverPort)
    : BooleanMonitor("M-XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX(1.0.0)", serverIp, serverPort)
{
  _node                       = rclcpp::Node::make_shared(MONITOR_NODE_NAME);
  std::string topicName       = "/demo/bool";
  rclcpp::QoS topicQosProfile = rclcpp::QoS{10}; // If your topic uses a specific QoS, you need to update it here accordingly.

  __detectionSub = _node->create_subscription<std_msgs::msg::Bool>(
      topicName, topicQosProfile, std::bind(&Ros2MonitorADTransitionDisengagement::booleanMsgCallback, this, std::placeholders::_1));
}

void Ros2MonitorADTransitionDisengagement::booleanMsgCallback(const std::shared_ptr<std_msgs::msg::Bool> msg)
{
  // Get the content of the std_msgs::Bool message.
  // Use current time , not the timestamp of the message (As not all messages contain a timestamp in their content, we use current time as reception time)
  rclcpp::Clock rclcpp_clock;
  rclcpp::Time current_time = rclcpp_clock.now();
  std::string iso_now       = ros2_sample_utils::rclcpptime_to_iso_extended_str(current_time);

  // Outputs for example purpose: Monitor the engagement signal
  if (msg->data)
  {
    RCLCPP_INFO(_node->get_logger(), "I heard: True (engaged)");
  }
  else
  {
    RCLCPP_INFO(_node->get_logger(), "I heard: False (disengaged)");
  }

  // Detection logic
  // Transform sysclock timestamp to iso_extended format and use updateValue to notify monitor of a new value.
  // Use inverse value to monitor the disengagement signal
  this->updateValue(!msg->data, iso_now);
}

void Ros2MonitorADTransitionDisengagement::awaitReady()
{
  RCLCPP_INFO(_node->get_logger(), "Waiting for kernel to be ready ...");
  while (this->isReady() == false && rclcpp::ok())
  {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_INFO(_node->get_logger(), "Connected to kernel.");
}
