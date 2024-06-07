/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ros2SampleUtils.h"

// Type(s) of message(s) the Monitor shall use (required when deserialized to read data)
#include "std_msgs/msg/bool.hpp"

// Type of Monitor choosen from the Heex SDK library. Can be custom as well.
#include "BooleanMonitor.h"

class Ros2MonitorADTransitionDisengagement : public BooleanMonitor
{
public:
  /// @brief Construct a new Ros2MonitorADTransitionDisengagement object
  ///
  /// @param nh ROS node handle pointer
  /// @param uuid
  /// @param serverIp
  /// @param serverPort
  Ros2MonitorADTransitionDisengagement(const std::string& serverIp, const unsigned int& serverPort);

  /// @brief Override the awaitReady to make the node responsive to ros status and controls while pending connection with the Core
  virtual void awaitReady() override;

  /// @brief returns the monitor node
  ///
  /// @return _node - std::shared_ptr<rclcpp::Node>
  std::shared_ptr<rclcpp::Node> get_node() { return _node; };

private:
  /// @brief Callback in which the monitor perform the detection logic
  ///
  /// @param msg Message of type std_msgs::Bool. Require to include std_msgs/Bool.h
  void booleanMsgCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);

  //
  // Attributes
  //
  std::shared_ptr<rclcpp::Node> _node;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> __detectionSub; /// Subscriber the monitor deploys
};
