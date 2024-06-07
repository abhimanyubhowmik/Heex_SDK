/// Copyright (c) 2024 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

// Type(s) of message(s) the Monitor shall use (required when deserialized to read data)
#include "std_msgs/msg/bool.hpp"

// Type of Monitor choosen from the Heex SDK library. Can be custom as well.
#include "BooleanMonitor.h"

class SampleRos2BooleanMonitor : public BooleanMonitor
{
public:
  /// @brief Construct a new SampleRos2BooleanMonitor object
  ///
  /// @param uuid
  /// @param serverIp
  /// @param serverPort
  SampleRos2BooleanMonitor(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort);

  /// @brief Destroy the SampleRos2BooleanMonitor object
  ///
  ~SampleRos2BooleanMonitor() = default;

  /// @brief Override the awaitReady to make the node responsive to ros status and controls while pending connection with the Core
  ///
  virtual void awaitReady() override;

  /// @brief returns the monitor node
  ///
  /// @return _node - std::shared_ptr<rclcpp::Node>
  std::shared_ptr<rclcpp::Node> getMonitorNode() { return _node; };

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
