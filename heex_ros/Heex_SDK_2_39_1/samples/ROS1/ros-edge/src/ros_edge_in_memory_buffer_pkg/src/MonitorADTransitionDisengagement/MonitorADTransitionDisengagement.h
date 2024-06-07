///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.
#pragma once

#include <ros/ros.h>

#include <string>

// Type(s) of message(s) the Monitor shall use (required when deserialized to read data)
#include <std_msgs/Bool.h>

// Type of Monitor choosen from the Heex SDK library. Can be custom as well.
#include "BooleanMonitor.h"

class MonitorADTransitionDisengagement : public BooleanMonitor
{
public:
  /// @brief Construct a new MonitorADTransitionDisengagement object
  ///
  /// @param nh ROS node handle pointer
  /// @param uuid
  /// @param serverIp
  /// @param serverPort
  MonitorADTransitionDisengagement(ros::NodeHandle* nh, const std::string& serverIp, const unsigned int& serverPort);

  /// @brief Override the awaitReady to make the node responsive to ros status and controls while pending connection with the Core
  ///
  virtual void awaitReady() override;

private:
  /// @brief Callback in which the monitor perform the detection logic
  ///
  /// @param msg Message of type std_msgs::Bool. Require to include std_msgs/Bool.h
  void booleanMsgCallback(const std_msgs::Bool::ConstPtr& msg);

  //
  // Attributes
  //
  ros::Subscriber __detectionSub; /// Subscriber the monitor deploys
};
