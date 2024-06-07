///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file MonitorADTransitionDisengagement.h
/// @author Matthieu Carré (matthieu@heex.io)
/// @brief Header file for the Monitor on ADTransitionDisengagement for the rosbag offline extraction.
/// @version 0.1
/// @date 2022-04-21
#pragma once

// Type(s) of message(s) the Monitor shall use
#include <std_msgs/Bool.h>

// Type of Monitor choosen from the Heex SDK library. Can be custom as well.
#include "BooleanMonitor.h"

class MonitorADTransitionDisengagement : public BooleanMonitor
{
public:
  MonitorADTransitionDisengagement(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort);

  /// @brief Callback in which the Monitor perform the detection logic
  ///
  /// @param msg Message of type std_msgs::Bool. Require to include std_msgs/Bool.h
  void computeNewBooleanMsg(const std_msgs::Bool::ConstPtr& msg, const std::string& ts);
};
