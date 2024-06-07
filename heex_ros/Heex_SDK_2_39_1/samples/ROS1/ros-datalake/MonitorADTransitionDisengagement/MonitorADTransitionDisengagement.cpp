///
/// Copyright (c) 2022 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

///
/// @file MonitorADTransitionDisengagement.cpp
/// @author Matthieu Carré (matthieu@heex.io)
/// @brief Header file for the Monitor on ADTransitionDisengagement for the rosbag offline extraction.
/// @version 0.1
/// @date 2022-04-21
#include "MonitorADTransitionDisengagement.h"

MonitorADTransitionDisengagement::MonitorADTransitionDisengagement(const std::string& uuid, const std::string& serverIp, const unsigned int& serverPort)
    : BooleanMonitor(uuid, serverIp, serverPort)
{
}

void MonitorADTransitionDisengagement::computeNewBooleanMsg(const std_msgs::Bool::ConstPtr& msg, const std::string& ts)
{
  // Outputs for example purpose: Monitor the engagement signal
  if (msg->data)
  {
    HEEX_LOG(info) << "[Bag Time: " << ts << "] I heard: True (engaged)";
  }
  else
  {
    HEEX_LOG(info) << "[Bag Time: " << ts << "] I heard: False (disengaged)";
  }

  // Detection logic
  // Use inverse value to monitor the disengagement signal
  this->updateValue(!msg->data, ts);
}
