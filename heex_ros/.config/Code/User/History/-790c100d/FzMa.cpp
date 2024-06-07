/// Copyright (c) 2023 Heex Technologies
/// All Rights Reserved.
///
/// This file is subject to the terms and conditions defined in
/// file 'LICENSE', which is part of this source code package.

#include <boost/bind/bind.hpp>
using namespace boost::placeholders;
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "MonitorADTransitionDisengagement.h"

MonitorADTransitionDisengagement::MonitorADTransitionDisengagement(ros::NodeHandle* nh, const std::string& serverIp, const unsigned int& serverPort)
    : BooleanMonitor("D-c3e3f4ed-e3b9-4d39-921f-2e6e1c7834ca(1.0.0)", serverIp, serverPort)
{
  __detectionSub = nh->subscribe("/demo/bool", 1000, &MonitorADTransitionDisengagement::booleanMsgCallback, this);
}

void MonitorADTransitionDisengagement::booleanMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
  // Get the content of the std_msgs::Bool message.
  // Use current time , not the timestamp of the message (As not all messages contain a timestamp in their content, we use current time as reception time)
  ros::Time rosGetTime = ros::Time::now();

  // Detection logic
  // Transform ROS timestamp to iso_extended format and use updateValue to
  // notify monitor of a new value. Use inverse value to monitor the
  // disengagement signal
  std::string ts = boost::posix_time::to_iso_extended_string(rosGetTime.toBoost());
  this->updateValue(!msg->data, ts);
}

void MonitorADTransitionDisengagement::awaitReady()
{
  while (this->isReady() == false && ros::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
